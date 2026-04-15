#include "avatar.hpp"
#include "common.hpp"
#include "network/udp_reliable.hpp"

#include <iostream>

Avatar::Avatar(const YAML::Node& config) {
    YAML::Node sys_config = YAML::LoadFile(config["robot_config"].as<std::string>());

    for (const auto& dev : sys_config["devices"]) {
        if (!dev["enabled"].as<bool>(true)) continue;
        std::string type = dev["type"].as<std::string>();

        if (type == "arm") {
            arm_instances.push_back(new ArmControl(dev));
        } else if (type == "head") {
            head_instances.push_back(new HeadControl(dev));
        }
    }

    if (sys_config["avatar"]["transmission"]) {
        UdpReliableConfig cmd_cfg;
        cmd_cfg.transport.remote_ip   = sys_config["avatar"]["transmission"]["remote_ip"].as<std::string>();
        cmd_cfg.transport.remote_port = sys_config["avatar"]["transmission"]["send_port"].as<int>();
        cmd_cfg.transport.bind_port   = sys_config["avatar"]["transmission"]["receive_port"].as<int>();
        cmd_cfg.poll_rate_hz          = sys_config["avatar"]["transmission"]["frequency"].as<int>();
        cmd_channel_ = std::make_unique<UdpReliable>(cmd_cfg);

        cmd_channel_->registerHandler("state_change",[this](const ReliableEnvelope& env, const msgpack::object& payload) {
            std::map<std::string, msgpack::object> fields;
            payload.convert(fields);
            auto it = fields.find("requested_state");
            if (it != fields.end()) {
                cmd_requested_.store(static_cast<SysState>(it->second.as<uint8_t>()));
            }
        });
    }

#ifndef WITH_FRANKA
    sim_ = std::make_shared<Simulation>(config);

    YAML::Node sim_config = YAML::LoadFile(config["sim_config"].as<std::string>());
    std::unordered_map<std::string, YAML::Node> sim_devs;
    for (const auto& sd : sim_config["devices"])
        sim_devs[sd["name"].as<std::string>()] = YAML::Node(sd);

    std::unordered_map<std::string, YAML::Node> robot_devs;
    for (const auto& rd : sys_config["devices"])
        robot_devs[rd["name"].as<std::string>()] = YAML::Node(rd);

    for (auto& arm : arm_instances) {
        arm->robot->set_simulation(*sim_, sim_devs[arm->getDeviceName()], robot_devs[arm->getDeviceName()]);
        std::string gripper_name = "hand_" + arm->getDeviceName().substr(arm->getDeviceName().find('_') + 1);
        if (sim_devs.count(gripper_name)){
            arm->gripper->set_simulation(*sim_, gripper_name);
        }
    }
    for (auto& head : head_instances)
        head->module->set_simulation(*sim_, sim_devs[head->getDeviceName()]);
#endif
}

Avatar::~Avatar(){

}

void Avatar::start(){
    if (cmd_channel_) cmd_channel_->start();
    for(const auto& head : head_instances){
        head->start();
    }
    for(const auto& arm : arm_instances){
        arm->start();
    }
    std::cout << "[AVATAR-INFO]: All devices started" << std::endl;

    constexpr std::chrono::microseconds control_period(static_cast<int>(1e6 / 100));
	auto next_control_time = std::chrono::high_resolution_clock::now();

    SysState cmd_state = SysState::IDLE;
    SysState prev_state = SysState::IDLE;
    state_ = SysState::IDLE;
    if (cmd_channel_) cmd_channel_->setState(SysState::IDLE);
    auto last_heartbeat = std::chrono::steady_clock::now();
    auto start_time = std::chrono::steady_clock::now();
    if (cmd_channel_) cmd_channel_->resetAliveTimer();
    bRunning = true;
   
    while(bRunning){
        SysState cmd_state = cmd_requested_.load();

        // connection watchdog — if operator goes silent, fall back to IDLE
        if (cmd_channel_ && !cmd_channel_->isAlive()) {
            if (state_ != SysState::IDLE && state_ != SysState::OFFLINE) {
                std::cout << "[AVATAR-WARN]: Operator connection lost, reverting to IDLE." << std::endl;
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
                cmd_requested_.store(SysState::IDLE);
                cmd_channel_->setState(SysState::IDLE);
            }
        } else {
            updateStateMachine(cmd_state);
        }

        if (state_ != prev_state && cmd_channel_) {
            cmd_channel_->setState(state_);
        }
        prev_state = state_;


        // heartbeat
        auto now = std::chrono::steady_clock::now();
        if (cmd_channel_ && std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat).count() >= 500) {
            msgpack::sbuffer buf;
            msgpack::pack(buf, std::map<std::string, int64_t>{{"uptime_ms", std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count()}});
            cmd_channel_->send("heartbeat", buf, false);
            last_heartbeat = now;
        }
        #ifndef WITH_FRANKA
            for (auto& arm : arm_instances) {
                Eigen::Isometry3d T = arm->getTargetPose();
                std::string dev = arm->getDeviceName();
                std::string side = dev.substr(dev.find('_') + 1);
                sim_->setFramePose("target_" + side + "_frame", T.translation(), Eigen::Quaterniond(T.rotation()));
            }
        #endif

        next_control_time += control_period;
        std::this_thread::sleep_until(next_control_time);
    }
}

void Avatar::stop(){
    for(const auto& head : head_instances){
        head->stop();
    }
    for(const auto& arm : arm_instances){
        arm->stop();
    }
    if (cmd_channel_) cmd_channel_->stop();
    bRunning = false;
}

void Avatar::updateStateMachine(SysState cmd_state){
    if(cmd_state == SysState::STOP){
        state_ = SysState::STOP;
    }
    switch (state_) {
        case SysState::IDLE:
            if(cmd_state == SysState::HOMING){
                requestAllDevices(SysState::HOMING);
                state_ = SysState::HOMING;  // transition avatar itself immediately
                std::cout << "[AVATAR-INFO]: Homing." << std::endl;
            }
            break;

        case SysState::HOMING:
            if(cmd_state == SysState::IDLE){
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
            }
            else if(allInState(SysState::AWAITING)){
                state_ = SysState::AWAITING;
                std::cout << "[AVATAR-INFO]: Awaiting engagement." << std::endl;
            }
            break;

        case SysState::AWAITING:
            if(cmd_state == SysState::IDLE){
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
            }
            else if(cmd_state == SysState::ENGAGED && allInState(SysState::AWAITING)){
                requestAllDevices(SysState::ENGAGED);
                state_ = SysState::ENGAGED;
                std::cout << "[AVATAR-INFO]: Engage system." << std::endl;
            }
            break;

        case SysState::ENGAGED:
            if(cmd_state == SysState::IDLE){
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
                std::cout << "[AVATAR-INFO]: Switch engage -> idle." << std::endl;
            }
            else if(cmd_state == SysState::PAUSED){
                requestAllDevices(SysState::PAUSED);
                state_ = SysState::PAUSED;
                std::cout << "[AVATAR-INFO]: Switch engage -> pause." << std::endl;
            }
            break;
            
        case SysState::PAUSED:
            if(cmd_state == SysState::IDLE){
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
                std::cout << "[AVATAR-INFO]: Switch pause -> idle." << std::endl;
            }
            else if(cmd_state == SysState::ENGAGED && allInState(SysState::PAUSED)){
                requestAllDevices(SysState::ENGAGED);
                state_ = SysState::ENGAGED;
                std::cout << "[AVATAR-INFO]: Switch pause -> engage." << std::endl;
            }
            break;

        default:
            break;
    }
}

bool Avatar::allInState(SysState state) {
    for (auto& arm  : arm_instances)  if (arm->getState()  != state) return false;
    for (auto& head : head_instances) if (head->getState() != state) return false;
    return true;
}

bool Avatar::anyoneInState(SysState state) {
    for (auto& arm  : arm_instances)  if (arm->getState()  == state) return true;
    for (auto& head : head_instances) if (head->getState() == state) return true;
    return false;
}

void Avatar::requestAllDevices(SysState state) {
    for (auto& arm  : arm_instances)  arm->requestState(state);
    for (auto& head : head_instances) head->requestState(state);
}