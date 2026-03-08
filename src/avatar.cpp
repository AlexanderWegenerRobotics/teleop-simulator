#include "avatar.hpp"
#include "common.hpp"

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
        TransmissionConfig tx_config{
            .remote_ip    = sys_config["avatar"]["transmission"]["remote_ip"].as<std::string>(),
            .send_port    = sys_config["avatar"]["transmission"]["send_port"].as<int>(),
            .receive_port = sys_config["avatar"]["transmission"]["receive_port"].as<int>(),
            .frequency    = sys_config["avatar"]["transmission"]["frequency"].as<int>(),
            .role         = TransmissionRole::AVATAR
        };
        transmission_ = std::make_unique<Transmission>(tx_config);
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

    for (auto& arm : arm_instances)
        arm->robot->set_simulation(*sim_, sim_devs[arm->getDeviceName()], robot_devs[arm->getDeviceName()]);
    for (auto& head : head_instances)
        head->module->set_simulation(*sim_, sim_devs[head->getDeviceName()]);
#endif
}

Avatar::~Avatar(){

}

void Avatar::start(){
    if (transmission_) transmission_->start();
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
    state_ = SysState::IDLE;

    bRunning = true;
    while(bRunning){

        if (transmission_ && transmission_->hasNewCommand()) {
            AvatarCommandMsg cmd = transmission_->getAvatarCommand();
            cmd_state = cmd.requested_state;
        }

        updateStateMachine(cmd_state);

        AvatarStateMsg state_msg{};
        state_msg.system_state = getState();
        if (transmission_) transmission_->sendAvatarState(state_msg);

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
    if (transmission_) transmission_->stop();

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
            }
            if(allInState(SysState::AWAITING)){
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