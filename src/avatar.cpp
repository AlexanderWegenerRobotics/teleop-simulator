#include "avatar.hpp"
#include "common.hpp"

Avatar::Avatar(const YAML::Node& config){


#ifndef WITH_FRANKA

    YAML::Node sim_config = YAML::LoadFile(config["sim_config"].as<std::string>());
    sim_ = std::make_shared<Simulation>(sim_config);

    // Iterate devices defined in sim_config
    for (const auto& dev : sim_config["devices"]) {
        if (!dev["enabled"].as<bool>(true)) continue;

        std::string name = dev["name"].as<std::string>();
        std::string type = dev["type"].as<std::string>();

        if (type == "arm") {
            ArmControl* arm = new ArmControl(dev);
            arm->robot->set_simulation(*sim_, dev);
            arm_instances.push_back(arm);
        }
        else if (type == "head") {
            HeadControl* head = new HeadControl(dev);
            head->module->set_simulation(*sim_, dev);
            head_instances.push_back(head);
        }
    }
#endif

    if (config["transmission"]) {
        TransmissionConfig tx_config{
            .remote_ip    = config["transmission"]["remote_ip"].as<std::string>(),
            .send_port    = config["transmission"]["send_port"].as<int>(),
            .receive_port = config["transmission"]["receive_port"].as<int>(),
            .frequency    = config["transmission"]["frequency"].as<int>(),
            .role         = TransmissionRole::AVATAR
        };
        transmission_ = std::make_unique<Transmission>(tx_config);
    }
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

    constexpr std::chrono::microseconds control_period(static_cast<int>(1e6 / 100));
	auto next_control_time = std::chrono::high_resolution_clock::now();

    SysState cmd_state = SysState::IDLE;

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
            }
            break;
        case SysState::ENGAGED:
            if(cmd_state == SysState::IDLE){
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
            }
            else if(cmd_state == SysState::PAUSED){
                requestAllDevices(SysState::PAUSED);
                state_ = SysState::PAUSED;
            }
            break;
        case SysState::PAUSED:
            if(cmd_state == SysState::IDLE){
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
            }
            else if(cmd_state == SysState::ENGAGED && allInState(SysState::PAUSED)){
                requestAllDevices(SysState::ENGAGED);
                state_ = SysState::ENGAGED;
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