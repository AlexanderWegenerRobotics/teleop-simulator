#include "head_control.hpp"
#include <iostream>

using namespace franka_joint_driver;

HeadControl::HeadControl(const YAML::Node& device_config)
    : bRunning(false)
    , module(std::make_unique<franka_joint_driver::Driver>())
    , state_(SysState::OFFLINE)
    , alpha_dq(0.5)
    , interpolator_(InterpolatorConfig{
        .control_freq   = 1000,
        .comm_freq      = 200,
        .n_dof          = 2,
        .max_linear_vel = 0.1,
        .max_angular_vel = 0.5
    })
{
    name_ = device_config["name"].as<std::string>();

    auto q0_vec = device_config["q0"].as<std::vector<double>>();
    q0_ = Eigen::Map<const Vector2>(q0_vec.data());

    if (device_config["transmission"]) {
        TransmissionConfig tx_config{
            .remote_ip    = device_config["transmission"]["remote_ip"].as<std::string>(),
            .send_port    = device_config["transmission"]["send_port"].as<int>(),
            .receive_port = device_config["transmission"]["receive_port"].as<int>(),
            .frequency    = device_config["transmission"]["frequency"].as<int>(),
            .role = TransmissionRole::HEAD
        };
        transmission_ = std::make_unique<Transmission>(tx_config);
    }
}

HeadControl::~HeadControl(){
    stop();
}

void HeadControl::start(){
    bRunning = true;
    state_ = SysState::IDLE;
    control_thread = std::thread(&HeadControl::runControlHandler, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    state_thread = std::thread(&HeadControl::runStateHandler, this);
    if (transmission_) transmission_->start();
}

void HeadControl::stop(){
    bRunning = false;
    if (control_thread.joinable()) control_thread.join();
    if (state_thread.joinable()) state_thread.join();
    if (transmission_) transmission_->stop();
}

void HeadControl::runStateHandler(){
    constexpr std::chrono::microseconds control_period(static_cast<int>(1e6 / 100));
    auto next_control_time = std::chrono::high_resolution_clock::now();

    SysState prev_state = state_;
    Vector2 q_current = Vector2::Zero();
    bool has_cmd = false;
    HeadCommandMsg cmd;

    while(bRunning){

        if (transmission_ && transmission_->hasNewCommand()) {
            cmd = transmission_->getHeadCommand();
            has_cmd = true;
        }

        updateStateMachine(cmd_state_);

        if (state_ == SysState::HOMING && prev_state != SysState::HOMING) {
            {
                std::lock_guard<std::mutex> lock(state_mtx);
                q_current = current_state.q;
            }
            interpolator_.planJoint(q_current, q0_);
        }
        else if (state_ == SysState::ENGAGED) {
            Vector2 q_target = q0_;
            if (has_cmd) {
                q_target(0) = static_cast<double>(cmd.pan);
                q_target(1) = static_cast<double>(cmd.tilt);
                has_cmd = false;
            }
            interpolator_.planJoint(interpolator_.getCurrentJoint(), q_target);
        }

        if (transmission_) {
            HeadStateMsg state_msg{};
            Vector2 q, dq;
            {
                std::lock_guard<std::mutex> lock(state_mtx);
                q  = current_state.q;
                dq = current_state.dq;
            }
            state_msg.state = state_;
            state_msg.pan   = static_cast<float>(q(0));
            state_msg.tilt  = static_cast<float>(q(1));
            transmission_->sendHeadState(state_msg);
        }

        prev_state = state_;
        next_control_time += control_period;
        std::this_thread::sleep_until(next_control_time);
    }
}

void HeadControl::updateStateMachine(SysState cmd_state){
    if(cmd_state == SysState::STOP){
        state_ = SysState::STOP;
    }
    switch (state_) {
        case SysState::IDLE:
            if(cmd_state == SysState::HOMING){
                state_ = SysState::HOMING;
                std::cout << "[INFO]: " << name_ << " is homing." << std::endl;
            }
            break;
        case SysState::HOMING:
            if(isHome()){
                state_ = SysState::AWAITING;
                std::cout << "[INFO]: " << name_ << " is awaiting." << std::endl;
            }
            break;
        case SysState::AWAITING:
            if(cmd_state == SysState::IDLE){
                state_ = SysState::IDLE;
            }
            else if(cmd_state == SysState::ENGAGED){
                state_ = SysState::ENGAGED;
                std::cout << "[INFO]: " << name_ << " engaged." << std::endl;
            }
            break;
        case SysState::ENGAGED:
            if(cmd_state == SysState::IDLE){
                state_ = SysState::IDLE;
            }
            else if(cmd_state == SysState::PAUSED){
                state_ = SysState::PAUSED;
            }
            break;
        case SysState::PAUSED:
            if(cmd_state == SysState::IDLE){
                state_ = SysState::IDLE;
            }
            else if(cmd_state == SysState::ENGAGED){
                state_ = SysState::ENGAGED;
            }
            break;

        default:
            break;
    }
}

void HeadControl::runControlHandler() {
    franka_joint_driver::Driver::CallbackFunctionTorque torque_control_callback =
        [&](const std::vector<Driver::State>& driver_state,
            std::vector<Driver::CommandTorque>& command) -> void {

            interpolator_.step();

            Vector2 q, tau_j;
            for (size_t i = 0; i < 2; ++i) {
                q(i)     = driver_state[i].theta;
                tau_j(i) = driver_state[i].tau_j;
            }

            auto now = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration<double>(now - current_state.last_dq_update).count();

            Vector2 dq = Vector2::Zero();
            if (dt > 1e-4) {
                Vector2 dq_raw = (q - current_state.q) / dt;    
                std::lock_guard<std::mutex> lock(state_mtx);
                dq = dq_raw * alpha_dq + (1.0 - alpha_dq) * current_state.dq;
                current_state.last_dq_update = now;
                current_state.q = q;
                current_state.dq = dq;
            }


            Vector2 q_cmd = interpolator_.getCurrentJoint();
            Vector2 e = q_cmd - q;
            Vector2 tau_cmd = kp_.cwiseProduct(e) - kd_.cwiseProduct(dq);

            for (size_t i = 0; i < 2; ++i) {
                command[i].tau_j_d  = tau_cmd[i];
                command[i].finished = false;
            }
        };

    if (module->control(torque_control_callback) == Driver::Error::kCommunicationError) {
        std::cout << "[WARNING]: " << name_ << " has a communication error." << std::endl;
        state_    = SysState::FAULT;
        bRunning  = false;
    }
}

bool HeadControl::isHome() {
    Vector2 q, dq;
    {
        std::lock_guard<std::mutex> lock(state_mtx);
        q  = current_state.q;
        dq = current_state.dq;
    }
    return (q0_ - q).cwiseAbs().maxCoeff() < 0.05 
        && dq.cwiseAbs().maxCoeff() < 0.01;
}