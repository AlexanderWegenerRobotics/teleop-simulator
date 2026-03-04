#include "arm_control.hpp"
#include "common.hpp"
#include "sim_env/model.hpp"
#include <chrono>
#include <iostream>

ArmControl::ArmControl(const YAML::Node& device_config) 
    : robot(std::make_unique<franka::Robot>())
    , gripper(std::make_unique<franka::Gripper>())
    , bRunning(false)
    , state_(SysState::OFFLINE)
    , interpolator_(InterpolatorConfig{
        .control_freq   = 1000,
        .comm_freq      = 200,
        .n_dof          = 7,
        .max_linear_vel = 0.1,
        .max_angular_vel = 0.5
    })
{
    name_ = device_config["name"].as<std::string>();
    
    auto pos  = device_config["base_pose"]["position"].as<std::vector<double>>();
    auto ori  = device_config["base_pose"]["orientation"].as<std::vector<double>>();

    base_position_ = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    base_orientation_ = Eigen::Quaterniond(ori[0], ori[1], ori[2], ori[3]);  // w x y z

    T_base_ = Eigen::Isometry3d::Identity();
    T_base_.translation() = base_position_;
    T_base_.linear()      = base_orientation_.toRotationMatrix();

    auto q0_vec = device_config["q0"].as<std::vector<double>>();
    q0_ = Eigen::Map<const Vector7>(q0_vec.data());

    if (device_config["transmission"]) {
        TransmissionConfig tx_config{
            .remote_ip    = device_config["transmission"]["remote_ip"].as<std::string>(),
            .send_port    = device_config["transmission"]["send_port"].as<int>(),
            .receive_port = device_config["transmission"]["receive_port"].as<int>(),
            .frequency    = device_config["transmission"]["frequency"].as<int>(),
            .role         = TransmissionRole::ARM
        };
        transmission_ = std::make_unique<Transmission>(tx_config);
    }
}

ArmControl::~ArmControl(){
    stop();
}

void ArmControl::start(){
    bRunning = true;
    state_ = SysState::IDLE;
    cmd_state_ = SysState::IDLE;
    current_state = robot->readOnce();
    model = std::make_unique<franka::Model>(robot->loadModel());
    control_thread = std::thread(&ArmControl::runControlHandler, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    state_thread = std::thread(&ArmControl::runStateHandler, this);
    if (transmission_) transmission_->start();
}

void ArmControl::stop(){
    bRunning = false;
    state_ = SysState::OFFLINE;
    if (control_thread.joinable()) control_thread.join();
    if (state_thread.joinable()) state_thread.join();
    if (transmission_) transmission_->stop();
}

void ArmControl::runStateHandler(){
    constexpr std::chrono::microseconds control_period(static_cast<int>(1e6 / 100));
    auto next_control_time = std::chrono::high_resolution_clock::now();
    SysState prev_state = state_;
    Eigen::VectorXd q_current = Eigen::VectorXd::Zero(7);
    bool has_cmd = false;
    ArmCommandMsg cmd;

    while(bRunning){

        if (transmission_ && transmission_->hasNewCommand()) {
            cmd = transmission_->getArmCommand();
            has_cmd = true;
        }

        updateStateMachine(cmd_state_);

        if (state_ == SysState::HOMING && prev_state != SysState::HOMING) {
            {
                std::lock_guard<std::mutex> lock(state_mtx);
                q_current = Eigen::Map<const Vector7>(current_state.q.data());
            }
            interpolator_.planJoint(q_current, q0_);
        }
        else if (state_ == SysState::ENGAGED) {
            Eigen::Isometry3d T_cmd = Eigen::Isometry3d::Identity();

            if (has_cmd) {
                Eigen::Vector3d pos(cmd.position[0], cmd.position[1], cmd.position[2]);
                Eigen::Quaterniond q(cmd.quaternion[0], cmd.quaternion[1],
                                     cmd.quaternion[2], cmd.quaternion[3]);
                T_cmd.translation() = pos;
                T_cmd.linear()      = q.toRotationMatrix();
                has_cmd = false;
            }

            Eigen::Isometry3d T_target = transformCommandToBase(T_cmd);
            interpolator_.planCartesian(interpolator_.getCurrentCartesian(), T_target);
        }

        // send state back
        if (transmission_) {
            ArmStateMsg state_msg{};
            franka::RobotState rs;
            {
                std::lock_guard<std::mutex> lock(state_mtx);
                rs = current_state;
            }
            Eigen::Isometry3d T_ee(Eigen::Map<const Eigen::Matrix4d>(rs.O_T_EE.data()));
            Eigen::Quaterniond q_ee(T_ee.rotation());

            state_msg.state       = state_;
            state_msg.position[0] = static_cast<float>(T_ee.translation().x());
            state_msg.position[1] = static_cast<float>(T_ee.translation().y());
            state_msg.position[2] = static_cast<float>(T_ee.translation().z());
            state_msg.quaternion[0] = static_cast<float>(q_ee.w());
            state_msg.quaternion[1] = static_cast<float>(q_ee.x());
            state_msg.quaternion[2] = static_cast<float>(q_ee.y());
            state_msg.quaternion[3] = static_cast<float>(q_ee.z());

            transmission_->sendArmState(state_msg);
        }

        prev_state = state_;
        next_control_time += control_period;
        std::this_thread::sleep_until(next_control_time);
    }
}

void ArmControl::updateStateMachine(SysState cmd_state){
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
                interpolator_.planCartesian(T_origin_, T_origin_);
                std::cout << "[INFO]: " << name_ << " is awaiting." << std::endl;
            }
            break;
        case SysState::AWAITING:
            if(cmd_state == SysState::ENGAGED){
                state_ = SysState::ENGAGED;
                std::cout << "[INFO]: " << name_ << " engaged." << std::endl;
            }
            break;
        case SysState::ENGAGED:
            if(cmd_state == SysState::PAUSED){
                state_ = SysState::PAUSED;
            }
            break;
        case SysState::PAUSED:
            if(cmd_state == SysState::ENGAGED){
                state_ = SysState::ENGAGED;
            }
            break;

        default:
            break;
    }
}

void ArmControl::runControlHandler(){
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        control_callback = [&](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {
            
            Vector7 ctrl_torque = Vector7::Zero();
            interpolator_.step();
            {
                std::lock_guard<std::mutex> lock(state_mtx);
                current_state = robot_state;
            }

            switch(state_){
                case SysState::HOMING:
                    ctrl_torque = jointImpedanceControl(robot_state);
                    break;

                case SysState::ENGAGED:
                    ctrl_torque = cartesianImpedanceControl(robot_state);
                    break;

                default:
                break;
            }
            
            std::array<double, 7> ctrl_array;
            Eigen::Map<Vector7>(ctrl_array.data()) = ctrl_torque;

            franka::Torques tau(ctrl_array);
            tau.motion_finished = !bRunning;
            return tau;
        };

    robot->control(control_callback);
}


Vector7 ArmControl::jointImpedanceControl(const franka::RobotState& rs) {
    Eigen::Map<const Vector7> q(rs.q.data());
    Eigen::Map<const Vector7> dq(rs.dq.data());

    Vector7 q_target = interpolator_.getCurrentJoint();

    Vector7 e = q_target - q;
    Vector7 de = -dq;

    auto mass_array = model->mass(rs.q);
    Matrix7 M = Eigen::Map<Matrix7>(mass_array.data());

    auto coriolis_array = model->coriolis(rs.q, rs.dq);
    Vector7 tau_coriolis = Eigen::Map<Vector7>(coriolis_array.data());

    Vector7 tau = M * (kp_joint_.cwiseProduct(e) + kd_joint_.cwiseProduct(de)) + tau_coriolis;

    return tau;
}

Vector7 ArmControl::cartesianImpedanceControl(const franka::RobotState& rs) {
    Eigen::Map<const Vector7> q(rs.q.data());
    Eigen::Map<const Vector7> dq(rs.dq.data());

    Eigen::Isometry3d T_ee(Eigen::Map<const Eigen::Matrix4d>(rs.O_T_EE.data()));

    // position and orientation error
    Eigen::Isometry3d T_ee_target = interpolator_.getCurrentCartesian();
    Eigen::Vector3d pos_error = T_ee_target.translation() - T_ee.translation();
    Eigen::Matrix3d R_error   = T_ee_target.rotation() * T_ee.rotation().transpose();
    Eigen::AngleAxisd aa(R_error);
    Eigen::Vector3d ori_error = aa.angle() * aa.axis();

    Eigen::Matrix<double, 6, 1> error;
    error << pos_error, ori_error;

    // jacobian
    auto J_array = model->zeroJacobian(rs.q);
    Matrix6x7 J  = Eigen::Map<Matrix6x7>(J_array.data());

    // ee velocity
    Eigen::Matrix<double, 6, 1> ee_vel = J * dq;

    // cartesian impedance
    Eigen::Matrix<double, 6, 1> F = kp_cart_.cwiseProduct(error) - kd_cart_.cwiseProduct(ee_vel);
    Vector7 tau_task = J.transpose() * F;

    // mass matrix and coriolis
    auto mass_array = model->mass(rs.q);
    Matrix7 M = Eigen::Map<Matrix7>(mass_array.data());

    auto coriolis_array = model->coriolis(rs.q, rs.dq);
    Vector7 tau_coriolis = Eigen::Map<Vector7>(coriolis_array.data());

    // null space control — keep joints close to q0_
    Eigen::Matrix<double, 7, 6> J_pinv = M.inverse() * J.transpose() *
        (J * M.inverse() * J.transpose()).inverse();

    Eigen::Matrix<double, 7, 7> N = Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;

    Vector7 tau_null = N * (kp_null_.cwiseProduct(q0_ - q) - kd_null_.cwiseProduct(dq));

    return tau_task + tau_null + tau_coriolis;
}

bool ArmControl::isHome() {
    Vector7 q, dq;
    Eigen::Isometry3d T_ee;
    {
        std::lock_guard<std::mutex> lock(state_mtx);
        q    = Eigen::Map<const Vector7>(current_state.q.data());
        dq   = Eigen::Map<const Vector7>(current_state.dq.data());
        T_ee = Eigen::Isometry3d(Eigen::Map<const Eigen::Matrix4d>(current_state.O_T_EE.data()));
    }

    bool position_reached = (q0_ - q).cwiseAbs().maxCoeff() < 0.05;
    bool velocity_settled = dq.cwiseAbs().maxCoeff() < 0.01;

    if (position_reached && velocity_settled) {
        T_origin_ = T_ee;
        return true;
    }
    return false;
}

Eigen::Isometry3d ArmControl::transformCommandToBase(const Eigen::Isometry3d& T_cmd_world) const {
    Eigen::Isometry3d T_target = T_origin_;

    T_target.translation() += T_base_.rotation().transpose() * T_cmd_world.translation();
    T_target.linear() = T_cmd_world.rotation() * T_origin_.rotation();

    return T_target;
}