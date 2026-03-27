#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

#include <yaml-cpp/yaml.h>

#include "sim_env/robot.hpp"
#include "sim_env/gripper.hpp"
#include "interpolator.hpp"
#include "network/udp_stream.hpp"
#include "data_logger.hpp"
#include "common.hpp"

class Simulation;

class ArmControl{
public:
    ArmControl(const YAML::Node& device_config);
    ~ArmControl();

    void start();
    void stop();
    bool isRunning() const {return bRunning;}
    std::string getDeviceName() const {return name_;}
    void requestState(SysState state){cmd_state_ = state;}
    SysState getState() const {return state_;}

public:
    std::unique_ptr<franka::Robot> robot;
	std::unique_ptr<franka::Gripper> gripper;
	std::unique_ptr<franka::Model> model;

private:
    std::thread control_thread;
    std::thread state_thread;
    std::mutex data_mtx;
    std::atomic<bool> bRunning;
    std::atomic<SysState> state_{SysState::OFFLINE};
    std::atomic<SysState> cmd_state_{SysState::OFFLINE};
    Interpolator interpolator_;
    using ArmStream = UdpStream<ArmCommandMsg, ArmStateMsg>;
    std::unique_ptr<ArmStream> transmission_;
    std::unique_ptr<DataLogger<ArmLogEntry>> logger_;
    std::chrono::high_resolution_clock::time_point startTime_;
    
private:
    void runControlHandler();
    void runStateHandler();
    Vector7 jointImpedanceControl(const franka::RobotState& rs);
    Vector7 cartesianImpedanceControl(const franka::RobotState& rs);
    void updateStateMachine(SysState cmd_state);
    bool isHome();
    Eigen::Isometry3d transformCommandToBase(const Eigen::Isometry3d& T_cmd_world) const;

private:
    std::string name_;
    Eigen::Vector3d base_position_;
    Eigen::Quaterniond base_orientation_;
    Eigen::Isometry3d T_base_;
    Vector7 q0_;
    Vector7 tau_max_;
    Vector7 tau_rate_max_;
    std::mutex state_mtx;
    franka::RobotState current_state;
    Eigen::Isometry3d T_origin_;

private:
    Vector7 kp_joint_, kd_joint_;
    Eigen::Matrix<double, 6, 1> kp_cart_, kd_cart_;
    Vector7 kp_null_, kd_null_;
    double motion_scale_ = 3.0;
    double rotation_scale_ = 1.0;
};