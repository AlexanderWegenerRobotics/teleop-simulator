#include <iostream>
#include <chrono>
#include <thread>

#include "sim_env/robot.hpp"
#include "sim_env/simulation.hpp"


using namespace franka;

Torques::Torques(const std::array<double, 7>& torques) noexcept : tau_J(torques) {}

Torques::Torques(std::initializer_list<double> torques) {
    std::copy(torques.begin(), torques.end(), tau_J.begin());
}

Robot::Robot() {
    r_      = Vector7::Zero();
    p_prev_ = Vector7::Zero();
}

Robot::~Robot() {}

void Robot::set_simulation(Simulation& _sim, const YAML::Node& sim_dev, const YAML::Node& robot_dev) {
    sim          = &_sim;
    name_        = sim_dev["name"].as<std::string>();
    ee_frame_name_ = sim_dev["urdf_ee_name"]
                     ? sim_dev["urdf_ee_name"].as<std::string>()
                     : "panda_link8";

    std::string urdf_path = sim_dev["urdf_path"].as<std::string>();

    auto ori = robot_dev["base_pose"]["orientation"].as<std::vector<double>>();
    std::array<double, 4> base_quat = {ori[0], ori[1], ori[2], ori[3]};

    model_ = std::make_unique<franka::Model>(urdf_path, base_quat, ee_frame_name_);
}

Model& Robot::loadModel() {
    return *model_;
}

RobotState Robot::readOnce() {
    return robot_state_;
}

void Robot::updateGMO(const std::array<double, 7>& q,
                       const std::array<double, 7>& dq,
                       const std::array<double, 7>& tau_cmd,
                       double dt) {
    Vector7 tau_eig = Eigen::Map<const Vector7>(tau_cmd.data());

    auto [p, tau_model] = model_->computeGMOInputs(q, dq);

    r_ += K_GMO * (p - p_prev_ - (tau_eig - tau_model + r_) * dt);
    p_prev_ = p;

    std::array<double, 7> tau_ext;
    Eigen::Map<Vector7>(tau_ext.data()) = r_;
    robot_state_.tau_ext_hat_filtered   = tau_ext;
    robot_state_.O_F_ext_hat_K          = model_->cartesianWrench(q, tau_ext);
}

void Robot::populateRobotState(const DeviceState& ds, double dt) {
    for (size_t i = 0; i < 7 && i < ds.q.size(); ++i) {
        robot_state_.q[i]     = ds.q[i];
        robot_state_.dq[i]    = ds.dq[i];
        robot_state_.tau_J[i] = ds.tau_J[i];
    }
    robot_state_.O_T_EE = model_->EEPose(robot_state_.q);
    updateGMO(robot_state_.q, robot_state_.dq, robot_state_.tau_J_d, dt);
}

void Robot::control(std::function<Torques(const RobotState&, Duration)> control_callback) {
    constexpr double dt = 1.0 / 1000.0;
    constexpr std::chrono::microseconds control_period(static_cast<int>(1e6 / 1000.0));
    auto next_control_time = std::chrono::high_resolution_clock::now();
    Duration dur;

    if (sim == nullptr) {
        std::cout << "You need to set the simulator first" << std::endl;
        return;
    }

    bRunning = true;
    while (bRunning) {
        if (!sim->isRunning()) {
            std::cout << "Simulation stopped" << std::endl;
            bRunning = false;
            break;
        }

        DeviceState device_state = sim->getDeviceState(name_);
        populateRobotState(device_state, dt);

        Torques tau_cmd = control_callback(robot_state_, dur);

        std::array<double, 7> gravity = model_->gravity(robot_state_.q);

        std::array<double, 7> tau_total;
        for (int i = 0; i < 7; ++i)
            tau_total[i] = tau_cmd.tau_J[i] + gravity[i];

        if (tau_cmd.motion_finished) {
            std::cout << "Stopped robot arm control loop" << std::endl;
            bRunning = false;
        }

        if (bRunning) {
            robot_state_.tau_J_d = tau_total;
            sim->setCtrl(name_, std::vector<double>(tau_total.begin(), tau_total.end()));
            next_control_time += control_period;
            std::this_thread::sleep_until(next_control_time);
        }
    }
}