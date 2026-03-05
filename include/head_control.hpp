#pragma once
#include <atomic>
#include <string>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <thread>

#include "sim_env/tum_head_driver.hpp"
#include "interpolator.hpp"
#include "transmission.hpp"
#include "data_logger.hpp"
#include "common.hpp"

struct HeadState {
    Vector2 q, dq;
    std::chrono::high_resolution_clock::time_point last_dq_update;

    HeadState() {
        q.setZero();
        dq.setZero();
        last_dq_update = std::chrono::high_resolution_clock::now();
    }
};

class HeadControl{
public:
    HeadControl(const YAML::Node& device_config);
    ~HeadControl();

    void start();
    void stop();
    bool isRunning() const {return bRunning;}
    std::string getDeviceName() const {return name_;}
    void requestState(SysState state){cmd_state_ = state;}
    SysState getState() const {return state_;}

public:
    std::unique_ptr<franka_joint_driver::Driver> module;
    Interpolator interpolator_;
    std::unique_ptr<Transmission> transmission_;
    std::unique_ptr<DataLogger<HeadLogEntry>> logger_;
    std::chrono::high_resolution_clock::time_point startTime_;

private:
    std::atomic<bool> bRunning;
    std::thread control_thread;
    std::thread state_thread;
    std::mutex state_mtx;
    std::string name_;
    std::atomic<SysState> state_{SysState::OFFLINE};
    std::atomic<SysState> cmd_state_{SysState::OFFLINE};
    Vector2 q0_;
    HeadState current_state;
    double alpha_dq;
    const Vector2 kp_ = (Vector2() << 50.0, 50.0).finished();
    const Vector2 kd_ = (Vector2() << 5.0, 5.0).finished();

private:
    void runControlHandler();
    void runStateHandler();
    void updateStateMachine(SysState cmd_state);
    bool isHome();
};