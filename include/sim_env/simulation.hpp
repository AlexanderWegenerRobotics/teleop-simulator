#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjrender.h>

#include "streamer/shared_memory.hpp"
#include "sim_env/scene_builder.hpp"

struct DeviceState {
    std::vector<double> q;      // joint positions
    std::vector<double> dq;     // joint velocities
    std::vector<double> tau_J;  // measured torques (from qfrc_actuator)
    std::vector<double> tau_ext; // external torques (from qfrc_constraint or GMO)
};


class Simulation {
public:
    explicit Simulation(const YAML::Node& config);
    ~Simulation();

    void start();
    void stop();
    bool isRunning() const;
    void setCtrl(const std::string& deviceName, const std::vector<double>& values);
    void setGripper(const std::string& deviceName, double value);
    DeviceState getDeviceState(const std::string& deviceName);

private:
    mjModel* model = nullptr;
    mjData*  data  = nullptr;

    std::vector<DeviceConfig> devices_;
    std::vector<ObjectConfig> objects_;
    std::vector<CameraConfig> cameras_;

private:
    void run_model();
    void run_rendering();
    void applyInitialPositions();
    void buildActuatorIndex();
    const std::vector<int>& getJointActuatorIds(const std::string& deviceName) const;
    std::mutex data_mtx;
    std::vector<double> ctrl_buffer_;
    std::mutex          ctrl_mtx_;
    std::unordered_map<std::string, std::vector<int>> actuator_ids_;
    std::unordered_map<std::string, int> gripper_ids_;
    std::unordered_map<std::string, std::vector<int>> joint_ids_;
    std::atomic<bool> bModelIsRunning{false};
    std::atomic<bool> bRenderingIsRunning{false};
    std::thread       model_thread;
    std::thread       rendering_thread;
    std::unique_ptr<SharedMemoryWriter> shm_writer_;

private:
    // Rendering
    struct CamEntry { std::string name; int id; };

    mjvScene    scn_;
    mjvOption   vopt_;
    mjrContext  con_;
    mjData*              snap_[2]     = {nullptr, nullptr};
    std::atomic<int>     snap_write_  {0};
    std::atomic<int>     snap_read_   {1};
    int  render_fps_     = 20;
    void buildCameraList();
    void initRendering();
    void renderFrame();
    void swapSnapshots();

public:
    GLFWwindow* window_ = nullptr;
    bool render_enabled_ = false;
    std::vector<CamEntry> render_cams_;
    bool shm_enabled_;
    std::string stream_camera_;
    GLFWwindow* offscreen_window_ = nullptr;

private:
    mjvScene    stream_scn_;
    mjvOption   stream_vopt_;
    mjrContext  stream_con_;
    int         stream_width_  = 1280;
    int         stream_height_ = 720;
    int         stream_fps_    = 30;
    std::thread stream_thread_;
    std::atomic<bool> bStreamingIsRunning{false};

    void run_streaming();
    void initOffscreenStreaming();
    void renderStreamFrame();
};