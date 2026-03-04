#include "sim_env/simulation.hpp"

#include <stdexcept>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <unordered_map>

#include <GLFW/glfw3.h>
#include <yaml-cpp/yaml.h>


// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

Simulation::Simulation(const YAML::Node& config) {
    BuiltScene scene = SceneBuilder::build(config);

    shm_enabled_ = config["rendering"] && config["rendering"]["streaming"].as<bool>(false);
    stream_camera_ = config["rendering"]["stream_camera"].as<std::string>("");

    devices_ = std::move(scene.devices);
    objects_ = std::move(scene.objects);
    cameras_ = std::move(scene.cameras);

    char err[2000] = {};
    model = mj_loadXML(scene.xml_path.c_str(), nullptr, err, sizeof(err));
    if (!model)
        throw std::runtime_error(std::string("mj_loadXML failed: ") + err);

    if (config["simulation"] && config["simulation"]["timestep"])
        model->opt.timestep = config["simulation"]["timestep"].as<double>();

    data = mj_makeData(model);
    if (!data)
        throw std::runtime_error("mj_makeData failed");

    ctrl_buffer_.assign(model->nu, 0.0);

    buildActuatorIndex();
    applyInitialPositions();

    render_enabled_ = config["rendering"] && config["rendering"]["enabled"].as<bool>();
    if (config["rendering"] && config["rendering"]["fps"])
        render_fps_ = config["rendering"]["fps"].as<int>();

    snap_[0] = mj_copyData(nullptr, model, data);
    snap_[1] = mj_copyData(nullptr, model, data);
    buildCameraList();
}

Simulation::~Simulation() {
    if (snap_[0]) mj_deleteData(snap_[0]);
    if (snap_[1]) mj_deleteData(snap_[1]);
    if (data)     mj_deleteData(data);
    if (model)    mj_deleteModel(model);
}


// ---------------------------------------------------------------------------
// Threading
// ---------------------------------------------------------------------------

void Simulation::start() {
    rendering_thread = std::thread(&Simulation::run_rendering, this);
    while (!bRenderingIsRunning)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    model_thread = std::thread(&Simulation::run_model, this);
}

void Simulation::stop() {
    bModelIsRunning     = false;
    bRenderingIsRunning = false;
    if (model_thread.joinable())     model_thread.join();
    if (rendering_thread.joinable()) rendering_thread.join();
}

bool Simulation::isRunning() const {
    return bModelIsRunning || bRenderingIsRunning;
}

void Simulation::run_model() {
    bModelIsRunning = true;
    while (bModelIsRunning) {
        {
            std::lock_guard<std::mutex> lock(data_mtx);
            {
                std::lock_guard<std::mutex> ctrl_lock(ctrl_mtx_);
                for (int i = 0; i < model->nu; ++i){
                    data->ctrl[i] = ctrl_buffer_[i];
                }
            }
            mj_step(model, data);
            swapSnapshots();
        }
        std::this_thread::sleep_for(
            std::chrono::microseconds(
                static_cast<int>(model->opt.timestep * 1e6)));
    }
}


// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

void Simulation::buildCameraList() {
    render_cams_.clear();
    for (int i = 0; i < model->ncam; ++i) {
        const char* name = mj_id2name(model, mjOBJ_CAMERA, i);
        render_cams_.push_back({name ? name : ("cam" + std::to_string(i)), i});
    }
}

static void gridDims(int n, int& cols, int& rows) {
    cols = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(n))));
    rows = (n + cols - 1) / cols;
}

void Simulation::initRendering() {
    if (!window_)
        throw std::runtime_error("[Rendering] window_ is null — must be created on main thread");

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);

    mjv_defaultOption(&vopt_);
    mjv_defaultScene(&scn_);
    mjv_makeScene(model, &scn_, 2000);
    mjr_defaultContext(&con_);
    mjr_makeContext(model, &con_, mjFONTSCALE_100);

    int win_w, win_h;
    glfwGetFramebufferSize(window_, &win_w, &win_h);

    if (shm_enabled_)
        shm_writer_ = std::make_unique<SharedMemoryWriter>("/avatar_cam", win_w, win_h);
}

void Simulation::run_rendering() {
    if (!render_enabled_) {
        bRenderingIsRunning = true;
        return;
    }

    try {
        initRendering();
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        bRenderingIsRunning = true;
        return;
    }

    bRenderingIsRunning = true;

    auto frame_duration = std::chrono::microseconds(1000000 / render_fps_);
    auto next_frame     = std::chrono::steady_clock::now();

    while (bRenderingIsRunning && !glfwWindowShouldClose(window_)) {
        renderFrame();
        next_frame += frame_duration;
        std::this_thread::sleep_until(next_frame);
    }

    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);

    bRenderingIsRunning = false;
}

void Simulation::renderFrame() {
    int ncam = static_cast<int>(render_cams_.size());
    if (ncam == 0) return;

    int win_w, win_h;
    glfwGetFramebufferSize(window_, &win_w, &win_h);

    int cols, rows;
    gridDims(ncam, cols, rows);
    int cell_w = win_w / cols;
    int cell_h = win_h / rows;

    int r        = snap_read_.load(std::memory_order_acquire);
    mjData* snap = snap_[r];

    for (int i = 0; i < ncam; ++i) {
        int col = i % cols;
        int row = i / cols;

        int x = col * cell_w;
        int y = (rows - 1 - row) * cell_h;
        mjrRect viewport = {x, y, cell_w, cell_h};

        mjvCamera mjcam;
        mjv_defaultCamera(&mjcam);
        mjcam.type       = mjCAMERA_FIXED;
        mjcam.fixedcamid = render_cams_[i].id;

        mjv_updateScene(model, snap, &vopt_, nullptr, &mjcam, mjCAT_ALL, &scn_);
        mjr_render(viewport, &scn_, &con_);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,render_cams_[i].name.c_str(), nullptr, &con_);
        
        if (shm_writer_ && render_cams_[i].name == stream_camera_) {
            std::vector<uint8_t> pixels(cell_w * cell_h * 3);
            mjrRect vp = {x, y, cell_w, cell_h};
            mjr_readPixels(pixels.data(), nullptr, vp, &con_);

            for (int row = 0; row < cell_h / 2; ++row) {
                uint8_t* top = pixels.data() + row * cell_w * 3;
                uint8_t* bot = pixels.data() + (cell_h - 1 - row) * cell_w * 3;
                std::swap_ranges(top, top + cell_w * 3, bot);
            }

            shm_writer_->write(pixels.data(), pixels.size());
        }
    }

    glfwSwapBuffers(window_);
}

void Simulation::swapSnapshots() {
    int w = snap_write_.load(std::memory_order_relaxed);
    mj_copyData(snap_[w], model, data);
    int next = 1 - w;
    snap_write_.store(next, std::memory_order_release);
    snap_read_.store(w,    std::memory_order_release);
}


// ---------------------------------------------------------------------------
// Control API
// ---------------------------------------------------------------------------

void Simulation::setCtrl(const std::string& deviceName,
                          const std::vector<double>& values) {
    auto it = actuator_ids_.find(deviceName);
    if (it == actuator_ids_.end()) {
        std::cerr << "[Simulation] setCtrl: unknown device '" << deviceName << "'\n";
        return;
    }
    const auto& ids = it->second;

    std::lock_guard<std::mutex> lock(ctrl_mtx_);
    for (size_t i = 0; i < values.size() && i < ids.size(); ++i)
        ctrl_buffer_[ids[i]] = values[i];
}

void Simulation::setGripper(const std::string& deviceName, double value) {
    auto it = gripper_ids_.find(deviceName);
    if (it == gripper_ids_.end() || it->second < 0) {
        std::cerr << "[Simulation] setGripper: device '" << deviceName
                  << "' has no gripper actuator defined\n";
        return;
    }

    std::lock_guard<std::mutex> lock(ctrl_mtx_);
    ctrl_buffer_[it->second] = value;
}


// ---------------------------------------------------------------------------
// Actuator index tables
// ---------------------------------------------------------------------------

void Simulation::buildActuatorIndex() {
    for (const auto& dev : devices_) {
        std::string gripperFullName;
        if (!dev.gripper_actuator.empty())
            gripperFullName = dev.name + "_" + dev.gripper_actuator;

        std::vector<int> jointActuators;
        std::vector<int> joints;
        int gripperId = -1;

        for (int i = 0; i < model->nu; ++i) {
            const char* aname = mj_id2name(model, mjOBJ_ACTUATOR, i);
            if (!aname) continue;
            std::string name(aname);
            if (name.rfind(dev.name + "_", 0) != 0) continue;
            if (!gripperFullName.empty() && name == gripperFullName)
                gripperId = i;
            else
                jointActuators.push_back(i);
        }

        for (int j = 0; j < model->njnt; ++j) {
            const char* jname = mj_id2name(model, mjOBJ_JOINT, j);
            if (jname && std::string(jname).rfind(dev.name + "_", 0) == 0)
                joints.push_back(j);
        }

        actuator_ids_[dev.name] = std::move(jointActuators);
        gripper_ids_[dev.name]  = gripperId;
        joint_ids_[dev.name]    = std::move(joints);
    }
}


// ---------------------------------------------------------------------------
// Initial joint positions
// ---------------------------------------------------------------------------

void Simulation::applyInitialPositions() {
    for (const auto& dev : devices_) {
        std::vector<int> joints;
        for (int j = 0; j < model->njnt; ++j) {
            const char* jname = mj_id2name(model, mjOBJ_JOINT, j);
            if (jname && std::string(jname).rfind(dev.name + "_", 0) == 0)
                joints.push_back(j);
        }
        for (size_t i = 0; i < dev.q0.size() && i < joints.size(); ++i)
            data->qpos[model->jnt_qposadr[joints[i]]] = dev.q0[i];
    }
    mj_forward(model, data);
}

DeviceState Simulation::getDeviceState(const std::string& deviceName) {
    auto it = joint_ids_.find(deviceName);
    if (it == joint_ids_.end()) {
        std::cerr << "[Simulation] getDeviceState: unknown device '" << deviceName << "'\n";
        return {};
    }

    int r = snap_read_.load(std::memory_order_acquire);
    mjData* snap = snap_[r];

    DeviceState state;
    for (int j : it->second) {
        int qadr = model->jnt_qposadr[j];
        int vadr = model->jnt_dofadr[j];
        state.q.push_back(snap->qpos[qadr]);
        state.dq.push_back(snap->qvel[vadr]);
        state.tau_J.push_back(snap->qfrc_actuator[vadr]);
        state.tau_ext.push_back(snap->qfrc_constraint[vadr]);
    }

    return state;
}