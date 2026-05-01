#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "common.hpp"

struct ArmLogEntry {
    double                 time;
    std::array<double, 7>  q;
    std::array<double, 7>  q_cmd;
    std::array<double, 7>  dq;
    std::array<double, 7>  tau_J;
    std::array<double, 7>  tau_ext;
    std::array<double, 16> O_T_EE;
    std::array<double, 16> O_T_EE_cmd;
    std::array<double, 6>  F_ext;
    double                 gripper_width;
    SysState               state;
};

struct HeadLogEntry {
    double                time;
    std::array<double, 2> q;
    std::array<double, 2> q_cmd;
    std::array<double, 2> dq;
    std::array<double, 2> tau_J;
    SysState              state;
};

template<typename T>
class DataLogger {
public:
    DataLogger(const std::string& path,
               std::function<std::string()>         headerFn,
               std::function<std::string(const T&)> rowFn,
               const std::string& session_id)
        : headerFn_(headerFn)
        , rowFn_(rowFn)
        , session_id_(session_id)
        , bRunning_(false)
        , bEnabled_(false)
        , bHasNewData_(false)
    {
        openFiles(path);
    }

    ~DataLogger() { stop(); }

    void start() {
        file_ << headerFn_();
        bRunning_ = true;
        startTime_ = std::chrono::high_resolution_clock::now();
        thread_ = std::thread(&DataLogger::run, this);
    }

    void stop() {
        bRunning_ = false;
        if (thread_.joinable()) thread_.join();
        file_.close();
        meta_file_.close();
    }

    // Close current files, open new ones at new_path, restart logging thread.
    // Call this between episodes when you want a fresh file in a new folder.
    void restart(const std::string& new_path) {
        // Stop logging thread and flush
        bEnabled_ = false;
        bRunning_ = false;
        if (thread_.joinable()) thread_.join();
        file_.close();
        meta_file_.close();

        episode_id_ = 0;
        openFiles(new_path);

        file_ << headerFn_();
        bRunning_ = true;
        startTime_ = std::chrono::high_resolution_clock::now();
        thread_ = std::thread(&DataLogger::run, this);
        bEnabled_ = true; 
    }

    void enable(bool e) { bEnabled_ = e; }

    void write(const T& data) {
        if (!bEnabled_) return;
        std::lock_guard<std::mutex> lock(mtx_);
        data_        = data;
        bHasNewData_ = true;
    }

    void markEpisodeStart() {
        writeMarker("episode_start", "");
        episode_id_++;
    }

    void markEpisodeEnd(const std::string& reason) {
        writeMarker("episode_end", reason);
    }

    // Write episode config (pick/place pose, mode) to meta file once per episode
    void writeEpisodeConfig(double pick_x, double pick_y, double pick_z,
                            double place_x, double place_y, double place_z,
                            int mode)
    {
        double t = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - startTime_).count();
        std::lock_guard<std::mutex> lock(meta_mtx_);
        meta_file_ << session_id_ << ";"
                   << episode_id_ << ";"
                   << "episode_config" << ";"
                   << t << ";"
                   << "" << ";"   // reason column (empty for config)
                   << pick_x  << ";" << pick_y  << ";" << pick_z  << ";"
                   << place_x << ";" << place_y << ";" << place_z << ";"
                   << mode << "\n";
        meta_file_.flush();
    }

private:
    void openFiles(const std::string& path) {
        file_.open(path);
        if (!file_)
            throw std::runtime_error("DataLogger: failed to open file: " + path);

        std::string meta_path = path.substr(0, path.rfind('.')) + "_meta.csv";
        meta_file_.open(meta_path);
        if (!meta_file_)
            throw std::runtime_error("DataLogger: failed to open meta file: " + meta_path);

        // Extended header: includes episode config columns
        meta_file_ << "session_id;episode_id;event;time_s;reason;"
                      "pick_x;pick_y;pick_z;place_x;place_y;place_z;mode\n";
        meta_file_.flush();
    }

    void writeMarker(const std::string& event, const std::string& reason) {
        double t = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - startTime_).count();
        std::lock_guard<std::mutex> lock(meta_mtx_);
        // Pad the config columns with empty fields for non-config events
        meta_file_ << session_id_ << ";"
                   << episode_id_ << ";"
                   << event       << ";"
                   << t           << ";"
                   << reason      << ";;;;;;;\n";
        meta_file_.flush();
    }

    void run() {
        std::string buffer;
        buffer.reserve(1024 * 1024);
        int rowCount = 0;
        while (bRunning_) {
            if (bHasNewData_ && bEnabled_) {
                T snapshot;
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    snapshot     = data_;
                    bHasNewData_ = false;
                }
                buffer += rowFn_(snapshot);
                if (++rowCount >= 500) {
                    file_.write(buffer.data(), buffer.size());
                    buffer.clear();
                    rowCount = 0;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
        if (!buffer.empty())
            file_.write(buffer.data(), buffer.size());
    }

    std::function<std::string()>         headerFn_;
    std::function<std::string(const T&)> rowFn_;
    std::string                          session_id_;

    std::ofstream file_;
    std::ofstream meta_file_;
    std::thread   thread_;

    std::atomic<bool> bRunning_;
    std::atomic<bool> bEnabled_;
    std::atomic<bool> bHasNewData_;

    std::mutex mtx_;
    std::mutex meta_mtx_;
    T          data_{};
    int        episode_id_ = 0;

    std::chrono::high_resolution_clock::time_point startTime_;
};


inline std::string armLogHeader() {
    std::string h = "time;";
    for (int i = 0; i < 7;  ++i) h += "q_"          + std::to_string(i) + ";";
    for (int i = 0; i < 7;  ++i) h += "q_cmd_"      + std::to_string(i) + ";";
    for (int i = 0; i < 7;  ++i) h += "dq_"         + std::to_string(i) + ";";
    for (int i = 0; i < 7;  ++i) h += "tau_J_"      + std::to_string(i) + ";";
    for (int i = 0; i < 7;  ++i) h += "tau_ext_"    + std::to_string(i) + ";";
    for (int i = 0; i < 16; ++i) h += "O_T_EE_"     + std::to_string(i) + ";";
    for (int i = 0; i < 16; ++i) h += "O_T_EE_cmd_" + std::to_string(i) + ";";
    for (int i = 0; i < 6;  ++i) h += "F_ext_"      + std::to_string(i) + ";";
    h += "gripper_width;";
    h += "state\n";
    return h;
}

inline std::string armLogRow(const ArmLogEntry& e) {
    std::string r = std::to_string(e.time) + ";";
    for (auto v : e.q)          r += std::to_string(v) + ";";
    for (auto v : e.q_cmd)      r += std::to_string(v) + ";";
    for (auto v : e.dq)         r += std::to_string(v) + ";";
    for (auto v : e.tau_J)      r += std::to_string(v) + ";";
    for (auto v : e.tau_ext)    r += std::to_string(v) + ";";
    for (auto v : e.O_T_EE)     r += std::to_string(v) + ";";
    for (auto v : e.O_T_EE_cmd) r += std::to_string(v) + ";";
    for (auto v : e.F_ext)      r += std::to_string(v) + ";";
    r += std::to_string(e.gripper_width) + ";";
    r += std::to_string(static_cast<uint8_t>(e.state)) + "\n";
    return r;
}

inline std::string headLogHeader() {
    std::string h = "time;";
    for (int i = 0; i < 2; ++i) h += "q_"     + std::to_string(i) + ";";
    for (int i = 0; i < 2; ++i) h += "q_cmd_" + std::to_string(i) + ";";
    for (int i = 0; i < 2; ++i) h += "dq_"    + std::to_string(i) + ";";
    for (int i = 0; i < 2; ++i) h += "tau_J_" + std::to_string(i) + ";";
    h += "state\n";
    return h;
}

inline std::string headLogRow(const HeadLogEntry& e) {
    std::string r = std::to_string(e.time) + ";";
    for (auto v : e.q)     r += std::to_string(v) + ";";
    for (auto v : e.q_cmd) r += std::to_string(v) + ";";
    for (auto v : e.dq)    r += std::to_string(v) + ";";
    for (auto v : e.tau_J) r += std::to_string(v) + ";";
    r += std::to_string(static_cast<uint8_t>(e.state)) + "\n";
    return r;
}

struct SceneLogEntry {
    double time;
    double object_x, object_y, object_z;
    double object_qw, object_qx, object_qy, object_qz;
    double pick_x, pick_y, pick_z;
    double place_x, place_y, place_z;
    int    mode;
};

inline std::string sceneLogHeader() {
    return "time;"
           "object_x;object_y;object_z;"
           "object_qw;object_qx;object_qy;object_qz;"
           "pick_x;pick_y;pick_z;"
           "place_x;place_y;place_z;"
           "mode\n";
}

inline std::string sceneLogRow(const SceneLogEntry& e) {
    std::string r = std::to_string(e.time) + ";";
    r += std::to_string(e.object_x)  + ";" + std::to_string(e.object_y)  + ";" + std::to_string(e.object_z)  + ";";
    r += std::to_string(e.object_qw) + ";" + std::to_string(e.object_qx) + ";" + std::to_string(e.object_qy) + ";" + std::to_string(e.object_qz) + ";";
    r += std::to_string(e.pick_x)    + ";" + std::to_string(e.pick_y)    + ";" + std::to_string(e.pick_z)    + ";";
    r += std::to_string(e.place_x)   + ";" + std::to_string(e.place_y)   + ";" + std::to_string(e.place_z)   + ";";
    r += std::to_string(e.mode) + "\n";
    return r;
}