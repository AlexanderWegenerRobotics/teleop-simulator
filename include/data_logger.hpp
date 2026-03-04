#pragma once

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
    std::array<double, 7>  dq;
    std::array<double, 7>  tau_J;
    std::array<double, 7>  tau_ext;
    std::array<double, 16> O_T_EE;
    std::array<double, 6>  F_ext;
    SysState               state;
};

struct HeadLogEntry {
    double                time;
    std::array<double, 2> q;
    std::array<double, 2> dq;
    std::array<double, 2> tau_J;
    SysState              state;
};

template<typename T>
class DataLogger {
public:
    DataLogger(const std::string& path,
               std::function<std::string()>        headerFn,
               std::function<std::string(const T&)> rowFn)
        : headerFn_(headerFn)
        , rowFn_(rowFn)
        , bRunning_(false)
        , bEnabled_(false)
        , bHasNewData_(false)
    {
        file_.open(path);
        if (!file_) {
            throw std::runtime_error("DataLogger: failed to open file: " + path);
        }
    }

    ~DataLogger() {
        stop();
    }

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
    }

    void enable(bool e) { bEnabled_ = e; }

    void write(const T& data) {
        if (!bEnabled_) return;
        std::lock_guard<std::mutex> lock(mtx_);
        data_        = data;
        bHasNewData_ = true;
    }

private:
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

private:
    std::function<std::string()>         headerFn_;
    std::function<std::string(const T&)> rowFn_;

    std::ofstream file_;
    std::thread   thread_;

    std::atomic<bool> bRunning_;
    std::atomic<bool> bEnabled_;
    std::atomic<bool> bHasNewData_;

    std::mutex mtx_;
    T          data_{};

    std::chrono::high_resolution_clock::time_point startTime_;
};


inline std::string armLogHeader() {
    std::string h = "time;";
    for (int i = 0; i < 7;  ++i) h += "q_"       + std::to_string(i) + ";";
    for (int i = 0; i < 7;  ++i) h += "dq_"      + std::to_string(i) + ";";
    for (int i = 0; i < 7;  ++i) h += "tau_J_"   + std::to_string(i) + ";";
    for (int i = 0; i < 7;  ++i) h += "tau_ext_" + std::to_string(i) + ";";
    for (int i = 0; i < 16; ++i) h += "O_T_EE_"  + std::to_string(i) + ";";
    for (int i = 0; i < 6;  ++i) h += "F_ext_"   + std::to_string(i) + ";";
    h += "state\n";
    return h;
}

inline std::string armLogRow(const ArmLogEntry& e) {
    std::string r = std::to_string(e.time) + ";";
    for (auto v : e.q)      r += std::to_string(v) + ";";
    for (auto v : e.dq)     r += std::to_string(v) + ";";
    for (auto v : e.tau_J)  r += std::to_string(v) + ";";
    for (auto v : e.tau_ext)r += std::to_string(v) + ";";
    for (auto v : e.O_T_EE) r += std::to_string(v) + ";";
    for (auto v : e.F_ext)  r += std::to_string(v) + ";";
    r += std::to_string(static_cast<uint8_t>(e.state)) + "\n";
    return r;
}

inline std::string headLogHeader() {
    std::string h = "time;";
    for (int i = 0; i < 2; ++i) h += "q_"     + std::to_string(i) + ";";
    for (int i = 0; i < 2; ++i) h += "dq_"    + std::to_string(i) + ";";
    for (int i = 0; i < 2; ++i) h += "tau_J_" + std::to_string(i) + ";";
    h += "state\n";
    return h;
}

inline std::string headLogRow(const HeadLogEntry& e) {
    std::string r = std::to_string(e.time) + ";";
    for (auto v : e.q)     r += std::to_string(v) + ";";
    for (auto v : e.dq)    r += std::to_string(v) + ";";
    for (auto v : e.tau_J) r += std::to_string(v) + ";";
    r += std::to_string(static_cast<uint8_t>(e.state)) + "\n";
    return r;
}