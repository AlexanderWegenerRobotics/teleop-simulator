#include <csignal>
#include <iostream>
#include <memory>

#include <yaml-cpp/yaml.h>

#include "streamer/video_streamer.hpp"

static std::unique_ptr<VideoStreamer> g_streamer;

static std::atomic<bool> g_running{true};
static void onSignal(int) {
    g_running = false;
}

int main(int argc, char** argv) {

    std::cout << "[INFO]: Initialize streamer." << std::endl;

    std::string config_path = "../config/streamer_config.yaml";
    if (argc > 1) config_path = argv[1];

    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load config: " << e.what() << std::endl;
        return 1;
    }

    StreamerConfig config{
        .host           = cfg["host"].as<std::string>(),
        .port           = cfg["port"].as<int>(),
        .feedback_port  = cfg["feedback_port"].as<int>(5005),
        .fps            = cfg["fps"].as<int>(),
        .bitrate_kbps   = cfg["bitrate_kbps"].as<int>(),
        .fec_percentage = cfg["fec_percentage"].as<int>(),
        .shm_name       = cfg["shm_name"].as<std::string>()
    };

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    try {
        g_streamer = std::make_unique<VideoStreamer>(config);
        std::cout << "[INFO]: Trying to start streamer." << std::endl;
        g_streamer->start();
        std::cout << "[INFO] Streamer running. Ctrl+C to stop." << std::endl;

        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        g_streamer->stop();
        g_streamer.reset();
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}