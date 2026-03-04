#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <cmath>

#include <GLFW/glfw3.h>
#include <yaml-cpp/yaml.h>

#include "avatar.hpp"

int main() {
    try {
        YAML::Node config = YAML::LoadFile("../config/config.yaml");

        if (!glfwInit())
            throw std::runtime_error("glfwInit failed");

        Avatar avatar(config);

#ifndef WITH_FRANKA
        auto sim = avatar.getSim();

        if (sim->render_enabled_) {
            int ncam = static_cast<int>(sim->render_cams_.size());
            int cols = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(ncam))));
            int rows = (ncam + cols - 1) / cols;

            glfwWindowHint(GLFW_SAMPLES, 4);
            sim->window_ = glfwCreateWindow(cols * 640, rows * 480,
                                            "avatar — simulation", nullptr, nullptr);
            if (!sim->window_)
                throw std::runtime_error("glfwCreateWindow failed");
        }
        if (sim->shm_enabled_) {
            glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
            sim->offscreen_window_ = glfwCreateWindow(1, 1, "", nullptr, nullptr);
            if (!sim->offscreen_window_)
                throw std::runtime_error("Failed to create offscreen window for streaming");
        }

        sim->start();
        std::thread avatar_thread([&]() {
            avatar.start();
        });

        while (sim->isRunning()) {
            if (sim->window_ && glfwWindowShouldClose(sim->window_)) {
                sim->stop();
                break;
            }
            glfwPollEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        avatar.stop();
        sim->stop();

        if (avatar_thread.joinable()) avatar_thread.join();
#endif

        glfwTerminate();

    } catch (const std::exception& e) {
        std::cerr << "[Fatal] " << e.what() << "\n";
        return 1;
    }
    std::cout << "Clean close" << std::endl;
    return 0;
}