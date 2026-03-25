#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include "streamer/camera_source.hpp"
#include "streamer/stream_quality_controller.hpp"

struct StreamerConfig {
    std::string host;
    int         port;
    int         feedback_port;
    int         fps;
    int         bitrate_kbps;
    int         fec_percentage;
    std::string shm_name;
    std::string source_type;
    std::string realsense_serial;
    int         stream_width;
    int         stream_height;
};

class VideoStreamer {
public:
    static constexpr size_t kTimestampBytes = 8;

    VideoStreamer(const StreamerConfig& config);
    ~VideoStreamer();

    void start();
    void stop();

private:
    void buildPipeline();
    void pushFrame(const uint8_t* rgb, uint32_t width, uint32_t height);

private:
    StreamerConfig                 config_;
    std::unique_ptr<CameraSource>  source_;

    GstElement*       pipeline_    = nullptr;
    GstElement*       appsrc_      = nullptr;
    GMainLoop*        loop_        = nullptr;
    std::thread       loop_thread_;
    std::atomic<bool> bRunning_{false};
    uint64_t          frame_count_ = 0;

    GstElement* encoder_ = nullptr;
    GstElement* fec_     = nullptr;
    std::unique_ptr<StreamQualityController> quality_;
    std::atomic<int> target_fps_{0};
};
