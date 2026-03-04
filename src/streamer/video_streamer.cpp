#include "streamer/video_streamer.hpp"

#include <stdexcept>
#include <iostream>
#include <cstring>

#include <gst/app/gstappsrc.h>

VideoStreamer::VideoStreamer(const StreamerConfig& config)
    : config_(config)
{
    gst_init(nullptr, nullptr);
    source_ = std::make_unique<MuJoCoSource>(config.shm_name, config.fps);
}

VideoStreamer::~VideoStreamer() {
    stop();
}

void VideoStreamer::start() {
    buildPipeline();

    loop_ = g_main_loop_new(nullptr, FALSE);
    loop_thread_ = std::thread([this]() { g_main_loop_run(loop_); });

    GstBus* bus = gst_element_get_bus(pipeline_);
    gst_bus_add_watch(bus, [](GstBus*, GstMessage* msg, gpointer) -> gboolean {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError* err; gchar* dbg;
            gst_message_parse_error(msg, &err, &dbg);
            std::cerr << "[GST ERROR] " << err->message << std::endl;
            std::cerr << "[GST DEBUG] " << (dbg ? dbg : "none") << std::endl;
            g_error_free(err);
            g_free(dbg);
        }
        return TRUE;
    }, nullptr);
    gst_object_unref(bus);

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    bRunning_ = true;
    source_->start([this](const uint8_t* rgb, uint32_t w, uint32_t h) {
        pushFrame(rgb, w, h);
    });

    std::cout << "[INFO] Streamer running on "
              << config_.host << ":" << config_.port
              << " " << source_->width() << "x" << source_->height()
              << " @ " << config_.fps << "fps" << std::endl;
}

void VideoStreamer::stop() {
    bRunning_ = false;
    if (source_) source_->stop();

    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        appsrc_   = nullptr;
    }

    if (loop_) {
        g_main_loop_quit(loop_);
        if (loop_thread_.joinable()) loop_thread_.join();
        g_main_loop_unref(loop_);
        loop_ = nullptr;
    }
}

void VideoStreamer::buildPipeline() {
    std::string pipeline_str =
        "appsrc name=src stream-type=0 format=3 is-live=true block=false"
        " caps=video/x-raw,format=RGB"
        ",width="      + std::to_string(source_->width())  +
        ",height="     + std::to_string(source_->height()) +
        ",framerate="  + std::to_string(config_.fps) + "/1"
        " ! queue max-size-buffers=2 leaky=downstream"
        " ! videoconvert"
        " ! video/x-raw,format=I420"
        " ! x264enc tune=zerolatency speed-preset=ultrafast"
        " bitrate="     + std::to_string(config_.bitrate_kbps) +
        " key-int-max=30"
        " ! rtph264pay pt=96"
        " ! rtpulpfecenc percentage=" + std::to_string(config_.fec_percentage) +
        " ! udpsink host=" + config_.host +
        " port="           + std::to_string(config_.port);

    std::cout << "[INFO] Pipeline: " << pipeline_str << std::endl;

    GError* err = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &err);
    if (err) {
        std::string msg = err->message;
        g_error_free(err);
        throw std::runtime_error("GStreamer pipeline error: " + msg);
    }

    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    if (!appsrc_)
        throw std::runtime_error("Failed to get appsrc element from pipeline");

    gst_app_src_set_stream_type(GST_APP_SRC(appsrc_), GST_APP_STREAM_TYPE_STREAM);
    gst_app_src_set_latency(GST_APP_SRC(appsrc_), 0, -1);
    gst_app_src_set_max_bytes(GST_APP_SRC(appsrc_), 0);
}

void VideoStreamer::pushFrame(const uint8_t* rgb, uint32_t width, uint32_t height) {
    if (!appsrc_ || !bRunning_) return;

    size_t size = width * height * 3;

    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    if (!buffer) return;

    GST_BUFFER_PTS(buffer) = frame_count_ * GST_SECOND / config_.fps;
    GST_BUFFER_DURATION(buffer) = GST_SECOND / config_.fps;
    frame_count_++;

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        gst_buffer_unref(buffer);
        return;
    }

    std::memcpy(map.data, rgb, size);
    gst_buffer_unmap(buffer, &map);

    gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
}