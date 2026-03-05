#pragma once

#include <cstdint>

struct StreamFeedbackMsg {
    uint64_t timestamp_ns;
    float    loss_rate;        // 0.0 – 1.0
    float    jitter_ms;
    float    rtt_ms;
    uint32_t packets_received;
    uint32_t packets_lost;
    uint32_t frames_received;
};

struct FrameTimestamp {
    uint64_t frame_number;
    uint64_t sender_timestamp_ns;
};