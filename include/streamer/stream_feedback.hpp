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
