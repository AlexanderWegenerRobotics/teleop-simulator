#pragma once

#include <cstdint>

struct StreamFeedbackMsg {
    uint64_t timestamp_ns;
    float    loss_rate;
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

struct StreamStatusMsg {
    uint32_t magic;
    uint8_t  state;
    uint64_t timestamp_ns;
} __attribute__((packed));

static constexpr uint32_t kStreamStatusMagic = 0x53545354;