#pragma once

#include <cstdint>
#include <string>
#include <msgpack.hpp>

struct AnnotationMsg {
    double      timestamp  = 0.0;
    std::string label;
    uint8_t     atype      = 0;
    float       confidence = 0.0f;
    float       score      = 0.0f;
    uint64_t    frame_id   = 0;
    MSGPACK_DEFINE_MAP(timestamp, label, atype, confidence, score, frame_id)
};
