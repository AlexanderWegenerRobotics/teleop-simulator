#pragma once

#include <cstdint>
#include <Eigen/Dense>

enum class SysState : uint8_t {
    OFFLINE  = 0,  // not initialized / not connected
    IDLE     = 1,  // initialized, gravity comp only, waiting
    HOMING   = 2,  // moving to home configuration
    AWAITING = 3,  // at home, waiting for operator to engage
    ENGAGED  = 4,  // actively tracking operator input (your TELEOP)
    PAUSED   = 5,  // holding current pose, operator temporarily disengaged
    FAULT    = 6,  // error state, safe torques only
    STOP     = 7,
};

enum class TransmissionRole : uint8_t {
    ARM    = 0,
    HEAD   = 1,
    AVATAR = 2
};

using Matrix6x7 = Eigen::Matrix<double, 6, 7>;
using Matrix7   = Eigen::Matrix<double, 7, 7>;
using Matrix4   = Eigen::Matrix<double, 4, 4>;
using Vector7   = Eigen::Matrix<double, 7, 1>;
using Vector2   = Eigen::Matrix<double, 2, 1>;

#pragma pack(push, 1)

struct ArmCommandMsg {
    uint8_t  device_id;
    SysState state;
    float    position[3];
    float    quaternion[4];
    float    gripper;
    uint64_t timestamp_ns;
};

struct ArmStateMsg {
    uint8_t  device_id;
    SysState state;
    float    position[3];
    float    quaternion[4];
    float    tau_ext[7];
    uint64_t timestamp_ns;
};

struct HeadCommandMsg {
    uint8_t  device_id;
    SysState state;
    float    pan;
    float    tilt;
    uint64_t timestamp_ns;
};

struct HeadStateMsg {
    uint8_t  device_id;
    SysState state;
    float    pan;
    float    tilt;
    uint64_t timestamp_ns;
};

struct AvatarCommandMsg {
    SysState requested_state;
    uint8_t  session_id;
    uint64_t timestamp_ns;
};

struct AvatarStateMsg {
    SysState system_state;
    uint64_t timestamp_ns;
};

#pragma pack(pop)