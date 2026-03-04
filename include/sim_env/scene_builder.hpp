#pragma once

#include <string>
#include <vector>
#include <array>
#include <filesystem>

#include <yaml-cpp/yaml.h>


// ---------------------------------------------------------------------------
// Config structs
// ---------------------------------------------------------------------------

struct DeviceConfig {
    std::string            name;
    std::string            type;
    bool                   enabled;
    std::string            model_path;
    std::string            root_body;
    std::array<double, 3>  position;
    std::array<double, 4>  orientation;   // MuJoCo quaternion: w x y z
    std::vector<double>    q0;

    // Optional: unprefixed actuator name for the gripper (e.g. "actuator8").
    // Leave empty for devices without a gripper (pan-tilt, etc.).
    // At runtime the full name is "<device_name>_<gripper_actuator>".
    std::string            gripper_actuator;
};

struct ObjectConfig {
    std::string            name;
    std::string            type;          // "static" | "visual" | ...
    std::string            model_path;
    std::array<double, 3>  position;
    std::array<double, 4>  orientation;   // w x y z
};

struct CameraConfig {
    std::string            name;
    std::string            type;          // "fixed" | ...
    std::array<double, 3>  pos;
    std::array<double, 4>  quat;          // w x y z, computed from look_at during parsing
    double                 fovy;
};

// ---------------------------------------------------------------------------
// Result returned from SceneBuilder::build()
// ---------------------------------------------------------------------------

struct BuiltScene {
    std::filesystem::path      xml_path;   // path to the written _generated_scene.xml
    std::vector<DeviceConfig>  devices;
    std::vector<ObjectConfig>  objects;
    std::vector<CameraConfig>  cameras;
};

// ---------------------------------------------------------------------------
// SceneBuilder
// ---------------------------------------------------------------------------

class SceneBuilder {
public:
    // Parse config, assemble XML, write it to disk, return BuiltScene.
    // Throws std::runtime_error on any failure.
    static BuiltScene build(const YAML::Node& config);

private:
    // --- YAML parsing ---
    static std::vector<DeviceConfig> parseDevices(const YAML::Node& config);
    static std::vector<ObjectConfig> parseObjects(const YAML::Node& config);
    static std::vector<CameraConfig> parseCameras(const YAML::Node& config);

    // Convert look_at + pos into a MuJoCo camera quaternion (camera looks along -Z, +Y up)
    static std::array<double, 4> lookAtToQuat(const std::array<double, 3>& pos,
                                               const std::array<double, 3>& look_at);

    // --- XML assembly ---
    static std::string buildSceneXML(const std::vector<DeviceConfig>& devices,
                                     const std::vector<ObjectConfig>&  objects,
                                     const std::vector<CameraConfig>&  cameras,
                                     const std::string&                baseScenePath);
};