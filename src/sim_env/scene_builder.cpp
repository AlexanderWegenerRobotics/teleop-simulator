#include "sim_env/scene_builder.hpp"

#include <fstream>
#include <stdexcept>
#include <filesystem>
#include <cmath>
#include <cstdio>

#include <tinyxml2.h>

using namespace tinyxml2;


// ===========================================================================
// Internal TinyXML-2 helpers  (file-scope, not exposed in the header)
// ===========================================================================

static std::unique_ptr<XMLDocument> loadXMLDoc(const std::string& path) {
    auto doc = std::make_unique<XMLDocument>();
    if (doc->LoadFile(path.c_str()) != XML_SUCCESS)
        throw std::runtime_error("TinyXML2 failed to load: " + path +
                                 "  (" + doc->ErrorStr() + ")");
    return doc;
}

// Deep-copy every child of src into dst (print/parse round-trip keeps ownership clean)
static void deepCopyChildren(XMLElement* dst, XMLDocument& dstDoc, const XMLElement* src) {
    for (const XMLNode* child = src->FirstChild(); child; child = child->NextSibling()) {
        XMLPrinter printer;
        child->Accept(&printer);

        XMLDocument tmp;
        if (tmp.Parse(printer.CStr()) == XML_SUCCESS && tmp.FirstChild())
            dst->InsertEndChild(tmp.FirstChild()->DeepClone(&dstDoc));
    }
}

static std::string docToString(XMLDocument& doc) {
    XMLPrinter printer;
    doc.Print(&printer);
    return printer.CStr();
}

// Merge all attributes from src <compiler> into dst, skipping meshdir
// (paths are resolved to absolute separately).
static void mergeCompilerAttributes(XMLElement* dst, const XMLElement* src) {
    for (const XMLAttribute* a = src->FirstAttribute(); a; a = a->Next()) {
        if (std::string(a->Name()) == "meshdir") continue;
        dst->SetAttribute(a->Name(), a->Value());
    }
}

// Recursively prefix every name-like attribute in the tree with "<prefix>_"
static void prefixNamesInTree(XMLElement* el, const std::string& prefix) {
    static const std::vector<const char*> kAttrs = {
        "name", "joint", "joint1", "joint2",
        "body", "body1", "body2",
        "mesh", "material", "tendon",
        "site", "actuator", "childclass", "class"
    };
    for (const char* attr : kAttrs) {
        const char* val = el->Attribute(attr);
        if (val)
            el->SetAttribute(attr, (prefix + "_" + val).c_str());
    }
    for (XMLElement* ch = el->FirstChildElement(); ch; ch = ch->NextSiblingElement())
        prefixNamesInTree(ch, prefix);
}

// MuJoCo derives a mesh name from the filename stem when name= is absent.
// Materialise that name explicitly so it survives prefixing.
static void normalizeMeshNames(XMLElement* assetEl) {
    for (XMLElement* el = assetEl->FirstChildElement("mesh");
         el; el = el->NextSiblingElement("mesh")) {
        if (!el->Attribute("name")) {
            const char* file = el->Attribute("file");
            if (file)
                el->SetAttribute("name",
                    std::filesystem::path(file).stem().string().c_str());
        }
    }
}

// Rewrite relative mesh file= paths to absolute using the resolved meshdir
static void absoluteMeshPaths(XMLElement* assetEl, const std::string& meshdir) {
    for (XMLElement* el = assetEl->FirstChildElement("mesh");
         el; el = el->NextSiblingElement("mesh")) {
        const char* file = el->Attribute("file");
        if (file && file[0] != '/')
            el->SetAttribute("file", (meshdir + "/" + file).c_str());
    }
}

// Resolve the meshdir declared in a robot/object compiler element to an absolute path
static std::string resolveMeshdir(const XMLElement* compilerEl,
                                   const std::string& modelPath) {
    if (!compilerEl) return {};
    const char* md = compilerEl->Attribute("meshdir");
    if (!md) return {};
    return std::filesystem::absolute(
        std::filesystem::path(modelPath).parent_path() / md).string();
}

// Inject one XML model (robot or prop) into the scene, prefixing all names.
// Returns without error if the document has no worldbody (shouldn't happen).
static void injectModel(const std::string& modelPath,
                        const std::string& namePrefix,
                        const std::array<double, 3>& pos,
                        const std::array<double, 4>& quat,
                        XMLDocument& scene,
                        XMLElement*  compilerEl,
                        XMLElement*  assetEl,
                        XMLElement*  worldbody,
                        XMLElement*  root) {
    auto doc = loadXMLDoc(modelPath);
    XMLElement* docRoot = doc->RootElement();

    // 1. Merge compiler flags (autolimits, angle, …) — skip meshdir
    if (XMLElement* comp = docRoot->FirstChildElement("compiler"))
        mergeCompilerAttributes(compilerEl, comp);

    // 2. Resolve meshdir before we mangle any attributes
    std::string meshdir = resolveMeshdir(
        docRoot->FirstChildElement("compiler"), modelPath);

    // 3. Normalise unnamed meshes so prefixing keeps geom references in sync
    if (XMLElement* ra = docRoot->FirstChildElement("asset"))
        normalizeMeshNames(ra);

    // 4. Prefix every name in the document
    prefixNamesInTree(docRoot, namePrefix);

    // 5. <default> blocks
    for (XMLElement* def = docRoot->FirstChildElement("default");
         def; def = def->NextSiblingElement("default"))
        root->InsertEndChild(def->DeepClone(&scene));

    // 6. <asset> children
    if (XMLElement* ra = docRoot->FirstChildElement("asset")) {
        if (!meshdir.empty())
            absoluteMeshPaths(ra, meshdir);
        deepCopyChildren(assetEl, scene, ra);
    }

    // 7. <worldbody> children wrapped in a named frame body
    if (XMLElement* rw = docRoot->FirstChildElement("worldbody")) {
        XMLElement* frame = scene.NewElement("body");
        frame->SetAttribute("name", (namePrefix + "_frame").c_str());

        char posBuf[64], quatBuf[80];
        std::snprintf(posBuf,  sizeof(posBuf),  "%.6g %.6g %.6g",
                      pos[0], pos[1], pos[2]);
        std::snprintf(quatBuf, sizeof(quatBuf), "%.6g %.6g %.6g %.6g",
                      quat[0], quat[1], quat[2], quat[3]);
        frame->SetAttribute("pos",  posBuf);
        frame->SetAttribute("quat", quatBuf);

        deepCopyChildren(frame, scene, rw);
        worldbody->InsertEndChild(frame);
    }

    // 8. Remaining top-level blocks (actuator, tendon, equality, contact)
    for (const char* tag : {"actuator", "tendon", "equality", "contact"})
        for (XMLElement* el = docRoot->FirstChildElement(tag);
             el; el = el->NextSiblingElement(tag))
            root->InsertEndChild(el->DeepClone(&scene));
}


// ===========================================================================
// SceneBuilder — public interface
// ===========================================================================

BuiltScene SceneBuilder::build(const YAML::Node& config) {
    BuiltScene result;
    result.devices = parseDevices(config);
    result.objects = parseObjects(config);
    result.cameras = parseCameras(config);

    std::string baseScenePath;
    if (config["simulation"] && config["simulation"]["model_path"])
        baseScenePath = config["simulation"]["model_path"].as<std::string>();

    std::string xml = buildSceneXML(result.devices, result.objects,
                                    result.cameras, baseScenePath);

    result.xml_path =
        std::filesystem::path(baseScenePath).parent_path() / "_generated_scene.xml";

    {
        std::ofstream out(result.xml_path);
        if (!out.is_open())
            throw std::runtime_error("Cannot write generated scene: " +
                                     result.xml_path.string());
        out << xml;
    }
    return result;
}


// ===========================================================================
// YAML parsing
// ===========================================================================

std::vector<DeviceConfig> SceneBuilder::parseDevices(const YAML::Node& config) {
    std::vector<DeviceConfig> devices;
    if (!config["devices"]) return devices;

    for (const auto& node : config["devices"]) {
        if (node["enabled"] && !node["enabled"].as<bool>()) continue;

        DeviceConfig dev;
        dev.name       = node["name"].as<std::string>();
        dev.type       = node["type"].as<std::string>();
        dev.enabled    = node["enabled"] ? node["enabled"].as<bool>() : true;
        dev.model_path = node["model_path"].as<std::string>();
        dev.root_body  = node["root_body"].as<std::string>();

        auto pos = node["base_pose"]["position"];
        dev.position = {pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>()};

        auto ori = node["base_pose"]["orientation"];
        dev.orientation = {ori[0].as<double>(), ori[1].as<double>(),
                           ori[2].as<double>(), ori[3].as<double>()};

        if (node["q0"])
            for (const auto& q : node["q0"])
                dev.q0.push_back(q.as<double>());

        if (node["gripper_actuator"])
            dev.gripper_actuator = node["gripper_actuator"].as<std::string>();

        devices.push_back(dev);
    }
    return devices;
}

std::vector<ObjectConfig> SceneBuilder::parseObjects(const YAML::Node& config) {
    std::vector<ObjectConfig> objects;
    if (!config["objects"]) return objects;

    for (const auto& node : config["objects"]) {
        ObjectConfig obj;
        obj.name       = node["name"].as<std::string>();
        obj.type       = node["type"].as<std::string>();
        obj.model_path = node["model_path"].as<std::string>();

        auto pos = node["pose"]["position"];
        obj.position = {pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>()};

        auto ori = node["pose"]["orientation"];
        obj.orientation = {ori[0].as<double>(), ori[1].as<double>(),
                           ori[2].as<double>(), ori[3].as<double>()};

        objects.push_back(obj);
    }
    return objects;
}

std::vector<CameraConfig> SceneBuilder::parseCameras(const YAML::Node& config) {
    std::vector<CameraConfig> cameras;
    if (!config["cameras"]) return cameras;

    for (const auto& node : config["cameras"]) {
        CameraConfig cam;
        cam.name = node["name"].as<std::string>();
        cam.type = node["type"].as<std::string>();
        cam.fovy = node["fovy"].as<double>();

        auto p = node["pos"];
        cam.pos = {p[0].as<double>(), p[1].as<double>(), p[2].as<double>()};

        auto la = node["look_at"];
        std::array<double, 3> look_at = {
            la[0].as<double>(), la[1].as<double>(), la[2].as<double>()};

        cam.quat = lookAtToQuat(cam.pos, look_at);
        cameras.push_back(cam);
    }
    return cameras;
}

// ---------------------------------------------------------------------------
// look_at → MuJoCo camera quaternion
//
// MuJoCo camera convention: looks along -Z, +Y is up in camera space.
// We build a rotation matrix whose columns are the world-space right/up/back
// vectors of the camera, then convert to quaternion.
// ---------------------------------------------------------------------------
std::array<double, 4> SceneBuilder::lookAtToQuat(const std::array<double, 3>& pos,
                                                   const std::array<double, 3>& look_at) {
    // Forward vector (from camera toward target) in world space
    double fx = look_at[0] - pos[0];
    double fy = look_at[1] - pos[1];
    double fz = look_at[2] - pos[2];
    double flen = std::sqrt(fx*fx + fy*fy + fz*fz);
    if (flen < 1e-9) return {1.0, 0.0, 0.0, 0.0}; // degenerate — identity
    fx /= flen; fy /= flen; fz /= flen;

    // World up hint — switch to X if forward is nearly parallel to Z
    double wx = 0.0, wy = 0.0, wz = 1.0;
    if (std::abs(fz) > 0.999) { wx = 1.0; wy = 0.0; wz = 0.0; }

    // Right = forward × up_hint
    double rx = fy*wz - fz*wy;
    double ry = fz*wx - fx*wz;
    double rz = fx*wy - fy*wx;
    double rlen = std::sqrt(rx*rx + ry*ry + rz*rz);
    rx /= rlen; ry /= rlen; rz /= rlen;

    // Camera up = right × forward
    double ux = ry*fz - rz*fy;
    double uy = rz*fx - rx*fz;
    double uz = rx*fy - ry*fx;

    // Rotation matrix R maps world axes to camera frame:
    //   camera -Z = forward  →  R col2 =  (fx, fy, fz)  but stored as -Z so negate
    //   camera +Y = up       →  R col1 =  (ux, uy, uz)
    //   camera +X = right    →  R col0 =  (rx, ry, rz)
    //
    // Written as a 3×3 row-major matrix where R * e_camX = world_right etc.:
    //   | rx  ux  -fx |
    //   | ry  uy  -fy |
    //   | rz  uz  -fz |
    double m00 = rx,  m01 = ux,  m02 = -fx;
    double m10 = ry,  m11 = uy,  m12 = -fy;
    double m20 = rz,  m21 = uz,  m22 = -fz;

    // Rotation matrix → quaternion (Shepperd method)
    double trace = m00 + m11 + m22;
    double qw, qx, qy, qz;
    if (trace > 0.0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (m21 - m12) * s;
        qy = (m02 - m20) * s;
        qz = (m10 - m01) * s;
    } else if (m00 > m11 && m00 > m22) {
        double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
        qw = (m21 - m12) / s;
        qx = 0.25 * s;
        qy = (m01 + m10) / s;
        qz = (m02 + m20) / s;
    } else if (m11 > m22) {
        double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
        qw = (m02 - m20) / s;
        qx = (m01 + m10) / s;
        qy = 0.25 * s;
        qz = (m12 + m21) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
        qw = (m10 - m01) / s;
        qx = (m02 + m20) / s;
        qy = (m12 + m21) / s;
        qz = 0.25 * s;
    }

    // MuJoCo quaternion convention: w x y z
    return {qw, qx, qy, qz};
}


// ===========================================================================
// XML assembly
// ===========================================================================

std::string SceneBuilder::buildSceneXML(const std::vector<DeviceConfig>& devices,
                                         const std::vector<ObjectConfig>&  objects,
                                         const std::vector<CameraConfig>&  cameras,
                                         const std::string&                baseScenePath) {
    XMLDocument scene;
    XMLElement* root = scene.NewElement("mujoco");
    root->SetAttribute("model", "scene");
    scene.InsertFirstChild(root);

    // Bare compiler element — attributes are merged in from each loaded model
    XMLElement* compilerEl = scene.NewElement("compiler");
    root->InsertEndChild(compilerEl);

    XMLElement* optionEl = scene.NewElement("option");
    optionEl->SetAttribute("gravity", "0 0 -9.81");
    optionEl->SetAttribute("integrator", "implicitfast");
    root->InsertEndChild(optionEl);

    XMLElement* assetEl   = scene.NewElement("asset");
    XMLElement* worldbody = scene.NewElement("worldbody");
    root->InsertEndChild(assetEl);
    root->InsertEndChild(worldbody);

    // -----------------------------------------------------------------------
    // Base scene (lights, ground plane, etc.)
    // -----------------------------------------------------------------------
    {
        auto baseDoc  = loadXMLDoc(baseScenePath);
        XMLElement* baseRoot = baseDoc->RootElement();
        if (XMLElement* ba = baseRoot->FirstChildElement("asset"))
            deepCopyChildren(assetEl, scene, ba);
        if (XMLElement* bw = baseRoot->FirstChildElement("worldbody"))
            deepCopyChildren(worldbody, scene, bw);
    }

    // -----------------------------------------------------------------------
    // Devices  (actuated robots)
    // -----------------------------------------------------------------------
    for (const auto& dev : devices)
        injectModel(dev.model_path, dev.name,
                    dev.position, dev.orientation,
                    scene, compilerEl, assetEl, worldbody, root);

    // -----------------------------------------------------------------------
    // Objects  (static props, visual markers)
    // -----------------------------------------------------------------------
    for (const auto& obj : objects)
        injectModel(obj.model_path, obj.name,
                    obj.position, obj.orientation,
                    scene, compilerEl, assetEl, worldbody, root);

    // -----------------------------------------------------------------------
    // Cameras  (injected directly as <camera> elements in worldbody)
    // -----------------------------------------------------------------------
    for (const auto& cam : cameras) {
        XMLElement* camEl = scene.NewElement("camera");
        camEl->SetAttribute("name", cam.name.c_str());

        char posBuf[64], quatBuf[80], fovBuf[32];
        std::snprintf(posBuf,  sizeof(posBuf),  "%.6g %.6g %.6g",
                      cam.pos[0], cam.pos[1], cam.pos[2]);
        std::snprintf(quatBuf, sizeof(quatBuf), "%.6g %.6g %.6g %.6g",
                      cam.quat[0], cam.quat[1], cam.quat[2], cam.quat[3]);
        std::snprintf(fovBuf,  sizeof(fovBuf),  "%.6g", cam.fovy);

        camEl->SetAttribute("pos",  posBuf);
        camEl->SetAttribute("quat", quatBuf);
        camEl->SetAttribute("fovy", fovBuf);

        worldbody->InsertEndChild(camEl);
    }

    return docToString(scene);
}