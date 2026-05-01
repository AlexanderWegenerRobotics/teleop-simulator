#include "avatar.hpp"
#include "common.hpp"
#include "network/udp_reliable.hpp"

#include <iostream>
#include <filesystem>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

Avatar::Avatar(const YAML::Node& config) {
    YAML::Node sys_config = YAML::LoadFile(config["robot_config"].as<std::string>());

    session_id_ = std::to_string(
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    for (const auto& dev : sys_config["devices"]) {
        if (!dev["enabled"].as<bool>(true)) continue;
        std::string type = dev["type"].as<std::string>();
        std::string name = dev["name"].as<std::string>();

        if (type == "arm") {
            arm_instances.push_back(new ArmControl(dev, session_id_));
            device_records_[name] = DeviceRecord{};
        } else if (type == "head") {
            head_instances.push_back(new HeadControl(dev, session_id_));
            device_records_[name] = DeviceRecord{};
        }
    }

    device_registry_ = std::make_shared<DeviceRegistry>();

    SelfCollisionConfig scp_config;
    if (sys_config["avatar"]["self_collision"]) {
        auto sc = sys_config["avatar"]["self_collision"];
        scp_config.d_min   = sc["d_min"].as<double>(0.04);
        scp_config.alpha   = sc["alpha"].as<double>(1.0);
        scp_config.beta    = sc["beta"].as<double>(0.10);
        scp_config.enabled = sc["enabled"].as<bool>(true);
    }

    for (auto& arm : arm_instances) {
        arm->initSelfCollisionProtection(device_registry_, scp_config);
    }

    getArm("arm_right")->setCollisionImportanceWeight(1.0);
    getArm("arm_left")->setCollisionImportanceWeight(1.0);

    log_base_dir_ = "logs";
    if (sys_config["avatar"]["log_dir"])
        log_base_dir_ = sys_config["avatar"]["log_dir"].as<std::string>();
    std::filesystem::create_directories(log_base_dir_);

    episode_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    struct timeval tv{};
    tv.tv_sec  = 1;
    tv.tv_usec = 0;
    setsockopt(episode_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (sys_config["avatar"]["transmission"]) {
        UdpReliableConfig cmd_cfg;
        cmd_cfg.transport.remote_ip   = sys_config["avatar"]["transmission"]["remote_ip"].as<std::string>();
        cmd_cfg.transport.remote_port = sys_config["avatar"]["transmission"]["send_port"].as<int>();
        cmd_cfg.transport.bind_port   = sys_config["avatar"]["transmission"]["receive_port"].as<int>();
        cmd_cfg.poll_rate_hz          = sys_config["avatar"]["transmission"]["frequency"].as<int>();
        cmd_channel_ = std::make_unique<UdpReliable>(cmd_cfg);

        cmd_channel_->registerHandler("state_change",[this](const ReliableEnvelope& env, const msgpack::object& payload) {
            std::map<std::string, msgpack::object> fields;
            payload.convert(fields);
            auto it = fields.find("requested_state");
            if (it != fields.end()) {
                cmd_requested_.store(static_cast<SysState>(it->second.as<uint8_t>()));
            }
        });

        cmd_channel_->registerHandler("arm_reset", [this](const ReliableEnvelope& env, const msgpack::object& payload) {
            std::map<std::string, msgpack::object> fields;
            payload.convert(fields);
            auto it = fields.find("device");
            if (it == fields.end()) return;
            std::string dev_name = it->second.as<std::string>();

            ArmControl* arm = getArm(dev_name);
            if (!arm) return;

            arm->recovery().requestRecovery(RecoveryTrigger::OPERATOR_RESET, arm->getQ0());

            auto rec_it = device_records_.find(dev_name);
            if (rec_it != device_records_.end())
                rec_it->second.active = false;

            std::cout << "[AVATAR-INFO]: Reset requested for " << dev_name << std::endl;
        });

        cmd_channel_->registerHandler("arm_resume", [this](const ReliableEnvelope& env, const msgpack::object& payload) {
            if (reset_all_pending_.load()) return;

            std::map<std::string, msgpack::object> fields;
            payload.convert(fields);
            auto it = fields.find("device");
            if (it == fields.end()) return;
            std::string dev_name = it->second.as<std::string>();

            ArmControl* arm = getArm(dev_name);
            if (!arm || !arm->recovery().isWaitingAck()) return;

            arm->reOrigin();
            arm->recovery().confirmResume();

            auto rec_it = device_records_.find(dev_name);
            if (rec_it != device_records_.end())
                rec_it->second.active = true;

            if (state_ == SysState::ENGAGED) {
                arm->requestState(SysState::ENGAGED);
            }

            std::cout << "[AVATAR-INFO]: Resume confirmed for " << dev_name << std::endl;
        });

        cmd_channel_->registerHandler("reset_all", [this](const ReliableEnvelope& env, const msgpack::object& payload) {
            std::string reason = "reset_all";
            if (payload.type == msgpack::type::MAP) {
                std::map<std::string, msgpack::object> fields;
                payload.convert(fields);
                auto it = fields.find("reason");
                if (it != fields.end())
                    reason = it->second.as<std::string>();
            }

            markEpisodeEnd(reason);

            for (auto& arm : arm_instances) {
                arm->recovery().requestRecovery(RecoveryTrigger::OPERATOR_RESET, arm->getQ0());
                auto rec_it = device_records_.find(arm->getDeviceName());
                if (rec_it != device_records_.end())
                    rec_it->second.active = false;
            }

            reset_all_pending_.store(true);

            std::cout << "[AVATAR-INFO]: Global reset requested (" << reason << ")" << std::endl;
        });
    }

#ifndef WITH_FRANKA
    sim_ = std::make_shared<Simulation>(config);

    YAML::Node sim_config = YAML::LoadFile(config["sim_config"].as<std::string>());
    std::unordered_map<std::string, YAML::Node> sim_devs;
    for (const auto& sd : sim_config["devices"])
        sim_devs[sd["name"].as<std::string>()] = YAML::Node(sd);

    std::unordered_map<std::string, YAML::Node> robot_devs;
    for (const auto& rd : sys_config["devices"])
        robot_devs[rd["name"].as<std::string>()] = YAML::Node(rd);

    for (auto& arm : arm_instances) {
        arm->robot->set_simulation(*sim_, sim_devs[arm->getDeviceName()], robot_devs[arm->getDeviceName()]);
        std::string gripper_name = "hand_" + arm->getDeviceName().substr(arm->getDeviceName().find('_') + 1);
        if (sim_devs.count(gripper_name)){
            arm->gripper->set_simulation(*sim_, gripper_name);
        }
    }
    for (auto& head : head_instances)
        head->module->set_simulation(*sim_, sim_devs[head->getDeviceName()]);

    current_episode_cfg_ = requestEpisodeConfig();
    applyEpisodeConfig(current_episode_cfg_);
#endif
}

Avatar::~Avatar() {
    if (episode_sock_ >= 0)
        close(episode_sock_);
}

void Avatar::start(){
    if (cmd_channel_) cmd_channel_->start();
    for(const auto& head : head_instances){
        head->start();
    }
    for(const auto& arm : arm_instances){
        arm->start();
    }
    std::cout << "[AVATAR-INFO]: All devices started" << std::endl;

    constexpr std::chrono::microseconds control_period(static_cast<int>(1e6 / 100));
	auto next_control_time = std::chrono::high_resolution_clock::now();

    SysState prev_state = SysState::IDLE;
    state_ = SysState::IDLE;
    if (cmd_channel_) cmd_channel_->setState(SysState::IDLE);
    auto last_heartbeat = std::chrono::steady_clock::now();
    auto start_time = std::chrono::steady_clock::now();
    if (cmd_channel_) cmd_channel_->resetAliveTimer();
    bRunning = true;
   
    while(bRunning){
        SysState cmd_state = cmd_requested_.load();

        if (cmd_channel_ && !cmd_channel_->isAlive()) {
            if (state_ != SysState::IDLE && state_ != SysState::OFFLINE) {
                std::cout << "[AVATAR-WARN]: Operator connection lost, reverting to IDLE." << std::endl;
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
                cmd_requested_.store(SysState::IDLE);
                cmd_channel_->setState(SysState::IDLE);
            }
        } else {
            updateStateMachine(cmd_state);
        }

        processRecoveryNotifications();
        processResetAllCompletion();

        if (state_ != prev_state && cmd_channel_) {
            cmd_channel_->setState(state_);
        }
        prev_state = state_;

        auto now = std::chrono::steady_clock::now();
        if (cmd_channel_ && std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat).count() >= 500) {
            msgpack::sbuffer buf;
            msgpack::pack(buf, std::map<std::string, int64_t>{{"uptime_ms", std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count()}});
            cmd_channel_->send("heartbeat", buf, false);
            last_heartbeat = now;
        }
        #ifndef WITH_FRANKA
            for (auto& arm : arm_instances) {
                Eigen::Isometry3d T = arm->getTargetPose();
                std::string dev = arm->getDeviceName();
                std::string side = dev.substr(dev.find('_') + 1);
                sim_->setFramePose("target_" + side + "_frame", T.translation(), Eigen::Quaterniond(T.rotation()));
            }
        #endif

        next_control_time += control_period;
        std::this_thread::sleep_until(next_control_time);
    }
}

void Avatar::stop(){
    for(const auto& head : head_instances){
        head->stop();
    }
    for(const auto& arm : arm_instances){
        arm->stop();
    }
    if (cmd_channel_) cmd_channel_->stop();
    bRunning = false;
}

void Avatar::processRecoveryNotifications() {
    if (!cmd_channel_) return;
    for (auto& arm : arm_instances) {
        if (arm->recovery().needsNotification()) {
            sendDeviceEvent(arm->getDeviceName(), "reset_complete");
            arm->recovery().clearNotification();
        }
    }
}

Avatar::EpisodeConfig Avatar::requestEpisodeConfig() {
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port   = htons(9100);
    inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr);

    msgpack::sbuffer req_buf;
    msgpack::pack(req_buf, std::map<std::string, std::string>{{"type", "request_episode_config"}});
    sendto(episode_sock_, req_buf.data(), req_buf.size(), 0,
           (sockaddr*)&server_addr, sizeof(server_addr));

    char recv_buf[1024];
    socklen_t addr_len = sizeof(server_addr);
    ssize_t n = recvfrom(episode_sock_, recv_buf, sizeof(recv_buf), 0,
                         (sockaddr*)&server_addr, &addr_len);

    EpisodeConfig cfg{};
    cfg.pick_x  = 0.65; cfg.pick_y  =  0.10; cfg.pick_z  = 0.62;
    cfg.place_x = 0.65; cfg.place_y = -0.10; cfg.place_z = 0.62;
    cfg.mode    = 0;

    if (n <= 0) {
        std::cerr << "[AVATAR-WARN]: Episode config server unreachable, using defaults" << std::endl;
        return cfg;
    }

    try {
        auto oh = msgpack::unpack(recv_buf, n);
        auto obj = oh.get();
        std::map<std::string, msgpack::object> fields;
        obj.convert(fields);
        cfg.pick_x  = fields.at("pick_x").as<double>();
        cfg.pick_y  = fields.at("pick_y").as<double>();
        cfg.pick_z  = fields.at("pick_z").as<double>();
        cfg.place_x = fields.at("place_x").as<double>();
        cfg.place_y = fields.at("place_y").as<double>();
        cfg.place_z = fields.at("place_z").as<double>();
        cfg.mode    = fields.at("mode").as<int>();
    } catch (const std::exception& e) {
        std::cerr << "[AVATAR-WARN]: Failed to parse episode config: " << e.what() << std::endl;
    }

    return cfg;
}

void Avatar::startNewEpisodeFolder() {
    char idx_buf[8];
    std::snprintf(idx_buf, sizeof(idx_buf), "%03d", episode_index_++);
    std::string folder = log_base_dir_ + "/" + std::string(idx_buf);
    std::filesystem::create_directories(folder);

    for (auto& arm : arm_instances) {
        std::string path = folder + "/" + arm->getDeviceName() + ".csv";
        arm->restartLogger(path);
    }
    for (auto& head : head_instances) {
        std::string path = folder + "/" + head->getDeviceName() + ".csv";
        head->restartLogger(path);
    }

    std::cout << "[AVATAR-INFO]: New episode folder: " << folder << std::endl;
}

void Avatar::applyEpisodeConfig(const EpisodeConfig& cfg) {
    std::cout << "[AVATAR-INFO]: Episode config — "
              << "pick=(" << cfg.pick_x << "," << cfg.pick_y << "," << cfg.pick_z << ") "
              << "place=(" << cfg.place_x << "," << cfg.place_y << "," << cfg.place_z << ") "
              << "mode=" << (cfg.mode == 0 ? "unimanual" : "bimanual") << std::endl;

#ifndef WITH_FRANKA
    sim_->setFreeBodyPose("box_1_box",
        Eigen::Vector3d(cfg.pick_x, cfg.pick_y, cfg.pick_z),
        Eigen::Quaterniond::Identity());

    sim_->setFramePose("place_pose_frame",
        Eigen::Vector3d(cfg.place_x, cfg.place_y, cfg.place_z),
        Eigen::Quaterniond::Identity());
#endif
}

void Avatar::processResetAllCompletion() {
    if (!reset_all_pending_.load()) return;

    for (auto& arm : arm_instances) {
        if (!arm->recovery().isWaitingAck()) return;
    }

    current_episode_cfg_ = requestEpisodeConfig();
    applyEpisodeConfig(current_episode_cfg_);

    startNewEpisodeFolder();

    for (auto& arm : arm_instances) {
        arm->reOrigin();
        arm->recovery().confirmResume();
        auto rec_it = device_records_.find(arm->getDeviceName());
        if (rec_it != device_records_.end())
            rec_it->second.active = true;
    }

    state_ = SysState::ENGAGED;
    cmd_requested_.store(SysState::ENGAGED);
    if (cmd_channel_) cmd_channel_->setState(SysState::ENGAGED);
    requestAllDevices(SysState::ENGAGED);

    markEpisodeStart();

    for (auto& arm : arm_instances)
        arm->writeEpisodeConfig(current_episode_cfg_.pick_x, current_episode_cfg_.pick_y, current_episode_cfg_.pick_z,
                                current_episode_cfg_.place_x, current_episode_cfg_.place_y, current_episode_cfg_.place_z,
                                current_episode_cfg_.mode);

    reset_all_pending_.store(false);

    std::cout << "[AVATAR-INFO]: Global reset complete, awaiting engagement." << std::endl;
}

void Avatar::sendDeviceEvent(const std::string& device, const std::string& event) {
    msgpack::sbuffer buf;
    msgpack::pack(buf, std::map<std::string, std::string>{{"device", device}, {"event", event}});
    cmd_channel_->send("device_event", buf, true);
}

void Avatar::updateStateMachine(SysState cmd_state){
    if (reset_all_pending_.load()) return;

    if(cmd_state == SysState::STOP){
        state_ = SysState::STOP;
    }
    switch (state_) {
        case SysState::IDLE:
            if(cmd_state == SysState::HOMING){
                requestAllDevices(SysState::HOMING);
                state_ = SysState::HOMING;
                std::cout << "[AVATAR-INFO]: Homing." << std::endl;
            }
            break;

        case SysState::HOMING:
            if(cmd_state == SysState::IDLE){
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
            }
            else if(allInState(SysState::AWAITING)){
                state_ = SysState::AWAITING;
                std::cout << "[AVATAR-INFO]: Awaiting engagement." << std::endl;
            }
            break;

        case SysState::AWAITING:
            if(cmd_state == SysState::IDLE){
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
            }
            else if(cmd_state == SysState::ENGAGED && allInState(SysState::AWAITING)){
                requestAllDevices(SysState::ENGAGED);
                state_ = SysState::ENGAGED;
                std::cout << "[AVATAR-INFO]: Engage system." << std::endl;
            }
            break;

        case SysState::ENGAGED:
            if(cmd_state == SysState::IDLE){
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
                markEpisodeEnd("operator_idle");
                std::cout << "[AVATAR-INFO]: Switch engage -> idle." << std::endl;
            }
            else if(cmd_state == SysState::PAUSED){
                requestAllDevices(SysState::PAUSED);
                state_ = SysState::PAUSED;
                markEpisodeEnd("operator_pause");
                std::cout << "[AVATAR-INFO]: Switch engage -> pause." << std::endl;
            }
            break;
            
        case SysState::PAUSED:
            if(cmd_state == SysState::IDLE){
                requestAllDevices(SysState::IDLE);
                state_ = SysState::IDLE;
                std::cout << "[AVATAR-INFO]: Switch pause -> idle." << std::endl;
            }
            else if(cmd_state == SysState::ENGAGED && allInState(SysState::PAUSED)){
                requestAllDevices(SysState::ENGAGED);
                state_ = SysState::ENGAGED;
                std::cout << "[AVATAR-INFO]: Switch pause -> engage." << std::endl;
            }
            break;

        default:
            break;
    }
}

bool Avatar::allInState(SysState state) {
    for (auto& arm  : arm_instances) {
        auto it = device_records_.find(arm->getDeviceName());
        if (it != device_records_.end() && !it->second.active) continue;
        if (arm->getState() != state) return false;
    }
    for (auto& head : head_instances) {
        auto it = device_records_.find(head->getDeviceName());
        if (it != device_records_.end() && !it->second.active) continue;
        if (head->getState() != state) return false;
    }
    return true;
}

bool Avatar::anyoneInState(SysState state) {
    for (auto& arm  : arm_instances) {
        auto it = device_records_.find(arm->getDeviceName());
        if (it != device_records_.end() && !it->second.active) continue;
        if (arm->getState() == state) return true;
    }
    for (auto& head : head_instances) {
        auto it = device_records_.find(head->getDeviceName());
        if (it != device_records_.end() && !it->second.active) continue;
        if (head->getState() == state) return true;
    }
    return false;
}

void Avatar::requestAllDevices(SysState state) {
    for (auto& arm  : arm_instances) {
        auto it = device_records_.find(arm->getDeviceName());
        if (it != device_records_.end() && !it->second.active) continue;
        arm->requestState(state);
    }
    for (auto& head : head_instances) {
        auto it = device_records_.find(head->getDeviceName());
        if (it != device_records_.end() && !it->second.active) continue;
        head->requestState(state);
    }
}

ArmControl* Avatar::getArm(const std::string& name) {
    for (auto& arm : arm_instances)
        if (arm->getDeviceName() == name) return arm;
    return nullptr;
}

void Avatar::markEpisodeStart() {
    for (auto& arm  : arm_instances) arm->markEpisodeStart();
    for (auto& head : head_instances) head->markEpisodeStart();
}

void Avatar::markEpisodeEnd(const std::string& reason) {
    for (auto& arm  : arm_instances) arm->markEpisodeEnd(reason);
    for (auto& head : head_instances) head->markEpisodeEnd(reason);
}