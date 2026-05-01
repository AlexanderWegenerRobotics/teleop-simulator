#include <yaml-cpp/yaml.h>
#include <atomic>
#include <unordered_map>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "arm_control.hpp"
#include "head_control.hpp"
#include "network/udp_reliable.hpp"
#include "sim_env/simulation.hpp"
#include "self_collision_protection.hpp"

class Avatar{

public:
    Avatar(const YAML::Node& config);
    ~Avatar();

    void start();
    void stop();
    bool isRunning() const {return bRunning;}
    SysState getState() const {return state_;}

    std::shared_ptr<Simulation> getSim() const { return sim_; }

private:
    void updateStateMachine(SysState cmd_state);
    void processRecoveryNotifications();
    bool allInState(SysState state);
    bool anyoneInState(SysState state);
    void requestAllDevices(SysState state);
    void sendDeviceEvent(const std::string& device, const std::string& event);
    ArmControl* getArm(const std::string& name);
    void markEpisodeStart();
    void markEpisodeEnd(const std::string& reason);
    void processResetAllCompletion();

private:
    std::vector<ArmControl*> arm_instances;
	std::vector<HeadControl*> head_instances;
    std::unique_ptr<UdpReliable> cmd_channel_;
    std::atomic<SysState> cmd_requested_{SysState::IDLE};

    std::shared_ptr<Simulation> sim_ = nullptr;
    std::atomic<bool> bRunning;
    std::atomic<SysState> state_;
    std::shared_ptr<DeviceRegistry> device_registry_;
    std::unordered_map<std::string, DeviceRecord> device_records_;
    std::atomic<bool> reset_all_pending_{false};

    std::string      session_id_;
    std::string      log_base_dir_;
    int              episode_sock_  = -1;
    int              episode_index_ = 0;

    struct EpisodeConfig {
        double pick_x, pick_y, pick_z;
        double place_x, place_y, place_z;
        int    mode;
    };
    EpisodeConfig current_episode_cfg_{};

    EpisodeConfig requestEpisodeConfig();
    void          startNewEpisodeFolder();
    void applyEpisodeConfig(const EpisodeConfig& cfg);
};