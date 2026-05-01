#include <yaml-cpp/yaml.h>
#include <atomic>
#include <unordered_map>

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
};