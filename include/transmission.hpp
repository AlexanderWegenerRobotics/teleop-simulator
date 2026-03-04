#pragma once

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <string>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/SocketAddress.h>

#include "common.hpp"

struct TransmissionConfig {
    std::string      remote_ip;
    int              send_port;
    int              receive_port;
    int              frequency;
    TransmissionRole role;
};

class Transmission {
public:
    explicit Transmission(const TransmissionConfig& config);
    ~Transmission();

    void start();
    void stop();

    void           sendArmState(const ArmStateMsg& msg);
    ArmCommandMsg  getArmCommand();

    void           sendHeadState(const HeadStateMsg& msg);
    HeadCommandMsg getHeadCommand();

    void             sendAvatarState(const AvatarStateMsg& msg);
    AvatarCommandMsg getAvatarCommand();

    bool hasNewCommand() const { return bHasNewCommand_; }
    bool isAlive()       const;

private:
    void run();
    void send();
    void receive();
    uint64_t timestamp() const;

private:
    TransmissionConfig        config_;
    Poco::Net::DatagramSocket socket_;
    Poco::Net::SocketAddress  send_addr_;
    Poco::Net::SocketAddress  recv_addr_;

    std::thread               thread_;
    std::atomic<bool>         bRunning_{false};
    std::atomic<bool>         bHasNewCommand_{false};

    std::mutex                recv_mtx_;
    std::mutex                send_mtx_;

    ArmCommandMsg             arm_cmd_in_{};
    ArmStateMsg               arm_state_out_{};

    HeadCommandMsg            head_cmd_in_{};
    HeadStateMsg              head_state_out_{};

    AvatarCommandMsg          avatar_cmd_in_{};
    AvatarStateMsg            avatar_state_out_{};

    std::chrono::steady_clock::time_point last_recv_time_;
};