#include "transmission.hpp"

Transmission::Transmission(const TransmissionConfig& config)
    : config_(config)
    , send_addr_(config.remote_ip, config.send_port)
    , recv_addr_("0.0.0.0", config.receive_port)
{
    socket_.bind(recv_addr_, true);
    socket_.setReceiveTimeout(Poco::Timespan(0, 1000));
    socket_.setSendTimeout(Poco::Timespan(0, 1000));
    last_recv_time_ = std::chrono::steady_clock::now();
}

Transmission::~Transmission() {
    stop();
    socket_.close();
}

void Transmission::start() {
    bRunning_ = true;
    thread_ = std::thread(&Transmission::run, this);
}

void Transmission::stop() {
    bRunning_ = false;
    if (thread_.joinable()) thread_.join();
}

uint64_t Transmission::timestamp() const {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}

void Transmission::run() {
    auto period = std::chrono::microseconds(1000000 / config_.frequency);
    auto next   = std::chrono::steady_clock::now();

    while (bRunning_) {
        receive();
        send();
        next += period;
        std::this_thread::sleep_until(next);
    }
}

void Transmission::send() {
    std::lock_guard<std::mutex> lock(send_mtx_);
    try {
        switch (config_.role) {
            case TransmissionRole::ARM:
                arm_state_out_.timestamp_ns = timestamp();
                socket_.sendTo(&arm_state_out_, sizeof(arm_state_out_), send_addr_);
                break;
            case TransmissionRole::HEAD:
                head_state_out_.timestamp_ns = timestamp();
                socket_.sendTo(&head_state_out_, sizeof(head_state_out_), send_addr_);
                break;
            case TransmissionRole::AVATAR:
                avatar_state_out_.timestamp_ns = timestamp();
                socket_.sendTo(&avatar_state_out_, sizeof(avatar_state_out_), send_addr_);
                break;
        }
    } catch (const Poco::Exception&) {}
}

void Transmission::receive() {
    Poco::Net::SocketAddress sender;
    try {
        switch (config_.role) {
            case TransmissionRole::ARM: {
                ArmCommandMsg msg{};
                int n = socket_.receiveFrom(&msg, sizeof(msg), sender);
                if (n == sizeof(ArmCommandMsg)) {
                    std::lock_guard<std::mutex> lock(recv_mtx_);
                    arm_cmd_in_     = msg;
                    bHasNewCommand_ = true;
                    last_recv_time_ = std::chrono::steady_clock::now();
                }
                break;
            }
            case TransmissionRole::HEAD: {
                HeadCommandMsg msg{};
                int n = socket_.receiveFrom(&msg, sizeof(msg), sender);
                if (n == sizeof(HeadCommandMsg)) {
                    std::lock_guard<std::mutex> lock(recv_mtx_);
                    head_cmd_in_    = msg;
                    bHasNewCommand_ = true;
                    last_recv_time_ = std::chrono::steady_clock::now();
                }
                break;
            }
            case TransmissionRole::AVATAR: {
                AvatarCommandMsg msg{};
                int n = socket_.receiveFrom(&msg, sizeof(msg), sender);
                if (n == sizeof(AvatarCommandMsg)) {
                    std::lock_guard<std::mutex> lock(recv_mtx_);
                    avatar_cmd_in_  = msg;
                    bHasNewCommand_ = true;
                    last_recv_time_ = std::chrono::steady_clock::now();
                }
                break;
            }
        }
    } catch (const Poco::Exception&) {}
}

void Transmission::sendArmState(const ArmStateMsg& msg) {
    std::lock_guard<std::mutex> lock(send_mtx_);
    arm_state_out_ = msg;
}

ArmCommandMsg Transmission::getArmCommand() {
    std::lock_guard<std::mutex> lock(recv_mtx_);
    bHasNewCommand_ = false;
    return arm_cmd_in_;
}

void Transmission::sendHeadState(const HeadStateMsg& msg) {
    std::lock_guard<std::mutex> lock(send_mtx_);
    head_state_out_ = msg;
}

HeadCommandMsg Transmission::getHeadCommand() {
    std::lock_guard<std::mutex> lock(recv_mtx_);
    bHasNewCommand_ = false;
    return head_cmd_in_;
}

void Transmission::sendAvatarState(const AvatarStateMsg& msg) {
    std::lock_guard<std::mutex> lock(send_mtx_);
    avatar_state_out_ = msg;
}

AvatarCommandMsg Transmission::getAvatarCommand() {
    std::lock_guard<std::mutex> lock(recv_mtx_);
    bHasNewCommand_ = false;
    return avatar_cmd_in_;
}

bool Transmission::isAlive() const {
    auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_recv_time_).count();
    return delta < 500;
}