#pragma once

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstring>
#include <type_traits>

#include "udp_transport.hpp"
#include "common.hpp"

struct UdpStreamConfig {
    TransportConfig transport;
    int             send_rate_hz  = 100;
    int             recv_enabled  = true;
};

template<typename TRecv, typename TSend>
class UdpStream {
    static_assert(std::is_trivially_copyable_v<TRecv>, "TRecv must be trivially copyable");
    static_assert(std::is_trivially_copyable_v<TSend>, "TSend must be trivially copyable");
    static_assert(offsetof(TRecv, header) == 0, "TRecv must start with MsgHeader");
    static_assert(offsetof(TSend, header) == 0, "TSend must start with MsgHeader");

public:
    explicit UdpStream(const UdpStreamConfig& config)
        : config_(config)
        , transport_(config.transport)
    {
        std::memset(&recv_msg_, 0, sizeof(TRecv));
        std::memset(&send_msg_, 0, sizeof(TSend));
        last_recv_time_ = std::chrono::steady_clock::now();
    }

    ~UdpStream() { stop(); }

    UdpStream(const UdpStream&) = delete;
    UdpStream& operator=(const UdpStream&) = delete;

    void start() {
        running_ = true;
        thread_ = std::thread(&UdpStream::run, this);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    void setSendData(const TSend& msg) {
        std::lock_guard<std::mutex> lock(send_mtx_);
        send_msg_ = msg;
    }

    TRecv getRecvData() {
        std::lock_guard<std::mutex> lock(recv_mtx_);
        has_new_ = false;
        return recv_msg_;
    }

    void setState(SysState state, FaultCode fault = FaultCode::NONE) {
        std::lock_guard<std::mutex> lock(send_mtx_);
        sticky_state_ = state;
        sticky_fault_ = fault;
    }

    bool hasNew() const { return has_new_; }
    bool isAlive() const {
        auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_recv_time_).count();
        return delta < ALIVE_TIMEOUT_MS;
    }

    uint32_t lastRecvSequence() const { return last_recv_seq_; }
    uint32_t droppedPackets() const { return dropped_count_; }

private:
    void run() {
        auto period = std::chrono::microseconds(1000000 / config_.send_rate_hz);
        auto next = std::chrono::steady_clock::now();

        while (running_) {
            receive();
            doSend();
            next += period;
            std::this_thread::sleep_until(next);
        }
    }

    void doSend() {
        std::lock_guard<std::mutex> lock(send_mtx_);
        send_msg_.header.sequence = ++send_seq_;
        send_msg_.header.timestamp_ns = timestamp_ns();
        send_msg_.header.state = sticky_state_;
        send_msg_.header.fault_code = sticky_fault_;
        transport_.sendTo(&send_msg_, sizeof(TSend));
    }

    void receive() {
        Poco::Net::SocketAddress sender;
        uint8_t buffer[sizeof(TRecv) + 64];

        while (true) {
            int n = transport_.receiveFrom(buffer, sizeof(buffer), sender);
            if (n <= 0) break;

            if (n == sizeof(TRecv)) {
                TRecv msg;
                std::memcpy(&msg, buffer, sizeof(TRecv));

                uint32_t seq = msg.header.sequence;
                if (seq > last_recv_seq_ + 1 && last_recv_seq_ > 0) {
                    dropped_count_ += (seq - last_recv_seq_ - 1);
                }

                if (seq > last_recv_seq_ || last_recv_seq_ == 0) {
                    std::lock_guard<std::mutex> lock(recv_mtx_);
                    recv_msg_ = msg;
                    has_new_ = true;
                    last_recv_seq_ = seq;
                    last_recv_time_ = std::chrono::steady_clock::now();
                }
            }
        }
    }

    UdpStreamConfig config_;
    UdpTransport      transport_;

    std::thread       thread_;
    std::atomic<bool> running_{false};

    std::mutex        send_mtx_;
    TSend             send_msg_;
    uint32_t          send_seq_ = 0;
    SysState          sticky_state_ = SysState::OFFLINE;
    FaultCode         sticky_fault_ = FaultCode::NONE;

    std::mutex        recv_mtx_;
    TRecv             recv_msg_;
    std::atomic<bool> has_new_{false};

    std::chrono::steady_clock::time_point last_recv_time_;
    std::atomic<uint32_t> last_recv_seq_{0};
    std::atomic<uint32_t> dropped_count_{0};

    static constexpr int ALIVE_TIMEOUT_MS = 500;
};