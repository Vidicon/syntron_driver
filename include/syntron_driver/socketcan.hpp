#pragma once

#include <iostream>
#include <functional>
#include <cstring>
#include <thread>
#include <atomic>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

class SocketCAN {
public:
    using CANCallback = std::function<void(const struct can_frame&)>;

    SocketCAN(const std::string& interface_name);
    ~SocketCAN();

    bool connect();
    void disconnect();

    bool transmit(const struct can_frame& frame);
    void registerCallback(CANCallback callback) { callback_ = callback; }

    bool isConnected() const { return is_connected_; }

private:
    void receiveLoop();

    std::string interface_name_;
    int sock_;
    std::atomic<bool> is_connected_;
    std::atomic<bool> running_;
    std::thread recv_thread_;
    CANCallback callback_;
};