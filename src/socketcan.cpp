#include "syntron_driver/socketcan.hpp"
#include <iostream>

// #include <iostream>
// #include <functional>
// #include <cstring>
// #include <thread>
// #include <atomic>
// #include <net/if.h>
// #include <sys/ioctl.h>
// #include <sys/socket.h>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <unistd.h>

SocketCAN::SocketCAN(const std::string& interface_name) 
    : interface_name_(interface_name), is_connected_(false), running_(false) 
{

}

SocketCAN::~SocketCAN() {
    disconnect();
}

bool SocketCAN::connect() {
    if (is_connected_) return false;

    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        std::cerr << "Error while opening socket\n";
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error while getting interface index\n";
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error in socket bind\n";
        return false;
    }

    is_connected_ = true;
    running_ = true;
    recv_thread_ = std::thread(&SocketCAN::receiveLoop, this);
    std::cout << "Connected to " << interface_name_.c_str() << "\n";
    return true;
}

void SocketCAN::disconnect() {
    if (is_connected_) {
        running_ = false;
        std::cout << "Disconnecting from " << interface_name_.c_str() << "\n";
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        close(sock_);
        is_connected_ = false;
        std::cout << "Disconnected\n";

    }
}

bool SocketCAN::transmit(const struct can_frame& frame) {
    if (!is_connected_) return false;

    int bytes_sent = write(sock_, &frame, sizeof(struct can_frame));
    return bytes_sent == sizeof(struct can_frame);
}

void SocketCAN::receiveLoop() {
    struct can_frame frame;
            struct timeval timeout;
        timeout.tv_sec = 1;  // Set timeout to 1 second
        timeout.tv_usec = 0;
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(sock_, &read_fds);
    while (running_) {
        int select_result = select(sock_ + 1, &read_fds, nullptr, nullptr, &timeout);
        if (select_result == -1) {
            std::cerr << "Error using canbus\n";
            return;
        } 
        
        if (select_result == 0) {
            // Timeout occurred
            continue;
        }
        int nbytes = read(sock_, &frame, sizeof(struct can_frame));
        if (nbytes > 0 && callback_) {
            callback_(frame);
        }
    }
}
