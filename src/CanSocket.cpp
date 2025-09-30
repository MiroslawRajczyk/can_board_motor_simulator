#include "CanSocket.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <poll.h>
#include <errno.h>

CanSocket::CanSocket(const std::string& interface_name)
    : socket_fd_(-1), interface_name_(interface_name), receiving_(false) {
}

CanSocket::~CanSocket() {
    close();
}

bool CanSocket::open() {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    if (socket_fd_ >= 0) {
        return true; // Already open
    }

    // Create socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "CanSocket: Failed to create socket: " << getLastError() << std::endl;
        return false;
    }

    // Get interface index
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "CanSocket: Interface " << interface_name_ << " not found: " << getLastError() << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Bind socket to interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "CanSocket: Failed to bind to interface " << interface_name_ << ": " << getLastError() << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    return true;
}

void CanSocket::close() {
    stopReceiving();

    std::lock_guard<std::mutex> lock(socket_mutex_);

    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CanSocket::isOpen() const {
    return socket_fd_ >= 0;
}

bool CanSocket::sendFrame(const struct can_frame& frame) {
    int fd;
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (socket_fd_ < 0) {
            std::cerr << "CanSocket: Socket not open" << std::endl;
            return false;
        }
        fd = socket_fd_;
    }

    ssize_t bytes_sent = write(fd, &frame, sizeof(frame));
    if (bytes_sent != sizeof(frame)) {
        std::cerr << "CanSocket: Failed to send frame: " << getLastError() << std::endl;
        return false;
    }

    return true;
}

bool CanSocket::startReceiving(ReceiveCallback callback) {
    if (receiving_) {
        return true;
    }

    if (!isOpen()) {
        std::cerr << "CanSocket: Cannot start receiving, socket not open" << std::endl;
        return false;
    }

    receive_callback_ = callback;
    receiving_ = true;
    receive_thread_ = std::thread(&CanSocket::receiveLoop, this);

    return true;
}

void CanSocket::stopReceiving() {
    if (!receiving_) {
        return;
    }

    receiving_ = false;

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
}

bool CanSocket::isReceiving() const {
    return receiving_;
}

bool CanSocket::receiveFrame(struct can_frame& frame, int timeout_ms) {
    int fd;
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (socket_fd_ < 0) {
            return false;
        }
        fd = socket_fd_;
    }

    if (timeout_ms > 0) {
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;

        int poll_result = poll(&pfd, 1, timeout_ms);
        if (poll_result <= 0) {
            return false;
        }
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (socket_fd_ != fd || socket_fd_ < 0) {
        return false;
    }

    ssize_t bytes_read = read(socket_fd_, &frame, sizeof(frame));
    return bytes_read == sizeof(frame);
}

const std::string& CanSocket::getInterfaceName() const {
    return interface_name_;
}

bool CanSocket::setFilters(const struct can_filter* filters, size_t filter_count) {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    if (socket_fd_ < 0) {
        std::cerr << "CanSocket: Socket not open" << std::endl;
        return false;
    }

    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters,
                   filter_count * sizeof(struct can_filter)) < 0) {
        std::cerr << "CanSocket: Failed to set filters: " << getLastError() << std::endl;
        return false;
    }

    return true;
}

void CanSocket::receiveLoop() {
    while (receiving_) {
        struct can_frame frame;

        if (receiveFrame(frame, 10)) {
            if (receive_callback_ && receiving_) {
                receive_callback_(frame);
            }
        }
    }
}

std::string CanSocket::getLastError() const {
    return std::strerror(errno);
}