#pragma once

#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <linux/can.h>
#include <linux/can/raw.h>

/**
 * @brief SocketCAN wrapper class for CAN bus communication
 *
 * Provides a simple interface for sending and receiving CAN frames
 * using Linux SocketCAN. Supports both blocking and non-blocking operations.
 */
class CanSocket {
public:
    /**
     * @brief CAN frame receive callback function type
     * @param frame The received CAN frame
     */
    using ReceiveCallback = std::function<void(const struct can_frame&)>;

private:
    int socket_fd_;
    std::string interface_name_;
    std::atomic<bool> receiving_;
    std::thread receive_thread_;
    ReceiveCallback receive_callback_;
    mutable std::mutex socket_mutex_;

public:
    /**
     * @brief Constructor
     * @param interface_name CAN interface name (e.g., "can0", "vcan0")
     */
    explicit CanSocket(const std::string& interface_name = "vcan0");
    
    /**
     * @brief Destructor - ensures socket is closed
     */
    ~CanSocket();

    /**
     * @brief Open CAN socket and bind to interface
     * @return true if successful, false otherwise
     */
    bool open();
    
    /**
     * @brief Close CAN socket
     */
    void close();
    
    /**
     * @brief Check if socket is open
     * @return true if socket is open and ready
     */
    bool isOpen() const;

    /**
     * @brief Send a CAN frame
     * @param frame CAN frame to send
     * @return true if sent successfully, false otherwise
     */
    bool sendFrame(const struct can_frame& frame);

    /**
     * @brief Start receiving CAN frames in background thread
     * @param callback Function to call when frame is received
     * @return true if started successfully, false otherwise
     */
    bool startReceiving(ReceiveCallback callback);
    
    /**
     * @brief Stop receiving CAN frames
     */
    void stopReceiving();
    
    /**
     * @brief Check if currently receiving
     * @return true if receive thread is running
     */
    bool isReceiving() const;

    /**
     * @brief Receive a single CAN frame (blocking)
     * @param frame Reference to store received frame
     * @param timeout_ms Timeout in milliseconds (0 = no timeout)
     * @return true if frame received, false on timeout or error
     */
    bool receiveFrame(struct can_frame& frame, int timeout_ms = 0);

    /**
     * @brief Get the CAN interface name
     * @return Interface name string
     */
    const std::string& getInterfaceName() const;

    /**
     * @brief Set CAN filters
     * @param filters Array of CAN filters
     * @param filter_count Number of filters
     * @return true if filters set successfully, false otherwise
     */
    bool setFilters(const struct can_filter* filters, size_t filter_count);

private:
    /**
     * @brief Receive thread function
     */
    void receiveLoop();
    
    /**
     * @brief Convert errno to string for error reporting
     * @return Error string
     */
    std::string getLastError() const;
};