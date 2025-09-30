#pragma once

#include "CanSocket.h"
#include <atomic>
#include <thread>
#include <chrono>
#include <functional>
#include <vector>
#include <mutex>
#include <memory>

// Forward declaration to avoid circular dependency
class Servo;

/**
 * @brief CAN board hardware simulation for servo systems
 *
 * Simulates a CAN-based microcontroller board that manages servo control
 * including periodic encoder reading, control signal updates, CAN communication, and other timed operations.
 * Each board has a unique CAN ID for all communication.
 */
class CanBoard {
public:
    /**
     * @brief Timer configuration for periodic operations
     */
    struct TimerConfig {
        std::string name;
        std::chrono::microseconds period;
        std::function<void()> callback;
        bool enabled = true;
    };

private:
    Servo& servo_;
    std::unique_ptr<CanSocket> can_socket_;
    uint32_t can_id_;
    std::atomic<bool> running_;
    std::vector<std::thread> timerThreads_;
    std::vector<TimerConfig> timers_;
    mutable std::mutex dataMutex_;

    // Encoder data cache (simulates hardware registers)
    std::atomic<long> cachedEncoderSteps_;
    std::atomic<double> cachedEncoderRadians_;
    std::atomic<int> currentControlSignal_;

    // Timer frequencies (in Hz)
    static constexpr double ENCODER_READ_FREQUENCY = 300.0;
    static constexpr double CONTROL_UPDATE_FREQUENCY = 300.0;
    static constexpr double CAN_TRANSMIT_FREQUENCY = 100.0;

public:
    /**
     * @brief Constructor
     * @param servo Reference to the servo to control
     * @param can_id Unique CAN ID for this board (used for all CAN communication)
     * @param can_interface CAN interface name (e.g., "can0", "vcan0")
     */
    explicit CanBoard(Servo& servo, uint32_t can_id, const std::string& can_interface = "vcan0");

    /**
     * @brief Destructor - ensures all timers are stopped
     */
    ~CanBoard();

    /**
     * @brief Start the board and all timers
     */
    void start();

    /**
     * @brief Stop the board and all timers
     */
    void stop();

    /**
     * @brief Check if board is running
     */
    bool isRunning() const;

    /**
     * @brief Set control signal (thread-safe)
     * @param signal Control signal value
     */
    void setControlSignal(int signal);

    /**
     * @brief Get cached encoder position in steps (from hardware registers)
     * @return Encoder position in steps
     */
    long getEncoderSteps() const;

    /**
     * @brief Get cached encoder position in radians (from hardware registers)
     * @return Encoder position in radians
     */
    double getEncoderRadians() const;

    /**
     * @brief Get current control signal
     * @return Current control signal value
     */
    int getControlSignal() const;

    /**
     * @brief Get CAN ID
     * @return CAN ID used for all communication
     */
    uint32_t getCanId() const;

    /**
     * @brief Get CAN socket reference for direct access
     * @return Reference to the CanSocket instance
     */
    CanSocket& getCanSocket();

    /**
     * @brief Enable/disable a timer by name
     * @param name Timer name
     * @param enabled Enable state
     */
    void setTimerEnabled(const std::string& name, bool enabled);

private:
    /**
     * @brief Initialize default timers
     */
    void initializeTimers();

    /**
     * @brief Timer thread function
     * @param config Timer configuration
     */
    void timerLoop(const TimerConfig& config);

    /**
     * @brief Encoder reading timer callback
     */
    void encoderReadTimer();

    /**
     * @brief Control update timer callback
     */
    void controlUpdateTimer();


    /**
     * @brief CAN transmission timer callback
     */
    void canTransmitTimer();

    /**
     * @brief CAN frame receive callback
     * @param frame Received CAN frame
     */
    void onCanFrameReceived(const struct can_frame& frame);
};