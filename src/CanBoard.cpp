#include "CanBoard.h"
#include <iostream>
#include <iomanip>
#include <cstring>

CanBoard::CanBoard(Servo& servo, uint32_t can_id, const std::string& can_interface)
    : servo_(servo), can_socket_(std::make_unique<CanSocket>(can_interface)),
    can_id_(can_id), running_(false), cachedEncoderSteps_(0),
    cachedEncoderRadians_(0.0), currentControlSignal_(1) {
    initializeTimers();
}

CanBoard::~CanBoard() {
    stop();
}

void CanBoard::start() {
    if (running_) {
        return;
    }

    // Open CAN socket
    if (!can_socket_->open()) {
        std::cerr << "CanBoard: Failed to open CAN socket, continuing without CAN communication" << std::endl;
    } else {
        // Set up CAN filter to only receive frames with CanBoard's CAN ID
        struct can_filter filter;
        filter.can_id = can_id_;
        filter.can_mask = CAN_SFF_MASK; // Standard frame format mask

        if (!can_socket_->setFilters(&filter, 1)) {
            std::cerr << "CanBoard: Failed to set CAN filter" << std::endl;
        }

        // Start CAN receiving
        can_socket_->startReceiving([this](const struct can_frame& frame) {
            onCanFrameReceived(frame);
        });
    }

    running_ = true;

    // Start all enabled timers
    for (const auto& timer : timers_) {
        if (timer.enabled) {
            timerThreads_.emplace_back(&CanBoard::timerLoop, this, timer);
        }
    }
}

void CanBoard::stop() {
    if (!running_) {
        return;
    }

    running_ = false;

    // Stop CAN communication
    can_socket_->close();

    // Wait for all timer threads to finish
    for (auto& thread : timerThreads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    timerThreads_.clear();
    std::cout << "CanBoard[0x" << std::hex << can_id_ << std::dec << "]: Stopped all timers" << std::endl;
}

bool CanBoard::isRunning() const {
    return running_;
}

void CanBoard::setControlSignal(int signal) {
    currentControlSignal_ = signal;
}

long CanBoard::getEncoderSteps() const {
    return cachedEncoderSteps_;
}

int CanBoard::getControlSignal() const {
    return currentControlSignal_;
}

uint32_t CanBoard::getCanId() const {
    return can_id_;
}

CanSocket& CanBoard::getCanSocket() {
    return *can_socket_;
}

void CanBoard::setTimerEnabled(const std::string& name, bool enabled) {
    std::lock_guard<std::mutex> lock(dataMutex_);
    for (auto& timer : timers_) {
        if (timer.name == name) {
            timer.enabled = enabled;
            break;
        }
    }
}

void CanBoard::initializeTimers() {
    // Encoder reading timer
    timers_.push_back({
        "encoder_read",
        std::chrono::microseconds(static_cast<long>(1000000.0 / ENCODER_READ_FREQUENCY)),
        [this]() { encoderReadTimer(); },
        true
    });

    // Control signal update timer
    timers_.push_back({
        "control_update",
        std::chrono::microseconds(static_cast<long>(1000000.0 / CONTROL_UPDATE_FREQUENCY)),
        [this]() { controlUpdateTimer(); },
        true
    });

    // CAN transmission timer
    timers_.push_back({
        "can_transmit",
        std::chrono::microseconds(static_cast<long>(1000000.0 / CAN_TRANSMIT_FREQUENCY)),
        [this]() { canTransmitTimer(); },
        true
    });
}

void CanBoard::timerLoop(const TimerConfig& config) {
    auto next_execution = std::chrono::steady_clock::now();

    while (running_) {
        config.callback();
        next_execution += config.period;
        std::this_thread::sleep_until(next_execution);
    }
}

void CanBoard::encoderReadTimer() {
    cachedEncoderSteps_ = servo_.getEncoder().getPositionSteps();
}

void CanBoard::controlUpdateTimer() {
    if (currentControlSignal_ == 1 || currentControlSignal_ == -1) {
        servo_.setControlSignal(0);
        return;
    }

    servo_.setControlSignal(currentControlSignal_);
}

void CanBoard::canTransmitTimer() {
    if (!can_socket_->isOpen()) {
        std::cout << "CAN socket is not open" << std::endl;
        return; // CAN not available
    }

    // Create CAN frame using Linux can_frame structure
    struct can_frame frame;
    frame.can_id = can_id_;
    frame.can_dlc = 6;

    // Message type
    frame.data[0] = 0x13;

    // Encoder position (16-bit unsigned)
    // Convert encoder steps to a 16-bit value (scale down if necessary)
    uint32_t encoder_steps = static_cast<uint32_t>(std::abs(cachedEncoderSteps_.load()));
    uint16_t encoder_16bit = static_cast<uint16_t>(encoder_steps & 0xFFFF);
    frame.data[1] = (encoder_16bit >> 8) & 0xFF;  // ENCODER_H
    frame.data[2] = encoder_16bit & 0xFF;         // ENCODER_L

    // Speed in RPM * 100 (16-bit signed)
    double velocity_rad_s = servo_.getAngularVelocity(); // TODO: replace with calculated speed from encoder readings
    double velocity_rpm = velocity_rad_s * (60.0 / (2.0 * M_PI));
    int16_t speed_scaled = static_cast<int16_t>(velocity_rpm * 100.0);
    frame.data[3] = (speed_scaled >> 8) & 0xFF;   // SPEED_H
    frame.data[4] = speed_scaled & 0xFF;          // SPEED_L

    // Effort (8-bit signed, -100 to +100)
    frame.data[5] = static_cast<uint8_t>(currentControlSignal_.load());

    can_socket_->sendFrame(frame);
}

void CanBoard::onCanFrameReceived(const struct can_frame& frame) {
    if (frame.can_dlc < 1) {
        return;
    }

    uint8_t message_type = frame.data[0];

    switch (message_type) {
        case 0x10: // Effort command
            if (frame.can_dlc == 2) {
                int8_t new_control = static_cast<int8_t>(frame.data[1]);
                if (new_control == 1 || new_control == -1) { // Stop without position hold
                    setControlSignal(1);

                } else if (new_control == 0) { // Stop with position hold
                    setControlSignal(0); // TODO: replace with position hold logic
                } else {
                    setControlSignal(new_control);
                }
            }
            break;

        default:
            // Unknown message type, ignore
            std::cout << "CanBoard[0x" << std::hex << can_id_ << std::dec
                    << "]: Unknown message type 0x" << std::hex << static_cast<int>(message_type)
                    << std::dec << std::endl;
            break;
    }
}
