#include "Servo.h"
#include "CanBoard.h"

// Private constructor used by builder
Servo::Servo(double max_velocity_rpm, int max_control_signal, double motor_time_constant,
             int bit_resolution, bool direction_inverted, uint32_t can_id, 
             const std::string& can_interface, bool can_enabled)
    : motor_(max_velocity_rpm, max_control_signal, motor_time_constant),
      encoder_(bit_resolution, direction_inverted),
      can_board_(nullptr) {
    
    if (can_enabled) {
        can_board_ = std::make_unique<CanBoard>(*this, can_id, can_interface);
        can_board_->start(); // Automatically start CAN board during construction
    }
}

// Default constructor
Servo::Servo() 
    : motor_(160.0, 1000, 0.3),
      encoder_(18, false),
      can_board_(nullptr) {
    // Create default CanBoard with default CAN ID and start it
    can_board_ = std::make_unique<CanBoard>(*this, 0x10, "vcan0");
    can_board_->start();
}

// Move constructor
Servo::Servo(Servo&& other) noexcept
    : motor_(std::move(other.motor_)),
      encoder_(std::move(other.encoder_)),
      can_board_(std::move(other.can_board_)) {
}

// Move assignment operator
Servo& Servo::operator=(Servo&& other) noexcept {
    if (this != &other) {
        motor_ = std::move(other.motor_);
        encoder_ = std::move(other.encoder_);
        can_board_ = std::move(other.can_board_);
    }
    return *this;
}

// Destructor - automatically stops CAN board
Servo::~Servo() {
    // unique_ptr destructor will automatically call CanBoard destructor which stops it
}

void Servo::start() {
    if (can_board_ && !can_board_->isRunning()) {
        can_board_->start();
    }
}

void Servo::stop() {
    if (can_board_) {
        can_board_->stop();
    }
}

uint32_t Servo::getCanId() const {
    if (can_board_) {
        return can_board_->getCanId();
    }
    return 0; // Return 0 if no CAN board
}