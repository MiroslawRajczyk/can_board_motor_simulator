#include "Servo.h"
#include "CanBoard.h"

Servo::Servo(double max_velocity_rpm, int max_control_signal, double motor_time_constant,
             int bit_resolution, bool direction_inverted, bool enable_can, uint32_t can_id, 
             const std::string& can_interface)
    : motor_(std::make_shared<Motor>(Motor::builder()
                                     .maxVelocityRPM(max_velocity_rpm)
                                     .maxControlSignal(max_control_signal)
                                     .timeConstant(motor_time_constant))),
      encoder_(std::make_shared<Encoder>(Encoder::builder()
                                         .bitResolution(bit_resolution)
                                         .directionInverted(direction_inverted))) {
    
    // Create CanBoard if CAN is enabled
    if (enable_can) {
        can_board_ = std::make_unique<CanBoard>(*this, can_id, can_interface);
    }
}

Servo::~Servo() {
    // Default destructor implementation - needed for unique_ptr<CanBoard>
    // The CanBoard destructor will be called automatically when the unique_ptr is destroyed
}

Servo::Servo(Servo&& other) noexcept 
    : motor_(std::move(other.motor_)),
      encoder_(std::move(other.encoder_)) {
    
    // If the other servo has a CanBoard, we need to create a new one
    // because CanBoard holds a reference to the Servo object
    if (other.can_board_) {
        uint32_t can_id = other.can_board_->getCanId();
        // Stop the old CanBoard first
        other.can_board_->stop();
        
        // Create new CanBoard for this servo
        // We need to extract the CAN interface from the old CanBoard
        can_board_ = std::make_unique<CanBoard>(*this, can_id, "vcan0");
        
        // Clear the other's CanBoard
        other.can_board_.reset();
    }
}

Servo& Servo::operator=(Servo&& other) noexcept {
    if (this != &other) {
        motor_ = std::move(other.motor_);
        encoder_ = std::move(other.encoder_);
        
        // Handle CanBoard properly due to reference issue
        if (other.can_board_) {
            uint32_t can_id = other.can_board_->getCanId();
            // Stop the old CanBoard first
            other.can_board_->stop();
            
            // Create new CanBoard for this servo
            can_board_ = std::make_unique<CanBoard>(*this, can_id, "vcan0");
            
            // Clear the other's CanBoard
            other.can_board_.reset();
        } else {
            can_board_.reset();
        }
    }
    return *this;
}

void Servo::startCAN() {
    if (can_board_) {
        can_board_->start();
    }
}

void Servo::stopCAN() {
    if (can_board_) {
        can_board_->stop();
    }
}

bool Servo::isCANRunning() const {
    return can_board_ && can_board_->isRunning();
}