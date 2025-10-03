#pragma once

#include "Motor.h"
#include "Encoder.h"
#include <memory>
#include <string>

// Forward declaration to avoid circular dependency
class CanBoard;

/**
 * @brief Servo class that combines Motor and Encoder
 *
 * This class provides an interface for a servo system that includes
 * both the motor physics simulation and encoder position tracking.
 * The motor and encoder are automatically synchronized during updates.
 * Can optionally include a CanBoard for CAN communication.
 */
class Servo {
private:
    std::shared_ptr<Motor> motor_;
    std::shared_ptr<Encoder> encoder_;
    std::unique_ptr<CanBoard> can_board_;

public:
    /**
     * @brief Builder class for Servo construction with fluent interface
     *
     * Provides a clear way to configure both Motor and Encoder parameters
     * in a single builder pattern.
     */
    class Builder {
    private:
        // Motor parameters
        double max_velocity_rpm_ = 160.0;
        int max_control_signal_ = 1000;
        double motor_time_constant_ = 0.3;

        // Encoder parameters
        int bit_resolution_ = 18;
        bool direction_inverted_ = false;

        // CAN parameters
        bool enable_can_ = false;
        uint32_t can_id_ = 0x10;
        std::string can_interface_ = "vcan0";

    public:
        /**
         * @brief Set motor maximum velocity in RPM
         */
        Builder& maxVelocityRPM(double rpm) {
            max_velocity_rpm_ = rpm;
            return *this;
        }

        /**
         * @brief Set motor maximum control signal
         */
        Builder& maxControlSignal(int signal) {
            max_control_signal_ = signal;
            return *this;
        }

        /**
         * @brief Set motor time constant in seconds
         */
        Builder& timeConstant(double constant) {
            motor_time_constant_ = constant;
            return *this;
        }

        /**
         * @brief Set encoder bit resolution
         */
        Builder& encoderBitResolution(int bits) {
            bit_resolution_ = bits;
            return *this;
        }

        /**
         * @brief Set encoder direction inversion
         */
        Builder& encoderDirectionInverted(bool inverted) {
            direction_inverted_ = inverted;
            return *this;
        }

        /**
         * @brief Enable CAN communication with specified ID
         */
        Builder& enableCAN(uint32_t can_id, const std::string& can_interface = "vcan0") {
            enable_can_ = true;
            can_id_ = can_id;
            can_interface_ = can_interface;
            return *this;
        }

        /**
         * @brief Set CAN ID (automatically enables CAN)
         */
        Builder& canId(uint32_t can_id) {
            enable_can_ = true;
            can_id_ = can_id;
            return *this;
        }

        /**
         * @brief Set CAN interface
         */
        Builder& canInterface(const std::string& can_interface) {
            can_interface_ = can_interface;
            return *this;
        }

        /**
         * @brief Build the Servo
         */
        Servo build() {
            return Servo(max_velocity_rpm_, max_control_signal_, motor_time_constant_,
                         bit_resolution_, direction_inverted_, enable_can_, can_id_, can_interface_);
        }

        /**
         * @brief Automatic conversion to Servo
         */
        operator Servo() {
            return build();
        }
    };

private:
    Servo(double max_velocity_rpm, int max_control_signal, double motor_time_constant,
          int bit_resolution, bool direction_inverted, bool enable_can, uint32_t can_id,
          const std::string& can_interface);

public:
    /**
     * @brief Default constructor with reasonable defaults
     */
    Servo() : Servo(160.0, 1000, 0.3, 18, false, false, 0x10, "vcan0") {}

    /**
     * @brief Destructor
     */
    ~Servo();

    // Copy constructor and assignment operator - deleted due to unique_ptr
    Servo(const Servo& other) = delete;
    Servo& operator=(const Servo& other) = delete;

    // Move constructor and assignment operator
    Servo(Servo&& other) noexcept;
    Servo& operator=(Servo&& other) noexcept;

    /**
     * @brief Create a builder instance
     */
    static Builder builder() {
        return Builder();
    }

    /**
     * @brief Update both motor and encoder physics
     * @param dt Time step in seconds
     */
    void update(double dt) {
        motor_->update(dt);
        encoder_->update(motor_->getAngularVelocity(), dt);
    }

    /**
     * @brief Set motor control signal
     */
    void setControlSignal(int signal) {
        motor_->setControlSignal(signal);
    }

    /**
     * @brief Start CAN communication if enabled
     */
    void startCAN();

    /**
     * @brief Stop CAN communication if enabled
     */
    void stopCAN();

    /**
     * @brief Check if CAN is enabled and running
     */
    bool isCANEnabled() const {
        return can_board_ != nullptr;
    }

    bool isCANRunning() const;

    /**
     * @brief Get shared pointer to motor for direct access
     */
    std::shared_ptr<Motor> getMotorPtr() { return motor_; }
    std::shared_ptr<const Motor> getMotorPtr() const { return motor_; }

    /**
     * @brief Get shared pointer to encoder for direct access
     */
    std::shared_ptr<Encoder> getEncoderPtr() { return encoder_; }
    std::shared_ptr<const Encoder> getEncoderPtr() const { return encoder_; }

    /**
     * @brief Get motor reference for direct access
     */
    Motor& getMotor() { return *motor_; }
    const Motor& getMotor() const { return *motor_; }

    /**
     * @brief Get encoder reference for direct access
     */
    Encoder& getEncoder() { return *encoder_; }
    const Encoder& getEncoder() const { return *encoder_; }

    /**
     * @brief Get CanBoard reference if enabled
     */
    CanBoard* getCanBoard() { return can_board_.get(); }
    const CanBoard* getCanBoard() const { return can_board_.get(); }

    /**
     * @brief Reset both motor and encoder to initial state
     */
    void reset() {
        motor_->reset();
        encoder_->reset();
    }

    /**
     * @brief Stop the motor (set control signal to 0)
     */
    void stop() {
        motor_->setControlSignal(0);
        stopCAN();
    }

    // Convenience methods that delegate to motor
    int getControlSignal() const { return motor_->getControlSignal(); }
    double getAngularVelocity() const { return motor_->getAngularVelocity(); }
    double getAngularPosition() const { return motor_->getAngularPosition(); }

    // Convenience methods that delegate to encoder
    long getEncoderPosition() const { return encoder_->getPositionSteps(); }
    double getEncoderPositionRadians() const { return encoder_->getPositionRadians(); }
};
