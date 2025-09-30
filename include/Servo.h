#pragma once

#include "Motor.h"
#include "Encoder.h"
#include "CanBoard.h"
#include <memory>
#include <string>
#include <optional>

/**
 * @brief Servo class that combines Motor, Encoder, and CanBoard
 *
 * This class provides an interface for a servo system that includes
 * the motor physics simulation, encoder position tracking, and CAN communication.
 * The motor, encoder, and CAN board are automatically synchronized during updates.
 */
class Servo {
private:
    Motor motor_;
    Encoder encoder_;
    std::unique_ptr<CanBoard> can_board_;

public:
    /**
     * @brief Builder class for Servo construction with fluent interface
     *
     * Provides a clear way to configure Motor, Encoder, and CanBoard parameters
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

        // CanBoard parameters
        uint32_t can_id_ = 0x00;
        std::string can_interface_ = "vcan0";
        bool can_enabled_ = true;

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
         * @brief Set CAN ID for the servo
         */
        Builder& canId(uint32_t id) {
            can_id_ = id;
            return *this;
        }

        /**
         * @brief Set CAN interface name
         */
        Builder& canInterface(const std::string& interface) {
            can_interface_ = interface;
            return *this;
        }

        /**
         * @brief Enable or disable CAN communication
         */
        Builder& canEnabled(bool enabled) {
            can_enabled_ = enabled;
            return *this;
        }

        /**
         * @brief Build the Servo
         */
        Servo build() {
            return Servo(max_velocity_rpm_, max_control_signal_, motor_time_constant_,
                         bit_resolution_, direction_inverted_, can_id_, can_interface_, can_enabled_);
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
          int bit_resolution, bool direction_inverted, uint32_t can_id, 
          const std::string& can_interface, bool can_enabled);

public:
    /**
     * @brief Default constructor with reasonable defaults
     */
    Servo();

    /**
     * @brief Move constructor
     */
    Servo(Servo&& other) noexcept;

    /**
     * @brief Move assignment operator
     */
    Servo& operator=(Servo&& other) noexcept;

    /**
     * @brief Destructor
     */
    ~Servo();

    /**
     * @brief Create a builder instance
     */
    static Builder builder() {
        return Builder();
    }

    /**
     * @brief Start the servo and CAN communication
     */
    void start();

    /**
     * @brief Stop the servo and CAN communication
     */
    void stop();

    /**
     * @brief Update motor and encoder physics
     * @param dt Time step in seconds
     */
    void update(double dt) {
        motor_.update(dt);
        encoder_.update(motor_.getAngularVelocity(), dt);
    }

    /**
     * @brief Set motor control signal
     */
    void setControlSignal(int signal) {
        motor_.setControlSignal(signal);
    }

    /**
     * @brief Get motor reference for direct access
     */
    Motor& getMotor() { return motor_; }
    const Motor& getMotor() const { return motor_; }

    /**
     * @brief Get encoder reference for direct access
     */
    Encoder& getEncoder() { return encoder_; }
    const Encoder& getEncoder() const { return encoder_; }

    /**
     * @brief Get CAN board reference for direct access
     */
    CanBoard* getCanBoard() { return can_board_ ? can_board_.get() : nullptr; }
    const CanBoard* getCanBoard() const { return can_board_ ? can_board_.get() : nullptr; }

    /**
     * @brief Reset motor and encoder to initial state
     */
    void reset() {
        motor_.reset();
        encoder_.reset();
    }

    // Convenience methods that delegate to motor
    int getControlSignal() const { return motor_.getControlSignal(); }
    double getAngularVelocity() const { return motor_.getAngularVelocity(); }
    double getAngularPosition() const { return motor_.getAngularPosition(); }

    // Convenience methods that delegate to encoder
    long getEncoderPosition() const { return encoder_.getPositionSteps(); }
    double getEncoderPositionRadians() const { return encoder_.getPositionRadians(); }

    // Convenience methods that delegate to CAN board
    uint32_t getCanId() const;
    bool isCanEnabled() const { return can_board_ != nullptr; }

    // Delete copy constructor and assignment operator to prevent copying
    Servo(const Servo&) = delete;
    Servo& operator=(const Servo&) = delete;
};
