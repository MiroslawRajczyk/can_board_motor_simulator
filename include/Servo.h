#pragma once

#include "Motor.h"
#include "Encoder.h"

/**
 * @brief Servo class that combines Motor and Encoder
 *
 * This class provides an interface for a servo system that includes
 * both the motor physics simulation and encoder position tracking.
 * The motor and encoder are automatically synchronized during updates.
 */
class Servo {
private:
    Motor motor_;
    Encoder encoder_;

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
         * @brief Build the Servo
         */
        Servo build() {
            return Servo(max_velocity_rpm_, max_control_signal_, motor_time_constant_,
                         bit_resolution_, direction_inverted_);
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
          int bit_resolution, bool direction_inverted)
        : motor_(Motor::builder()
                 .maxVelocityRPM(max_velocity_rpm)
                 .maxControlSignal(max_control_signal)
                 .timeConstant(motor_time_constant)),
          encoder_(Encoder::builder()
                   .bitResolution(bit_resolution)
                   .directionInverted(direction_inverted)) {}

public:
    /**
     * @brief Default constructor with reasonable defaults
     */
    Servo() : Servo(160.0, 1000, 0.3, 18, false) {}

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
     * @brief Reset both motor and encoder to initial state
     */
    void reset() {
        motor_.reset();
        encoder_.reset();
    }

    /**
     * @brief Stop the motor (set control signal to 0)
     */
    void stop() {
        motor_.setControlSignal(0);
    }

    // Convenience methods that delegate to motor
    int getControlSignal() const { return motor_.getControlSignal(); }
    double getAngularVelocity() const { return motor_.getAngularVelocity(); }
    double getAngularPosition() const { return motor_.getAngularPosition(); }

    // Convenience methods that delegate to encoder
    long getEncoderPosition() const { return encoder_.getPositionSteps(); }
    double getEncoderPositionRadians() const { return encoder_.getPositionRadians(); }
};
