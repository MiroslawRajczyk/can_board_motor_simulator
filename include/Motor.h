#pragma once

/**
 * @brief DC Motor simulation class with configurable parameters
 *
 * Simulates a DC motor with realistic physics including:
 * - Proportional velocity control (constant voltage -> constant steady-state velocity)
 * - Configurable time constant for acceleration/deceleration response
 * - Position and velocity tracking
 *
 * Example usage:
 *   Motor motor = Motor::builder()
 *       .maxVelocityRPM(120.0)
 *       .maxControlSignal(1000)
 *       .timeConstant(0.15);
 */
class Motor {
public:
    /**
     * @brief Builder class for Motor construction with fluent interface
     *
     * Provides a clear and flexible way to configure Motor parameters.
     * Example usage:
     *   Motor motor = Motor::builder()
     *       .maxVelocityRPM(120.0)
     *       .maxControlSignal(1000)
     *       .timeConstant(0.15);
     */
    class Builder {
    private:
        double max_angular_velocity_rpm_ = 60.0;  ///< Maximum angular velocity in RPM (default: 60 RPM)
        int max_control_signal_ = 1000;           ///< Maximum control signal value (default: 1000)
        double motor_time_constant_ = 0.15;       ///< Motor response time constant in seconds (default: 0.15s)

    public:
        /**
         * @brief Set maximum angular velocity in RPM
         * @param rpm Maximum velocity in rotations per minute
         * @return Reference to this builder for method chaining
         */
        Builder& maxVelocityRPM(double rpm) {
            max_angular_velocity_rpm_ = rpm;
            return *this;
        }

        /**
         * @brief Set maximum control signal value
         * @param signal Maximum control signal (typical value is 100)
         * @return Reference to this builder for method chaining
         */
        Builder& maxControlSignal(int signal) {
            max_control_signal_ = signal;
            return *this;
        }

        /**
         * @brief Set motor time constant (response speed)
         * @param time_constant Time in seconds for motor to reach ~63% of target velocity
         *                     Smaller values = faster response, larger values = slower response
         * @return Reference to this builder for method chaining
         */
        Builder& timeConstant(double time_constant) {
            motor_time_constant_ = time_constant;
            return *this;
        }

        /**
         * @brief Build and return the configured Motor instance
         * @return Motor object with specified parameters
         */
        Motor build() {
            return Motor(max_angular_velocity_rpm_, max_control_signal_, motor_time_constant_);
        }

        /**
         * @brief Automatic conversion to Motor (eliminates need for .build())
         * @return Motor object with specified parameters
         */
        operator Motor() {
            return build();
        }
    };

private:
    int control_signal_;      // Control signal (-1000 to +1000)
    double angular_velocity_; // Angular velocity (rad/s)
    double angular_position_; // Angular position (rad)

    int max_control_signal_;         // Maximum control signal (1000)
    double max_angular_velocity_;    // Maximum angular velocity (rad/s)
    double motor_time_constant_;     // Motor time constant (seconds)

public:
    Motor(double max_angular_velocity_rpm = 60.0, int max_control_signal = 1000, double motor_time_constant = 0.15);

    // Static method to create a Builder
    static Builder builder() {
        return Builder();
    }

    // Update motor physics simulation
    void update(double dt);

    // Set control signal
    void setControlSignal(int control_signal);

    // Get motor state
    int getControlSignal() const;
    double getAngularVelocity() const;
    double getAngularPosition() const;

    // Get motor parameters
    double getMaxAngularVelocity() const;
    int getMaxControlSignal() const;
    double getMotorTimeConstant() const;

    // Set motor limits
    void setMaxControlSignal(int max_control_signal);
    void setMaxAngularVelocity(double max_velocity_rpm);

    // Reset motor state
    void reset();
};