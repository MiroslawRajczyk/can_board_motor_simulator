#pragma once

class Motor {
private:
    int control_signal_;      // Control signal (-1000 to +1000)
    double angular_velocity_; // Angular velocity (rad/s)
    double angular_position_; // Angular position (rad)

    int max_control_signal_;         // Maximum control signal (1000)
    double max_angular_velocity_;    // Maximum angular velocity (rad/s)

public:
    Motor(double max_angular_velocity_rpm = 60.0, int max_control_signal = 1000);

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

    // Set motor limits
    void setMaxControlSignal(int max_control_signal);
    void setMaxAngularVelocity(double max_velocity_rpm);

    // Reset motor state
    void reset();
};