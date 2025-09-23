#pragma once

class Motor {
private:
    double voltage_;          // Applied voltage (V)
    double current_;          // Motor current (A) - simplified calculation
    double torque_;           // Output torque (Nm) - simplified calculation
    double angular_velocity_; // Angular velocity (rad/s)
    double angular_position_; // Angular position (rad)
    
    double max_voltage_;      // Maximum voltage limit
    double max_angular_velocity_; // Maximum angular velocity (rad/s)

public:
    Motor(double max_angular_velocity_rpm = 60.0, double max_voltage = 12.0);
    
    // Update motor physics simulation
    void update(double dt, double load_torque = 0.0);
    
    // Set applied voltage
    void setVoltage(double voltage);
    
    // Get motor state
    double getVoltage() const;
    double getCurrent() const;
    double getTorque() const;
    double getAngularVelocity() const;
    double getAngularPosition() const;
    
    // Get motor parameters
    double getMaxAngularVelocity() const;
    double getMaxVoltage() const;
    
    // Set motor limits
    void setMaxVoltage(double max_voltage);
    void setMaxAngularVelocity(double max_velocity_rpm);
    
    // Reset motor state
    void reset();
};