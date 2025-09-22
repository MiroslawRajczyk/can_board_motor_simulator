#pragma once

class Motor {
private:
    double voltage_;          // Applied voltage (V)
    double current_;          // Motor current (A)
    double torque_;           // Output torque (Nm)
    double angular_velocity_; // Angular velocity (rad/s)
    double angular_position_; // Angular position (rad)
    
    // Motor parameters
    double resistance_;       // Winding resistance (Ohms)
    double inductance_;       // Winding inductance (H)
    double back_emf_constant_;// Back EMF constant (V*s/rad)
    double torque_constant_;  // Torque constant (Nm/A)
    double inertia_;          // Rotor inertia (kg*m^2)
    double friction_;         // Friction coefficient
    
    double max_voltage_;      // Maximum voltage limit
    double max_current_;      // Maximum current limit
    double max_angular_velocity_; // Maximum angular velocity (rad/s)

public:
    Motor(double resistance = 1.0, double inductance = 0.001, 
          double back_emf_constant = 0.01, double torque_constant = 0.01,
          double inertia = 0.0001, double friction = 0.0001,
          double max_angular_velocity_rpm = 60.0); // 60 RPM default
    
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
    double getResistance() const;
    double getInductance() const;
    double getBackEmfConstant() const;
    double getTorqueConstant() const;
    double getInertia() const;
    double getFriction() const;
    
    // Set motor limits
    void setMaxVoltage(double max_voltage);
    void setMaxCurrent(double max_current);
    
    // Get/Set maximum angular velocity
    double getMaxAngularVelocity() const;
    void setMaxAngularVelocity(double max_velocity_rpm);
    
    // Reset motor state
    void reset();
};