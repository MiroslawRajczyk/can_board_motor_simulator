#pragma once
#include "Motor.h"
#include "Encoder.h"

class MotorController {
private:
    Motor motor_;
    Encoder encoder_;
    
    // PID controller parameters
    double kp_, ki_, kd_;
    double integral_error_;
    double previous_error_;
    double setpoint_;
    
    // Control mode
    enum ControlMode { OPEN_LOOP, POSITION_CONTROL, VELOCITY_CONTROL };
    ControlMode control_mode_;
    
    // Limits
    double max_voltage_;
    double max_velocity_;

public:
    MotorController();
    
    // Update the controller and motor simulation
    void update(double dt, double load_torque = 0.0);
    
    // Control methods
    void setVoltage(double voltage);  // Open loop control
    void setPosition(double position_radians);  // Position control
    void setVelocity(double velocity_rad_s);    // Velocity control
    
    // PID tuning
    void setPIDGains(double kp, double ki, double kd);
    
    // Get motor and encoder data
    const Motor& getMotor() const;
    const Encoder& getEncoder() const;
    
    // Get current setpoint and error
    double getSetpoint() const;
    double getCurrentError() const;
    
    // Reset controller
    void reset();
};