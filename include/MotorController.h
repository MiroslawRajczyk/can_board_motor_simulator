#pragma once
#include "Motor.h"
#include "Encoder.h"
#include <string>

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
    enum ControlMode { OPEN_LOOP, POSITION_CONTROL, VELOCITY_CONTROL, IDLE };
    ControlMode control_mode_;

    // Limits
    double max_voltage_;
    double max_velocity_;

    // Motor running state
    bool is_running_;

public:
    MotorController();

    // Update the controller and motor simulation
    void update(double dt, double load_torque = 0.0);

    // Control methods
    void setControlSignal(int control_signal);  // Open loop control
    void setPosition(double position_radians);  // Position control
    void setVelocity(double velocity_rad_s);    // Velocity control
    void stop();  // Stop motor and set to idle

    // PID tuning
    void setPIDGains(double kp, double ki, double kd);

    // Get motor and encoder data
    const Motor& getMotor() const;
    const Encoder& getEncoder() const;

    // Get current setpoint and error
    double getSetpoint() const;
    double getCurrentError() const;

    // Get current control mode as string
    std::string getControlModeString() const;

    // Check if motor is running
    bool isRunning() const;

    // Reset controller
    void reset();
};