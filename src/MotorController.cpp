#include "MotorController.h"
#include <algorithm>
#include <cmath>
#include <string>

MotorController::MotorController()
    : motor_(), encoder_(), kp_(1.0), ki_(0.1), kd_(0.01),
      integral_error_(0.0), previous_error_(0.0), setpoint_(0.0),
      control_mode_(IDLE), max_voltage_(12.0), max_velocity_(100.0),
      is_running_(false) {
}

void MotorController::update(double dt, double load_torque) {
    double control_output = 0.0;
    
    switch (control_mode_) {
        case IDLE:
            // Motor is idle, no voltage applied
            motor_.setVoltage(0.0);
            is_running_ = false;
            break;
            
        case OPEN_LOOP:
            // Voltage is already set, no feedback control
            is_running_ = (motor_.getVoltage() != 0.0);
            break;
            
        case POSITION_CONTROL: {
            double current_position = encoder_.getPositionRadians();
            double error = setpoint_ - current_position;
            
            // PID calculation
            integral_error_ += error * dt;
            double derivative_error = (error - previous_error_) / dt;
            
            control_output = kp_ * error + ki_ * integral_error_ + kd_ * derivative_error;
            control_output = std::clamp(control_output, -max_voltage_, max_voltage_);
            
            motor_.setVoltage(control_output);
            previous_error_ = error;
            is_running_ = true;
            break;
        }
        
        case VELOCITY_CONTROL: {
            double current_velocity = encoder_.getVelocity();
            double error = setpoint_ - current_velocity;
            
            // PID calculation
            integral_error_ += error * dt;
            double derivative_error = (error - previous_error_) / dt;
            
            control_output = kp_ * error + ki_ * integral_error_ + kd_ * derivative_error;
            control_output = std::clamp(control_output, -max_voltage_, max_voltage_);
            
            motor_.setVoltage(control_output);
            previous_error_ = error;
            is_running_ = true;
            break;
        }
    }
    
    // Update motor physics
    motor_.update(dt, load_torque);
    
    // Update encoder with motor's angular velocity
    encoder_.update(motor_.getAngularVelocity(), dt);
}

void MotorController::setVoltage(double voltage) {
    control_mode_ = OPEN_LOOP;
    motor_.setVoltage(voltage);
    integral_error_ = 0.0;
    previous_error_ = 0.0;
    is_running_ = (voltage != 0.0);
}

void MotorController::setPosition(double position_radians) {
    control_mode_ = POSITION_CONTROL;
    setpoint_ = position_radians;
    integral_error_ = 0.0;
    previous_error_ = 0.0;
    is_running_ = true;
}

void MotorController::setVelocity(double velocity_rad_s) {
    control_mode_ = VELOCITY_CONTROL;
    setpoint_ = std::clamp(velocity_rad_s, -max_velocity_, max_velocity_);
    integral_error_ = 0.0;
    previous_error_ = 0.0;
    is_running_ = true;
}

void MotorController::stop() {
    control_mode_ = IDLE;
    motor_.setVoltage(0.0);
    integral_error_ = 0.0;
    previous_error_ = 0.0;
    setpoint_ = 0.0;
    is_running_ = false;
    
    // Don't reset the motor - let it coast down naturally with physics
}

void MotorController::setPIDGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

const Motor& MotorController::getMotor() const {
    return motor_;
}

const Encoder& MotorController::getEncoder() const {
    return encoder_;
}

double MotorController::getSetpoint() const {
    return setpoint_;
}

double MotorController::getCurrentError() const {
    if (control_mode_ == POSITION_CONTROL) {
        return setpoint_ - encoder_.getPositionRadians();
    } else if (control_mode_ == VELOCITY_CONTROL) {
        return setpoint_ - encoder_.getVelocity();
    }
    return 0.0;
}

std::string MotorController::getControlModeString() const {
    switch (control_mode_) {
        case IDLE: return "IDLE";
        case OPEN_LOOP: return "OPEN_LOOP";
        case POSITION_CONTROL: return "POSITION_CONTROL";
        case VELOCITY_CONTROL: return "VELOCITY_CONTROL";
        default: return "UNKNOWN";
    }
}

bool MotorController::isRunning() const {
    return is_running_;
}

void MotorController::reset() {
    motor_.reset();
    encoder_.reset();
    integral_error_ = 0.0;
    previous_error_ = 0.0;
    setpoint_ = 0.0;
    control_mode_ = IDLE;
    is_running_ = false;
}