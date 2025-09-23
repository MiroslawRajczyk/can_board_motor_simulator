#include "Motor.h"
#include <algorithm>
#include <cmath>

Motor::Motor(double max_angular_velocity_rpm, double max_voltage)
    : voltage_(0.0), current_(0.0), torque_(0.0), angular_velocity_(0.0), 
      angular_position_(0.0), max_voltage_(max_voltage) {
    
    // Convert RPM to rad/s: RPM * (2*π/60)
    max_angular_velocity_ = max_angular_velocity_rpm * (2.0 * M_PI / 60.0);
}

void Motor::update(double dt, double load_torque) {
    // Clamp voltage to limits
    voltage_ = std::clamp(voltage_, -max_voltage_, max_voltage_);
    
    // Calculate target steady-state velocity based on voltage
    // At max voltage, we should reach max angular velocity
    double target_velocity = (voltage_ / max_voltage_) * max_angular_velocity_;
    
    // Simple velocity control - move towards target velocity
    double velocity_error = target_velocity - angular_velocity_;
    
    // Use a velocity time constant to control how fast we reach target velocity
    double velocity_time_constant = 0.2; // 200ms time constant for smooth acceleration
    double velocity_change = (velocity_error / velocity_time_constant) * dt;
    
    // Update angular velocity
    angular_velocity_ += velocity_change;
    
    // Apply maximum angular velocity limit (safety)
    angular_velocity_ = std::clamp(angular_velocity_, -max_angular_velocity_, max_angular_velocity_);
    
    // Update position
    angular_position_ += angular_velocity_ * dt;
    
    // Calculate simplified current and torque for display purposes
    // Higher velocity error means more "effort" needed
    double normalized_effort = std::abs(velocity_error) / max_angular_velocity_;
    current_ = normalized_effort * std::abs(voltage_) / max_voltage_ * 5.0; // Scale to reasonable current values
    torque_ = current_ * 0.01; // Simple torque calculation for display
    
    // Stop motor completely when voltage is 0
    if (voltage_ == 0.0) {
        angular_velocity_ = 0.0;
        current_ = 0.0;
        torque_ = 0.0;
    }
}

void Motor::setVoltage(double voltage) {
    voltage_ = voltage;
}

double Motor::getVoltage() const { return voltage_; }
double Motor::getCurrent() const { return current_; }
double Motor::getTorque() const { return torque_; }
double Motor::getAngularVelocity() const { return angular_velocity_; }
double Motor::getAngularPosition() const { return angular_position_; }

double Motor::getMaxAngularVelocity() const { return max_angular_velocity_; }
double Motor::getMaxVoltage() const { return max_voltage_; }

void Motor::setMaxVoltage(double max_voltage) {
    max_voltage_ = max_voltage;
}

void Motor::setMaxAngularVelocity(double max_velocity_rpm) {
    max_angular_velocity_ = max_velocity_rpm * (2.0 * M_PI / 60.0);
}

void Motor::reset() {
    voltage_ = 0.0;
    current_ = 0.0;
    torque_ = 0.0;
    angular_velocity_ = 0.0;
    angular_position_ = 0.0;
}