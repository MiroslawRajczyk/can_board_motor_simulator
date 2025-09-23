#include "Motor.h"
#include <algorithm>
#include <cmath>

Motor::Motor(double resistance, double inductance, double back_emf_constant, 
             double torque_constant, double inertia, double friction,
             double max_angular_velocity_rpm)
    : voltage_(0.0), current_(0.0), torque_(0.0), angular_velocity_(0.0), 
      angular_position_(0.0), resistance_(resistance), inductance_(inductance),
      back_emf_constant_(back_emf_constant), torque_constant_(torque_constant),
      inertia_(inertia), friction_(friction), max_voltage_(12.0), max_current_(10.0) {
    
    // Convert RPM to rad/s: RPM * (2*π/60)
    max_angular_velocity_ = max_angular_velocity_rpm * (2.0 * M_PI / 60.0);
}

void Motor::update(double dt, double load_torque) {
    // Clamp voltage to limits
    voltage_ = std::clamp(voltage_, -max_voltage_, max_voltage_);
    
    // Calculate target steady-state velocity based on voltage
    // At max voltage (12V), we should reach max angular velocity
    double target_velocity = (voltage_ / max_voltage_) * max_angular_velocity_;
    
    // Simple velocity control - move towards target velocity
    double velocity_error = target_velocity - angular_velocity_;
    
    // Use a velocity time constant to control how fast we reach target velocity
    double velocity_time_constant = 0.2; // 200ms time constant for smooth acceleration
    double velocity_change = (velocity_error / velocity_time_constant) * dt;
    
    // Update angular velocity
    angular_velocity_ += velocity_change;
    
    // Apply maximum angular velocity limit (should not be needed now but keep as safety)
    angular_velocity_ = std::clamp(angular_velocity_, -max_angular_velocity_, max_angular_velocity_);
    
    // Update position
    angular_position_ += angular_velocity_ * dt;
    
    // Calculate realistic current and torque based on the velocity control effort
    // For negative voltages, current should also be negative
    double normalized_effort = std::abs(velocity_error) / max_angular_velocity_;
    double effort_direction = (velocity_error >= 0) ? 1.0 : -1.0;
    current_ = normalized_effort * effort_direction * (std::abs(voltage_) / resistance_);
    current_ = std::clamp(current_, -max_current_, max_current_);
    torque_ = torque_constant_ * current_;
    
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

double Motor::getResistance() const { return resistance_; }
double Motor::getInductance() const { return inductance_; }
double Motor::getBackEmfConstant() const { return back_emf_constant_; }
double Motor::getTorqueConstant() const { return torque_constant_; }
double Motor::getInertia() const { return inertia_; }
double Motor::getFriction() const { return friction_; }

void Motor::setMaxVoltage(double max_voltage) {
    max_voltage_ = max_voltage;
}

void Motor::setMaxCurrent(double max_current) {
    max_current_ = max_current;
}

void Motor::reset() {
    voltage_ = 0.0;
    current_ = 0.0;
    torque_ = 0.0;
    angular_velocity_ = 0.0;
    angular_position_ = 0.0;
}

double Motor::getMaxAngularVelocity() const {
    return max_angular_velocity_;
}

void Motor::setMaxAngularVelocity(double max_velocity_rpm) {
    max_angular_velocity_ = max_velocity_rpm * (2.0 * M_PI / 60.0);
}