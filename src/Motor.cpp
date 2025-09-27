#include "Motor.h"
#include <algorithm>
#include <cmath>

Motor::Motor(double max_angular_velocity_rpm, int max_control_signal)
    : control_signal_(0), current_(0.0), torque_(0.0), angular_velocity_(0.0), 
      angular_position_(0.0), max_control_signal_(max_control_signal) {

    // Convert RPM to rad/s: RPM * (2*π/60)
    max_angular_velocity_ = max_angular_velocity_rpm * (2.0 * M_PI / 60.0);
}

void Motor::update(double dt, double load_torque) {
    // Clamp control signal to limits
    control_signal_ = std::clamp(control_signal_, -max_control_signal_, max_control_signal_);

    // Calculate target steady-state velocity based on control signal
    // At max control signal (1000), we should reach max angular velocity
    double target_velocity = (static_cast<double>(control_signal_) / static_cast<double>(max_control_signal_)) * max_angular_velocity_;

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
    current_ = normalized_effort * static_cast<double>(std::abs(control_signal_)) / static_cast<double>(max_control_signal_) * 5.0;
    torque_ = current_ * 0.01; // Simple torque calculation for display

    // Stop motor completely when control signal is 0
    if (control_signal_ == 0) {
        angular_velocity_ = 0.0;
        current_ = 0.0;
        torque_ = 0.0;
    }
}

void Motor::setControlSignal(int control_signal) {
    control_signal_ = control_signal;
}

int Motor::getControlSignal() const { return control_signal_; }
double Motor::getCurrent() const { return current_; }
double Motor::getTorque() const { return torque_; }
double Motor::getAngularVelocity() const { return angular_velocity_; }
double Motor::getAngularPosition() const { return angular_position_; }

double Motor::getMaxAngularVelocity() const { return max_angular_velocity_; }
int Motor::getMaxControlSignal() const { return max_control_signal_; }

void Motor::setMaxControlSignal(int max_control_signal) {
    max_control_signal_ = max_control_signal;
}

void Motor::setMaxAngularVelocity(double max_velocity_rpm) {
    max_angular_velocity_ = max_velocity_rpm * (2.0 * M_PI / 60.0);
}

void Motor::reset() {
    control_signal_ = 0;
    current_ = 0.0;
    torque_ = 0.0;
    angular_velocity_ = 0.0;
    angular_position_ = 0.0;
}