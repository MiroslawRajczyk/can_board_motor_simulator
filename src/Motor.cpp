#include "Motor.h"
#include <algorithm>
#include <cmath>

Motor::Motor(double max_angular_velocity_rpm, int max_control_signal, double motor_time_constant)
    : control_signal_(0), angular_velocity_(0.0),
      angular_position_(0.0), max_control_signal_(max_control_signal),
      motor_time_constant_(motor_time_constant) {

    // Convert RPM to rad/s: RPM * (2*π/60)
    max_angular_velocity_ = max_angular_velocity_rpm * (2.0 * M_PI / 60.0);

    // Cache expensive calculations
    inv_time_constant_ = 1.0 / motor_time_constant_;
    inv_max_control_signal_ = 1.0 / static_cast<double>(max_control_signal_);
}

void Motor::update(double dt) {
    // Calculate target steady-state velocity based on control signal
    // At max control signal (1000), we should reach max angular velocity
    double target_velocity = static_cast<double>(control_signal_) * inv_max_control_signal_ * max_angular_velocity_;

    // Simple velocity control - move towards target velocity with realistic time constant
    double velocity_error = target_velocity - angular_velocity_;

    // Use the configurable motor time constant (cached inverse)
    double velocity_change = velocity_error * inv_time_constant_ * dt;

    // Update angular velocity
    angular_velocity_ += velocity_change;

    // Apply maximum angular velocity limit (safety)
    angular_velocity_ = std::clamp(angular_velocity_, -max_angular_velocity_, max_angular_velocity_);

    // Update position
    angular_position_ += angular_velocity_ * dt;
}

void Motor::setControlSignal(int control_signal) {
    control_signal_ = std::clamp(control_signal, -max_control_signal_, max_control_signal_);
}

int Motor::getControlSignal() const { return control_signal_; }
double Motor::getAngularVelocity() const { return angular_velocity_; }
double Motor::getAngularPosition() const { return angular_position_; }

double Motor::getMaxAngularVelocity() const { return max_angular_velocity_; }
int Motor::getMaxControlSignal() const { return max_control_signal_; }
double Motor::getMotorTimeConstant() const { return motor_time_constant_; }

void Motor::setMaxControlSignal(int max_control_signal) {
    max_control_signal_ = max_control_signal;
}

void Motor::setMaxAngularVelocity(double max_velocity_rpm) {
    max_angular_velocity_ = max_velocity_rpm * (2.0 * M_PI / 60.0);
}

void Motor::reset() {
    control_signal_ = 0;
    angular_velocity_ = 0.0;
    angular_position_ = 0.0;
}