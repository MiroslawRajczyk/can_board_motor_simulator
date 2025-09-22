#include "Motor.h"
#include <algorithm>
#include <cmath>

Motor::Motor(double resistance, double inductance, double back_emf_constant, 
             double torque_constant, double inertia, double friction)
    : voltage_(0.0), current_(0.0), torque_(0.0), angular_velocity_(0.0), 
      angular_position_(0.0), resistance_(resistance), inductance_(inductance),
      back_emf_constant_(back_emf_constant), torque_constant_(torque_constant),
      inertia_(inertia), friction_(friction), max_voltage_(12.0), max_current_(10.0) {
}

void Motor::update(double dt, double load_torque) {
    // Clamp voltage to limits
    voltage_ = std::clamp(voltage_, -max_voltage_, max_voltage_);
    
    // Calculate back EMF
    double back_emf = back_emf_constant_ * angular_velocity_;
    
    // Calculate current using simplified electrical model
    // I = (V - back_emf) / R (ignoring inductance for simplicity)
    current_ = (voltage_ - back_emf) / resistance_;
    
    // Clamp current to limits
    current_ = std::clamp(current_, -max_current_, max_current_);
    
    // Calculate motor torque
    torque_ = torque_constant_ * current_;
    
    // Calculate net torque (motor torque - friction - load)
    double friction_torque = friction_ * angular_velocity_;
    double net_torque = torque_ - friction_torque - load_torque;
    
    // Update angular acceleration using Newton's law for rotation
    double angular_acceleration = net_torque / inertia_;
    
    // Update angular velocity and position using Euler integration
    angular_velocity_ += angular_acceleration * dt;
    angular_position_ += angular_velocity_ * dt;
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