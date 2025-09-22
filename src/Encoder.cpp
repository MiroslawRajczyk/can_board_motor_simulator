#include "Encoder.h"
#include <cmath>

Encoder::Encoder(double resolution) 
    : position_(0), resolution_(resolution), velocity_(0.0), 
      last_position_(0.0), last_time_(0.0) {
}

void Encoder::update(double angular_velocity, double dt) {
    // Update velocity
    velocity_ = angular_velocity;
    
    // Calculate position change in radians
    double position_change = angular_velocity * dt;
    
    // Convert to encoder ticks
    double ticks_change = (position_change * resolution_) / (2.0 * M_PI);
    
    // Update position
    position_ += static_cast<long>(round(ticks_change));
}

long Encoder::getPosition() const {
    return position_;
}

double Encoder::getPositionRadians() const {
    return (position_ * 2.0 * M_PI) / resolution_;
}

double Encoder::getVelocity() const {
    return velocity_;
}

void Encoder::reset() {
    position_ = 0;
    velocity_ = 0.0;
    last_position_ = 0.0;
    last_time_ = 0.0;
}

void Encoder::setResolution(double resolution) {
    resolution_ = resolution;
}