#include "Encoder.h"
#include <cmath>

Encoder::Encoder(int bit_resolution, bool direction_inverted)
    : position_steps_(0), fractional_steps_(0.0), bit_resolution_(bit_resolution),
      direction_inverted_(direction_inverted) {
    max_steps_ = 1L << bit_resolution_;  // Bit shift is equivalent to 2^n

    // Cache expensive calculations
    steps_per_radian_ = static_cast<double>(max_steps_) / (2.0 * M_PI);
    radians_per_step_ = (2.0 * M_PI) / static_cast<double>(max_steps_);
}

void Encoder::update(double angular_velocity, double dt) {
    // Calculate position change in radians
    double position_change_radians = angular_velocity * dt;

    // Apply direction inversion if needed
    if (direction_inverted_) {
        position_change_radians = -position_change_radians;
    }

    // Convert position change to fractional steps using cached conversion
    double fractional_steps_change = position_change_radians * steps_per_radian_;

    // Accumulate fractional steps
    fractional_steps_ += fractional_steps_change;

    // Extract whole steps and update position
    long whole_steps = static_cast<long>(fractional_steps_);
    if (whole_steps != 0) {
        position_steps_ += whole_steps;
        fractional_steps_ -= whole_steps; // Keep only the fractional part

        // Handle wraparound: absolute encoders wrap around at max_steps
        position_steps_ = position_steps_ % max_steps_;
        if (position_steps_ < 0) {
            position_steps_ += max_steps_;
        }
    }
}

long Encoder::getPositionSteps() const {
    return position_steps_;
}

double Encoder::getPositionRadians() const {
    return stepsToRadians(position_steps_);
}

void Encoder::reset() {
    position_steps_ = 0;
    fractional_steps_ = 0.0;
}

int Encoder::getBitResolution() const {
    return bit_resolution_;
}

long Encoder::getMaxSteps() const {
    return max_steps_;
}

double Encoder::getResolutionRadians() const {
    return radians_per_step_;  // Use cached value
}

// Private helper methods
double Encoder::stepsToRadians(long steps) const {
    return steps * radians_per_step_;  // Use cached value
}

long Encoder::radiansToSteps(double radians) const {
    return static_cast<long>(round(radians * steps_per_radian_));  // Use cached value
}

bool Encoder::isDirectionInverted() const {
    return direction_inverted_;
}