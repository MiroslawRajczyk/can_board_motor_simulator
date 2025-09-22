#include "Encoder.h"
#include <cmath>

Encoder::Encoder(int bit_resolution, bool direction_inverted)
    : position_steps_(0), bit_resolution_(bit_resolution), angular_velocity_(0.0),
      direction_inverted_(direction_inverted) {
    max_steps_ = 1L << bit_resolution_;  // Bit shift is equivalent to 2^n
}

void Encoder::update(double angular_velocity, double dt) {
    angular_velocity_ = angular_velocity;

    // Calculate position change in radians
    double position_change_radians = angular_velocity * dt;

    // Apply direction inversion if needed
    if (!direction_inverted_) {
        position_change_radians = -position_change_radians;
    }

    // Convert to encoder steps
    long steps_change = radiansToSteps(position_change_radians);

    // Update position with wraparound for absolute encoder
    position_steps_ += steps_change;

    // Handle wraparound: absolute encoders wrap around at max_steps
    if (position_steps_ >= max_steps_) {
        position_steps_ -= max_steps_;
    } else if (position_steps_ < 0) {
        position_steps_ += max_steps_;
    }
}

long Encoder::getPositionSteps() const {
    return position_steps_;
}

double Encoder::getPositionRadians() const {
    return stepsToRadians(position_steps_);
}

double Encoder::getVelocity() const {
    return angular_velocity_;
}

void Encoder::reset() {
    position_steps_ = 0;
    angular_velocity_ = 0.0;
}

int Encoder::getBitResolution() const {
    return bit_resolution_;
}

long Encoder::getMaxSteps() const {
    return max_steps_;
}

double Encoder::getResolutionRadians() const {
    return (2.0 * M_PI) / max_steps_;  // Radians per step
}

// Private helper methods
double Encoder::stepsToRadians(long steps) const {
    return (steps * 2.0 * M_PI) / max_steps_;
}

long Encoder::radiansToSteps(double radians) const {
    return static_cast<long>(round((radians * max_steps_) / (2.0 * M_PI)));
}

bool Encoder::isDirectionInverted() const {
    return direction_inverted_;
}