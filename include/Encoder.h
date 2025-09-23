#pragma once
#include <cmath>

class Encoder {
private:
    long position_steps_;     // Current encoder position in steps
    double fractional_steps_; // Accumulated fractional steps
    int bit_resolution_;      // Encoder bit resolution (e.g., 12 bits)
    long max_steps_;          // Maximum steps per revolution (2^bit_resolution)
    double angular_velocity_; // Angular velocity in rad/s
    bool direction_inverted_; // Encoder direction: false = normal, true = inverted

    // Conversion helpers
    double stepsToRadians(long steps) const;
    long radiansToSteps(double radians) const;

public:
    Encoder(int bit_resolution = 12, bool direction_inverted = false);
    
    // Update encoder position based on motor rotation
    void update(double angular_velocity, double dt);
    
    // Get current position in steps
    long getPositionSteps() const;
    
    // Get current position in radians
    double getPositionRadians() const;
    
    // Get current velocity in rad/s
    double getVelocity() const;
    
    // Reset encoder position to zero
    void reset();
    
    // Get encoder specifications
    int getBitResolution() const;
    long getMaxSteps() const;
    double getResolutionRadians() const; // Resolution in radians per step
    bool isDirectionInverted() const;
};