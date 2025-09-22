#pragma once

class Encoder {
private:
    long position_;           // Current encoder position (ticks)
    double resolution_;       // Pulses per revolution
    double velocity_;         // Angular velocity in rad/s
    double last_position_;    // Previous position for velocity calculation
    double last_time_;        // Previous time for velocity calculation

public:
    Encoder(double resolution = 1000.0);
    
    // Update encoder position based on motor rotation
    void update(double angular_velocity, double dt);
    
    // Get current position in ticks
    long getPosition() const;
    
    // Get current position in radians
    double getPositionRadians() const;
    
    // Get current velocity in rad/s
    double getVelocity() const;
    
    // Reset encoder position to zero
    void reset();
    
    // Set resolution (pulses per revolution)
    void setResolution(double resolution);
};