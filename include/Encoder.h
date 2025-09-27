#pragma once
#include <cmath>

// Constexpr helpers for encoder calculations
constexpr double radiansToSteps(double radians, long max_steps) {
    return (radians * max_steps) / (2.0 * M_PI);
}

constexpr double stepsToRadians(long steps, long max_steps) {
    return (steps * 2.0 * M_PI) / max_steps;
}

constexpr long maxStepsFromBits(int bit_resolution) {
    return 1L << bit_resolution;
}

/**
 * @brief Absolute rotary encoder simulation class
 *
 * Simulates an absolute rotary encoder with configurable parameters:
 * - Bit resolution (determines steps per revolution: 2^bits)
 * - Direction inversion (normal or inverted counting direction)
 * - Position tracking in steps and radians
 *
 * Example usage:
 *   Encoder encoder = Encoder::builder()
 *       .bitResolution(18)        // 262,144 steps per revolution
 *       .directionInverted(false); // Normal direction
 */
class Encoder {
public:
    /**
     * @brief Builder class for Encoder construction with fluent interface
     *
     * Provides a clear and flexible way to configure Encoder parameters.
     * Example usage:
     *   Encoder encoder = Encoder::builder()
     *       .bitResolution(18)
     *       .directionInverted(false);
     */
    class Builder {
    private:
        int bit_resolution_ = 18;         ///< Encoder bit resolution (default: 18 bits = 262,144 steps/rev)
        bool direction_inverted_ = false; ///< Direction inversion flag (default: false = normal direction)

    public:
        /**
         * @brief Set encoder bit resolution
         * @param resolution Number of bits (e.g., 12=4096 steps, 18=262144 steps per revolution)
         * @return Reference to this builder for method chaining
         */
        Builder& bitResolution(int resolution) {
            bit_resolution_ = resolution;
            return *this;
        }

        /**
         * @brief Set direction inversion
         * @param inverted true = inverted direction (positive control decreases encoder value)
         *                false = normal direction (positive control increases encoder value)
         * @return Reference to this builder for method chaining
         */
        Builder& directionInverted(bool inverted) {
            direction_inverted_ = inverted;
            return *this;
        }

        /**
         * @brief Build and return the configured Encoder instance
         * @return Encoder object with specified parameters
         */
        Encoder build() {
            return Encoder(bit_resolution_, direction_inverted_);
        }

        /**
         * @brief Automatic conversion to Encoder (eliminates need for .build())
         * @return Encoder object with specified parameters
         */
        operator Encoder() {
            return build();
        }
    };

private:
    long position_steps_;     // Current encoder position in steps
    double fractional_steps_; // Accumulated fractional steps
    int bit_resolution_;      // Encoder bit resolution (e.g., 12 bits)
    long max_steps_;          // Maximum steps per revolution (2^bit_resolution)
    bool direction_inverted_; // Encoder direction: false = normal, true = inverted

    // Cached values for performance optimization
    double steps_per_radian_; // max_steps_ / (2.0 * M_PI) (cached)
    double radians_per_step_; // (2.0 * M_PI) / max_steps_ (cached)

    // Conversion helpers
    double stepsToRadians(long steps) const;
    long radiansToSteps(double radians) const;

public:
    Encoder(int bit_resolution = 18, bool direction_inverted = false);

    // Static method to create a Builder
    static Builder builder() {
        return Builder();
    }

    // Update encoder position based on motor rotation
    void update(double angular_velocity, double dt);

    // Get current position in steps
    long getPositionSteps() const;

    // Get current position in radians
    double getPositionRadians() const;

    // Reset encoder position to zero
    void reset();

    // Get encoder specifications
    int getBitResolution() const;
    long getMaxSteps() const;
    double getResolutionRadians() const; // Resolution in radians per step
    bool isDirectionInverted() const;
};