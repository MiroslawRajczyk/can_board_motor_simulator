#pragma once

#include "Servo.h"
#include <vector>
#include <string>

/**
 * @brief Configuration for a single servo
 */
struct ServoConfig {
    double maxVelocityRPM = 60.0;
    int maxControlSignal = 100;
    double timeConstant = 0.15;
    int encoderBitResolution = 18;
    bool encoderDirectionInverted = false;
    uint32_t canId = 0x10;
    std::string canInterface = "vcan0";
    std::string name = "servo";  // Optional name for identification
};

/**
 * @brief Configuration loader for servo systems
 * 
 * Loads servo configurations from JSON files and creates Servo objects
 */
class ConfigLoader {
public:
    /**
     * @brief Load servo configurations from JSON file
     * @param filename Path to JSON configuration file
     * @return Vector of servo configurations, empty on error
     */
    static std::vector<ServoConfig> loadFromFile(const std::string& filename);

    /**
     * @brief Create servos from configuration vector
     * @param configs Vector of servo configurations
     * @return Vector of configured Servo objects
     */
    static std::vector<Servo> createServos(const std::vector<ServoConfig>& configs);

    /**
     * @brief Load servos directly from JSON file
     * @param filename Path to JSON configuration file
     * @return Vector of configured Servo objects
     */
    static std::vector<Servo> loadServosFromFile(const std::string& filename);

    /**
     * @brief Save servo configurations to JSON file
     * @param configs Vector of servo configurations
     * @param filename Output file path
     * @return true if saved successfully, false otherwise
     */
    static bool saveToFile(const std::vector<ServoConfig>& configs, const std::string& filename);

private:
    static bool parseJsonValue(const std::string& json, const std::string& key, double& value);
    static bool parseJsonValue(const std::string& json, const std::string& key, int& value);
    static bool parseJsonValue(const std::string& json, const std::string& key, uint32_t& value);
    static bool parseJsonValue(const std::string& json, const std::string& key, bool& value);
    static bool parseJsonValue(const std::string& json, const std::string& key, std::string& value);
    static std::string trimWhitespace(const std::string& str);
    static std::vector<std::string> splitJsonObjects(const std::string& json);
};