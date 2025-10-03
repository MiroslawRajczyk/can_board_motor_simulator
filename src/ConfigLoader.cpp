#include "ConfigLoader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>

std::vector<ServoConfig> ConfigLoader::loadFromFile(const std::string& filename) {
    std::vector<ServoConfig> configs;
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ConfigLoader: Cannot open file: " << filename << std::endl;
        return configs;
    }
    
    std::string json_content;
    std::string line;
    while (std::getline(file, line)) {
        json_content += line + "\n";
    }
    file.close();
    
    // Parse JSON array of servo objects
    std::vector<std::string> servo_objects = splitJsonObjects(json_content);
    
    for (const auto& servo_json : servo_objects) {
        ServoConfig config;
        
        parseJsonValue(servo_json, "name", config.name);
        parseJsonValue(servo_json, "maxVelocityRPM", config.maxVelocityRPM);
        parseJsonValue(servo_json, "maxControlSignal", config.maxControlSignal);
        parseJsonValue(servo_json, "timeConstant", config.timeConstant);
        parseJsonValue(servo_json, "encoderBitResolution", config.encoderBitResolution);
        parseJsonValue(servo_json, "encoderDirectionInverted", config.encoderDirectionInverted);
        parseJsonValue(servo_json, "canId", config.canId);
        parseJsonValue(servo_json, "canInterface", config.canInterface);
        
        configs.push_back(config);
        std::cout << "ConfigLoader: Loaded servo '" << config.name << "' with CAN ID 0x" 
                  << std::hex << config.canId << std::dec << std::endl;
    }
    
    std::cout << "ConfigLoader: Loaded " << configs.size() << " servo configurations" << std::endl;
    return configs;
}

std::vector<Servo> ConfigLoader::createServos(const std::vector<ServoConfig>& configs) {
    std::vector<Servo> servos;
    
    for (const auto& config : configs) {
        auto servo = Servo::builder()
            .maxVelocityRPM(config.maxVelocityRPM)
            .maxControlSignal(config.maxControlSignal)
            .timeConstant(config.timeConstant)
            .encoderBitResolution(config.encoderBitResolution)
            .encoderDirectionInverted(config.encoderDirectionInverted)
            .canId(config.canId)
            .canInterface(config.canInterface)
            .build();
            
        servos.push_back(std::move(servo));
    }
    
    return servos;
}

std::vector<Servo> ConfigLoader::loadServosFromFile(const std::string& filename) {
    auto configs = loadFromFile(filename);
    return createServos(configs);
}

bool ConfigLoader::saveToFile(const std::vector<ServoConfig>& configs, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ConfigLoader: Cannot create file: " << filename << std::endl;
        return false;
    }
    
    file << "[\n";
    for (size_t i = 0; i < configs.size(); ++i) {
        const auto& config = configs[i];
        file << "  {\n";
        file << "    \"name\": \"" << config.name << "\",\n";
        file << "    \"maxVelocityRPM\": " << config.maxVelocityRPM << ",\n";
        file << "    \"maxControlSignal\": " << config.maxControlSignal << ",\n";
        file << "    \"timeConstant\": " << config.timeConstant << ",\n";
        file << "    \"encoderBitResolution\": " << config.encoderBitResolution << ",\n";
        file << "    \"encoderDirectionInverted\": " << (config.encoderDirectionInverted ? "true" : "false") << ",\n";
        file << "    \"canId\": " << config.canId << ",\n";
        file << "    \"canInterface\": \"" << config.canInterface << "\"\n";
        file << "  }";
        if (i < configs.size() - 1) {
            file << ",";
        }
        file << "\n";
    }
    file << "]\n";
    
    file.close();
    return true;
}

bool ConfigLoader::parseJsonValue(const std::string& json, const std::string& key, double& value) {
    std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return false;
    
    pos = json.find(':', pos);
    if (pos == std::string::npos) return false;
    
    pos = json.find_first_not_of(" \t", pos + 1);
    if (pos == std::string::npos) return false;
    
    size_t end = json.find_first_of(",}\n", pos);
    if (end == std::string::npos) return false;
    
    std::string value_str = trimWhitespace(json.substr(pos, end - pos));
    try {
        value = std::stod(value_str);
        return true;
    } catch (...) {
        return false;
    }
}

bool ConfigLoader::parseJsonValue(const std::string& json, const std::string& key, int& value) {
    double temp;
    if (parseJsonValue(json, key, temp)) {
        value = static_cast<int>(temp);
        return true;
    }
    return false;
}

bool ConfigLoader::parseJsonValue(const std::string& json, const std::string& key, uint32_t& value) {
    double temp;
    if (parseJsonValue(json, key, temp)) {
        value = static_cast<uint32_t>(temp);
        return true;
    }
    return false;
}

bool ConfigLoader::parseJsonValue(const std::string& json, const std::string& key, bool& value) {
    std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return false;
    
    pos = json.find(':', pos);
    if (pos == std::string::npos) return false;
    
    pos = json.find_first_not_of(" \t", pos + 1);
    if (pos == std::string::npos) return false;
    
    size_t end = json.find_first_of(",}\n", pos);
    if (end == std::string::npos) return false;
    
    std::string value_str = trimWhitespace(json.substr(pos, end - pos));
    if (value_str == "true") {
        value = true;
        return true;
    } else if (value_str == "false") {
        value = false;
        return true;
    }
    return false;
}

bool ConfigLoader::parseJsonValue(const std::string& json, const std::string& key, std::string& value) {
    std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return false;
    
    pos = json.find(':', pos);
    if (pos == std::string::npos) return false;
    
    pos = json.find('"', pos + 1);
    if (pos == std::string::npos) return false;
    
    size_t end = json.find('"', pos + 1);
    if (end == std::string::npos) return false;
    
    value = json.substr(pos + 1, end - pos - 1);
    return true;
}

std::string ConfigLoader::trimWhitespace(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

std::vector<std::string> ConfigLoader::splitJsonObjects(const std::string& json) {
    std::vector<std::string> objects;
    
    size_t start = json.find('[');
    if (start == std::string::npos) return objects;
    
    size_t pos = start + 1;
    int brace_count = 0;
    size_t object_start = std::string::npos;
    
    while (pos < json.length()) {
        char c = json[pos];
        
        if (c == '{') {
            if (brace_count == 0) {
                object_start = pos;
            }
            brace_count++;
        } else if (c == '}') {
            brace_count--;
            if (brace_count == 0 && object_start != std::string::npos) {
                objects.push_back(json.substr(object_start, pos - object_start + 1));
                object_start = std::string::npos;
            }
        }
        pos++;
    }
    
    return objects;
}