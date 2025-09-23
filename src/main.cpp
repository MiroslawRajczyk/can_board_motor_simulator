#include "MotorController.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <sstream>
#include <atomic>

class MotorService {
private:
    MotorController controller_;
    std::atomic<bool> running_;
    std::chrono::steady_clock::time_point last_update_;
    const double dt_ = 0.0001; // 1ms update cycle (1kHz)
    
public:
    MotorService() : running_(false) {
        // Set default PID gains
        controller_.setPIDGains(5.0, 0.5, 0.1);
    }
    
    void start() {
        running_ = true;
        last_update_ = std::chrono::steady_clock::now();
        
        printMotorInfo();
        printHelp();
        std::cout << "motor> ";
        std::cout.flush();
    }
    
    void stop() {
        running_ = false;
        controller_.stop();
    }
    
    void update() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_);
        
        if (elapsed.count() >= 100) { // 0.1ms update cycle (100 microseconds)
            controller_.update(dt_);
            last_update_ = now;
        }
    }
    
    bool isRunning() const {
        return running_;
    }
    
    // Run the simulation loop - designed to be called from a separate thread
    void simulationLoop() {
        while (running_) {
            update();
            std::this_thread::sleep_for(std::chrono::microseconds(100)); // 0.1ms sleep
        }
    }

    void processCommand(const std::string& command) {
        if (command.empty()) return;
        
        std::istringstream iss(command);
        std::string cmd;
        iss >> cmd;
        
        if (cmd == "voltage") {
            double voltage;
            if (iss >> voltage) {
                controller_.setVoltage(voltage);
                std::cout << "Set voltage to " << voltage << "V (open loop mode)" << std::endl;
            } else {
                std::cout << "Usage: voltage <value>" << std::endl;
            }
        }
        else if (cmd == "position") {
            double position;
            if (iss >> position) {
                controller_.setPosition(position);
                std::cout << "Moving to position " << position << " radians" << std::endl;
            } else {
                std::cout << "Usage: position <value>" << std::endl;
            }
        }
        else if (cmd == "velocity") {
            double velocity;
            if (iss >> velocity) {
                controller_.setVelocity(velocity);
                std::cout << "Setting velocity to " << velocity << " rad/s" << std::endl;
            } else {
                std::cout << "Usage: velocity <value>" << std::endl;
            }
        }
        else if (cmd == "stop") {
            controller_.stop();
            std::cout << "Motor stopped" << std::endl;
        }
        else if (cmd == "status") {
            printStatus();
        }
        else if (cmd == "help") {
            printHelp();
        }
        else if (cmd == "quit" || cmd == "exit") {
            running_ = false;
            std::cout << "Shutting down motor service..." << std::endl;
            return;
        }
        else {
            std::cout << "Unknown command: " << cmd << std::endl;
            std::cout << "Type 'help' for available commands" << std::endl;
        }
        
        std::cout << "motor> ";
        std::cout.flush();
    }
    
private:
    void printMotorInfo() {
        const Motor& motor = controller_.getMotor();
        const Encoder& encoder = controller_.getEncoder();

        std::cout << "\nMotor Parameters:" << std::endl;
        std::cout << "  Max Voltage: " << motor.getMaxVoltage() << " V" << std::endl;
        std::cout << "  Max Angular Velocity: " << (motor.getMaxAngularVelocity() * 60.0 / (2.0 * M_PI)) 
                  << " RPM (" << motor.getMaxAngularVelocity() << " rad/s)" << std::endl;
        
        std::cout << "\nEncoder Parameters:" << std::endl;
        std::cout << "  Type: Absolute Encoder" << std::endl;
        std::cout << "  Bit Resolution: " << encoder.getBitResolution() << " bits" << std::endl;
        std::cout << "  Steps per Revolution: " << encoder.getMaxSteps() << " steps" << std::endl;
        std::cout << "  Resolution: " << (encoder.getResolutionRadians() * 180.0 / M_PI) 
                  << " degrees/step (" << encoder.getResolutionRadians() << " rad/step)" << std::endl;
        std::cout << "  Direction: " << (encoder.isDirectionInverted() ? "INVERTED" : "NORMAL") 
                  << " (positive voltage " << (encoder.isDirectionInverted() ? "decreases" : "increases") 
                  << " encoder value)" << std::endl;
    }
    
    void printStatus() {
        const Motor& motor = controller_.getMotor();
        const Encoder& encoder = controller_.getEncoder();

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "\n=== Motor Status ===" << std::endl;
        std::cout << "Mode: " << controller_.getControlModeString() << std::endl;
        std::cout << "Running: " << (controller_.isRunning() ? "YES" : "NO") << std::endl;
        std::cout << "Position: " << encoder.getPositionSteps() << " steps (" 
                  << encoder.getPositionRadians() << " rad, " 
                  << (encoder.getPositionRadians() * 180.0 / M_PI) << "°)" << std::endl;
        std::cout << "Velocity: " << encoder.getVelocity() << " rad/s" << std::endl;
        std::cout << "Voltage: " << motor.getVoltage() << " V" << std::endl;
        std::cout << "Current: " << motor.getCurrent() << " A" << std::endl;
        std::cout << "Torque: " << motor.getTorque() << " Nm" << std::endl;
        
        if (controller_.getControlModeString() != "IDLE" && controller_.getControlModeString() != "OPEN_LOOP") {
            std::cout << "Setpoint: " << controller_.getSetpoint() << std::endl;
            std::cout << "Error: " << controller_.getCurrentError() << std::endl;
        }
        std::cout << "===================" << std::endl;
    }
    
    void printHelp() {
        std::cout << "\nAvailable commands:" << std::endl;
        std::cout << "  voltage <value>    - Set motor voltage directly (V)" << std::endl;
        std::cout << "  position <value>   - Move to position in radians" << std::endl;
        std::cout << "  velocity <value>   - Set target velocity in rad/s" << std::endl;
        std::cout << "  stop               - Stop motor and set to idle" << std::endl;
        std::cout << "  status             - Show detailed motor status" << std::endl;
        std::cout << "  help               - Show this help message" << std::endl;
        std::cout << "  quit/exit          - Exit the motor service" << std::endl;
        std::cout << "\nExamples:" << std::endl;
        std::cout << "  voltage 6.0        - Apply 6V to motor" << std::endl;
        std::cout << "  position 3.14159   - Move to π radians (180°)" << std::endl;
        std::cout << "  velocity 10        - Spin at 10 rad/s" << std::endl;
    }
};

int main() {
    std::cout << "=======================" << std::endl;
    std::cout << "= CAN Motor Simulator =" << std::endl;
    std::cout << "=======================" << std::endl;
    std::cout << "Simulation frequency:: 10kHz" << std::endl;
    
    MotorService service;
    service.start();
    
    // Start the simulation loop in a separate thread
    std::thread simulationThread(&MotorService::simulationLoop, &service);
    
    // Main service loop - process user commands
    std::string command;
    while (service.isRunning() && std::getline(std::cin, command)) {
        service.processCommand(command);
    }
    
    // Stop the service and wait for the simulation thread to finish
    service.stop();
    simulationThread.join();
    
    std::cout << "Motor service stopped." << std::endl;
    return 0;
}