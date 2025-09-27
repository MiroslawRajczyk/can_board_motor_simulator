#include "TerminalUI.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

TerminalUI::TerminalUI(MotorController& controller, std::atomic<bool>& running)
    : controller_(controller), running_(running) {
}

void TerminalUI::printWelcome() {
    std::cout << "=======================" << std::endl;
    std::cout << "= CAN Motor Simulator =" << std::endl;
    std::cout << "=======================" << std::endl;
    std::cout << "Simulation frequency:: 10kHz" << std::endl;
}

void TerminalUI::printMotorInfo() {
    const Motor& motor = controller_.getMotor();
    const Encoder& encoder = controller_.getEncoder();

    std::cout << "\nMotor Parameters:" << std::endl;
    std::cout << "  Max Control Signal: " << motor.getMaxControlSignal() << " (range: -"
              << motor.getMaxControlSignal() << " to +" << motor.getMaxControlSignal() << ")" << std::endl;
    std::cout << "  Max Angular Velocity: " << (motor.getMaxAngularVelocity() * 60.0 / (2.0 * M_PI))
              << " RPM (" << motor.getMaxAngularVelocity() << " rad/s)" << std::endl;

    std::cout << "\nEncoder Parameters:" << std::endl;
    std::cout << "  Type: Absolute Encoder" << std::endl;
    std::cout << "  Bit Resolution: " << encoder.getBitResolution() << " bits" << std::endl;
    std::cout << "  Steps per Revolution: " << encoder.getMaxSteps() << " steps" << std::endl;
    std::cout << "  Resolution: " << (encoder.getResolutionRadians() * 180.0 / M_PI)
              << " degrees/step (" << encoder.getResolutionRadians() << " rad/step)" << std::endl;
    std::cout << "  Direction: " << (encoder.isDirectionInverted() ? "INVERTED" : "NORMAL")
              << " (positive control signal " << (encoder.isDirectionInverted() ? "decreases" : "increases")
              << " encoder value)" << std::endl;
}

void TerminalUI::printHelp() {
    std::cout << "\nAvailable commands:" << std::endl;
    std::cout << "  control <value>    - Set control signal directly (range: -1000 to +1000)" << std::endl;
    std::cout << "  position <value>   - Move to position in radians" << std::endl;
    std::cout << "  velocity <value>   - Set target velocity in rad/s" << std::endl;
    std::cout << "  stop               - Stop motor and set to idle" << std::endl;
    std::cout << "  status             - Show detailed motor status" << std::endl;
    std::cout << "  help               - Show this help message" << std::endl;
    std::cout << "  quit/exit          - Exit the motor service" << std::endl;
    std::cout << "\nExamples:" << std::endl;
    std::cout << "  control 500        - Apply control signal of 500" << std::endl;
    std::cout << "  position 3.14159   - Move to π radians (180°)" << std::endl;
    std::cout << "  velocity 10        - Spin at 10 rad/s" << std::endl;
}

void TerminalUI::printStatus() {
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
    std::cout << "Control Signal: " << motor.getControlSignal() << std::endl;
    std::cout << "Current: " << motor.getCurrent() << " A" << std::endl;
    std::cout << "Torque: " << motor.getTorque() << " Nm" << std::endl;

    if (controller_.getControlModeString() != "IDLE" && controller_.getControlModeString() != "OPEN_LOOP") {
        std::cout << "Setpoint: " << controller_.getSetpoint() << std::endl;
        std::cout << "Error: " << controller_.getCurrentError() << std::endl;
    }
    std::cout << "===================" << std::endl;
}

void TerminalUI::printPrompt() {
    std::cout << "motor> ";
    std::cout.flush();
}

void TerminalUI::processCommand(const std::string& command) {
    if (command.empty()) return;

    std::istringstream iss(command);
    std::string cmd;
    iss >> cmd;

    // Get the remaining arguments as a single string
    std::string args;
    std::getline(iss, args);
    // Remove leading whitespace from args
    if (!args.empty() && args[0] == ' ') {
        args = args.substr(1);
    }

    if (cmd == "control") {
        handleControlCommand(args);
    }
    else if (cmd == "position") {
        handlePositionCommand(args);
    }
    else if (cmd == "velocity") {
        handleVelocityCommand(args);
    }
    else if (cmd == "stop") {
        handleStopCommand();
    }
    else if (cmd == "status") {
        handleStatusCommand();
    }
    else if (cmd == "help") {
        handleHelpCommand();
    }
    else if (cmd == "quit" || cmd == "exit") {
        handleQuitCommand();
        return; // Don't print prompt after quit
    }
    else {
        handleUnknownCommand(cmd);
    }

    printPrompt();
}

void TerminalUI::handleControlCommand(const std::string& args) {
    if (args.empty()) {
        std::cout << "Usage: control <value> (range: -1000 to +1000)" << std::endl;
        return;
    }

    try {
        int control_signal = std::stoi(args);
        controller_.setControlSignal(control_signal);
        std::cout << "Set control signal to " << control_signal << " (open loop mode)" << std::endl;
    } catch (const std::exception&) {
        std::cout << "Invalid value. Usage: control <value> (range: -1000 to +1000)" << std::endl;
    }
}

void TerminalUI::handlePositionCommand(const std::string& args) {
    if (args.empty()) {
        std::cout << "Usage: position <value>" << std::endl;
        return;
    }

    try {
        double position = std::stod(args);
        controller_.setPosition(position);
        std::cout << "Moving to position " << position << " radians" << std::endl;
    } catch (const std::exception&) {
        std::cout << "Invalid value. Usage: position <value>" << std::endl;
    }
}

void TerminalUI::handleVelocityCommand(const std::string& args) {
    if (args.empty()) {
        std::cout << "Usage: velocity <value>" << std::endl;
        return;
    }

    try {
        double velocity = std::stod(args);
        controller_.setVelocity(velocity);
        std::cout << "Setting velocity to " << velocity << " rad/s" << std::endl;
    } catch (const std::exception&) {
        std::cout << "Invalid value. Usage: velocity <value>" << std::endl;
    }
}

void TerminalUI::handleStopCommand() {
    controller_.stop();
    std::cout << "Motor stopped" << std::endl;
}

void TerminalUI::handleStatusCommand() {
    printStatus();
}

void TerminalUI::handleHelpCommand() {
    printHelp();
}

void TerminalUI::handleQuitCommand() {
    running_ = false;
    std::cout << "Shutting down motor service..." << std::endl;
}

void TerminalUI::handleUnknownCommand(const std::string& cmd) {
    std::cout << "Unknown command: " << cmd << std::endl;
    std::cout << "Type 'help' for available commands" << std::endl;
}
