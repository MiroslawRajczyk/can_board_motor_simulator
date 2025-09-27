#include "TerminalUI.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

TerminalUI::TerminalUI(Motor& motor, Encoder& encoder, std::atomic<bool>& running)
    : motor_(motor), encoder_(encoder), running_(running) {
}

void TerminalUI::printWelcome() {
    std::cout << "=======================" << std::endl;
    std::cout << "= CAN Motor Simulator =" << std::endl;
    std::cout << "=======================" << std::endl;
    std::cout << "Simulation frequency:: 10kHz" << std::endl;
}

void TerminalUI::printMotorInfo() {
    std::cout << "\nMotor Parameters:" << std::endl;
    std::cout << "  Max Control Signal: " << motor_.getMaxControlSignal() << " (range: -"
              << motor_.getMaxControlSignal() << " to +" << motor_.getMaxControlSignal() << ")" << std::endl;
    std::cout << "  Max Angular Velocity: " << (motor_.getMaxAngularVelocity() * 60.0 / (2.0 * M_PI))
              << " RPM (" << motor_.getMaxAngularVelocity() << " rad/s)" << std::endl;
    std::cout << "  Motor Time Constant: " << motor_.getMotorTimeConstant() << " seconds" << std::endl;

    std::cout << "\nEncoder Parameters:" << std::endl;
    std::cout << "  Type: Absolute Encoder" << std::endl;
    std::cout << "  Bit Resolution: " << encoder_.getBitResolution() << " bits" << std::endl;
    std::cout << "  Steps per Revolution: " << encoder_.getMaxSteps() << " steps" << std::endl;
    std::cout << "  Resolution: " << (encoder_.getResolutionRadians() * 180.0 / M_PI)
              << " degrees/step (" << encoder_.getResolutionRadians() << " rad/step)" << std::endl;
    std::cout << "  Direction: " << (encoder_.isDirectionInverted() ? "INVERTED" : "NORMAL")
              << " (positive control signal " << (encoder_.isDirectionInverted() ? "decreases" : "increases")
              << " encoder value)" << std::endl;
}

void TerminalUI::printHelp() {
    std::cout << "\nAvailable commands:" << std::endl;
    std::cout << "  control <value>    - Set control signal directly (range: -1000 to +1000)" << std::endl;
    std::cout << "  stop               - Stop motor and set to idle" << std::endl;
    std::cout << "  status             - Show detailed motor status" << std::endl;
    std::cout << "  help               - Show this help message" << std::endl;
    std::cout << "  quit/exit          - Exit the motor service" << std::endl;
    std::cout << "\nExamples:" << std::endl;
    std::cout << "  control 500        - Apply control signal of 500" << std::endl;
    std::cout << "  stop               - Stop the motor" << std::endl;
}

void TerminalUI::printStatus() {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n====== Motor Status ======" << std::endl;
    std::cout << "Position: " << encoder_.getPositionSteps() << " steps ("
              << encoder_.getPositionRadians() << " rad, "
              << (encoder_.getPositionRadians() * 180.0 / M_PI) << "°)" << std::endl;

    // Convert rad/s to RPM: rad/s * (60 / 2π) = RPM
    double velocity_rad_s = motor_.getAngularVelocity();
    double velocity_rpm = velocity_rad_s * (60.0 / (2.0 * M_PI));
    std::cout << "Velocity: " << velocity_rpm << " RPM (" << velocity_rad_s << " rad/s)" << std::endl;

    std::cout << "Control Signal: " << motor_.getControlSignal() << std::endl;
    std::cout << "==========================" << std::endl;
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
        motor_.setControlSignal(control_signal);
        std::cout << "Set control signal to " << control_signal << std::endl;
    } catch (const std::exception&) {
        std::cout << "Invalid value. Usage: control <value> (range: -1000 to +1000)" << std::endl;
    }
}

void TerminalUI::handleStopCommand() {
    motor_.setControlSignal(0);
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
