#include "TerminalUI.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

TerminalUI::TerminalUI(SimulationEngine& simulation, std::atomic<bool>& running, double simulationFrequencyHz)
    : simulation_(simulation), running_(running), simulationFrequencyHz_(simulationFrequencyHz), currentServoIndex_(0) {
    printWelcome();
    printMotorInfo();
    printHelp();
    printPrompt();
}

void TerminalUI::printWelcome() {
    std::cout << "=======================" << std::endl;
    std::cout << "= CAN Motor Simulator =" << std::endl;
    std::cout << "=======================" << std::endl;
    std::cout << "Simulation frequency: " << simulationFrequencyHz_ << " Hz" << std::endl;
    std::cout << "Number of servos: " << simulation_.getServoCount() << std::endl;
    std::cout << "Current servo: " << currentServoIndex_ << std::endl;
}

void TerminalUI::printMotorInfo() {
    if (simulation_.getServoCount() == 0) {
        std::cout << "\nNo servos available!" << std::endl;
        return;
    }

    Motor& motor = simulation_.getMotor(currentServoIndex_);
    Encoder& encoder = simulation_.getEncoder(currentServoIndex_);

    std::cout << "\nServo " << currentServoIndex_ << " Parameters:" << std::endl;
    std::cout << "Motor Parameters:" << std::endl;
    std::cout << "  Max Control Signal: " << motor.getMaxControlSignal() << " (range: -"
              << motor.getMaxControlSignal() << " to +" << motor.getMaxControlSignal() << ")" << std::endl;
    std::cout << "  Max Angular Velocity: " << (motor.getMaxAngularVelocity() * 60.0 / (2.0 * M_PI))
              << " RPM (" << motor.getMaxAngularVelocity() << " rad/s)" << std::endl;
    std::cout << "  Motor Time Constant: " << motor.getMotorTimeConstant() << " seconds" << std::endl;

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
    std::cout << "  control <value>    - Set control signal for current servo (range: -1000 to +1000)" << std::endl;
    std::cout << "  stop               - Stop current servo" << std::endl;
    std::cout << "  stopall            - Stop all servos" << std::endl;
    std::cout << "  status             - Show current servo status" << std::endl;
    std::cout << "  statusall          - Show status of all servos" << std::endl;
    std::cout << "  select <index>     - Select servo to control (0-based index)" << std::endl;
    std::cout << "  list               - List all servos" << std::endl;
    std::cout << "  help               - Show this help message" << std::endl;
    std::cout << "  quit/exit          - Exit the motor service" << std::endl;
    std::cout << "\nExamples:" << std::endl;
    std::cout << "  control 500        - Apply control signal of 500 to current servo" << std::endl;
    std::cout << "  select 1           - Switch to controlling servo 1" << std::endl;
    std::cout << "  stopall            - Stop all servos" << std::endl;
}

void TerminalUI::printStatus() {
    if (simulation_.getServoCount() == 0) {
        std::cout << "\nNo servos available!" << std::endl;
        return;
    }

    Motor& motor = simulation_.getMotor(currentServoIndex_);
    Encoder& encoder = simulation_.getEncoder(currentServoIndex_);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n====== Servo " << currentServoIndex_ << " Status ======" << std::endl;
    std::cout << "Position: " << encoder.getPositionSteps() << " steps ("
              << encoder.getPositionRadians() << " rad, "
              << (encoder.getPositionRadians() * 180.0 / M_PI) << "°)" << std::endl;

    // Convert rad/s to RPM: rad/s * (60 / 2π) = RPM
    double velocity_rad_s = motor.getAngularVelocity();
    double velocity_rpm = velocity_rad_s * (60.0 / (2.0 * M_PI));
    std::cout << "Velocity: " << velocity_rpm << " RPM (" << velocity_rad_s << " rad/s)" << std::endl;

    std::cout << "Control Signal: " << motor.getControlSignal() << std::endl;
    std::cout << "==========================" << std::endl;
}

void TerminalUI::printPrompt() {
    std::cout << "servo[" << currentServoIndex_ << "]> ";
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
    else if (cmd == "stopall") {
        handleStopAllCommand();
    }
    else if (cmd == "status") {
        handleStatusCommand();
    }
    else if (cmd == "statusall") {
        handleStatusAllCommand();
    }
    else if (cmd == "select") {
        handleSelectServoCommand(args);
    }
    else if (cmd == "list") {
        handleListServosCommand();
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
    if (simulation_.getServoCount() == 0) {
        std::cout << "No servos available!" << std::endl;
        return;
    }

    if (args.empty()) {
        std::cout << "Usage: control <value> (range: -1000 to +1000)" << std::endl;
        return;
    }

    try {
        int control_signal = std::stoi(args);
        simulation_.getMotor(currentServoIndex_).setControlSignal(control_signal);
        std::cout << "Set control signal to " << control_signal << " for servo " << currentServoIndex_ << std::endl;
    } catch (const std::exception&) {
        std::cout << "Invalid value. Usage: control <value> (range: -1000 to +1000)" << std::endl;
    }
}

void TerminalUI::handleStopCommand() {
    if (simulation_.getServoCount() == 0) {
        std::cout << "No servos available!" << std::endl;
        return;
    }

    simulation_.getMotor(currentServoIndex_).setControlSignal(0);
    std::cout << "Servo " << currentServoIndex_ << " stopped" << std::endl;
}

void TerminalUI::handleStopAllCommand() {
    if (simulation_.getServoCount() == 0) {
        std::cout << "No servos available!" << std::endl;
        return;
    }

    for (size_t i = 0; i < simulation_.getServoCount(); ++i) {
        simulation_.getMotor(i).setControlSignal(0);
    }
    std::cout << "All servos stopped" << std::endl;
}

void TerminalUI::handleStatusCommand() {
    printStatus();
}

void TerminalUI::handleStatusAllCommand() {
    if (simulation_.getServoCount() == 0) {
        std::cout << "\nNo servos available!" << std::endl;
        return;
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n====== All Servos Status ======" << std::endl;

    for (size_t i = 0; i < simulation_.getServoCount(); ++i) {
        Motor& motor = simulation_.getMotor(i);
        Encoder& encoder = simulation_.getEncoder(i);

        std::cout << "Servo " << i << ":" << std::endl;
        std::cout << "  Position: " << encoder.getPositionSteps() << " steps ("
                  << encoder.getPositionRadians() << " rad, "
                  << (encoder.getPositionRadians() * 180.0 / M_PI) << "°)" << std::endl;

        double velocity_rad_s = motor.getAngularVelocity();
        double velocity_rpm = velocity_rad_s * (60.0 / (2.0 * M_PI));
        std::cout << "  Velocity: " << velocity_rpm << " RPM (" << velocity_rad_s << " rad/s)" << std::endl;
        std::cout << "  Control Signal: " << motor.getControlSignal() << std::endl;

        if (i < simulation_.getServoCount() - 1) {
            std::cout << std::endl;
        }
    }
    std::cout << "===============================" << std::endl;
}

void TerminalUI::handleSelectServoCommand(const std::string& args) {
    if (args.empty()) {
        std::cout << "Usage: select <index> (0-" << (simulation_.getServoCount() - 1) << ")" << std::endl;
        return;
    }

    try {
        size_t index = std::stoul(args);
        if (index >= simulation_.getServoCount()) {
            std::cout << "Invalid servo index. Available servos: 0-" << (simulation_.getServoCount() - 1) << std::endl;
            return;
        }

        currentServoIndex_ = index;
        std::cout << "Selected servo " << currentServoIndex_ << std::endl;
        printMotorInfo();
    } catch (const std::exception&) {
        std::cout << "Invalid index. Usage: select <index> (0-" << (simulation_.getServoCount() - 1) << ")" << std::endl;
    }
}

void TerminalUI::handleListServosCommand() {
    if (simulation_.getServoCount() == 0) {
        std::cout << "\nNo servos available!" << std::endl;
        return;
    }

    std::cout << "\nAvailable servos:" << std::endl;
    for (size_t i = 0; i < simulation_.getServoCount(); ++i) {
        std::cout << "  " << i << (i == currentServoIndex_ ? " (current)" : "") << std::endl;
    }
    std::cout << "Total: " << simulation_.getServoCount() << " servo(s)" << std::endl;
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
