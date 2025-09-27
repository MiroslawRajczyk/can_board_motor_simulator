#pragma once

#include "Motor.h"
#include "Encoder.h"
#include <string>
#include <atomic>

class TerminalUI {
private:
    Motor& motor_;
    Encoder& encoder_;
    std::atomic<bool>& running_;
    double simulationFrequencyHz_;

public:
    TerminalUI(Motor& motor, Encoder& encoder, std::atomic<bool>& running, double simulationFrequencyHz);
    void printWelcome();
    void printMotorInfo();
    void printHelp();
    void printStatus();
    void printPrompt();

    void processCommand(const std::string& command);

private:
    void handleControlCommand(const std::string& args);
    void handleStopCommand();
    void handleStatusCommand();
    void handleHelpCommand();
    void handleQuitCommand();
    void handleUnknownCommand(const std::string& cmd);
};
