#pragma once

#include "SimulationEngine.h"
#include <string>
#include <atomic>

class TerminalUI {
private:
    SimulationEngine& simulation_;
    std::atomic<bool>& running_;
    double simulationFrequencyHz_;
    size_t currentServoIndex_;

public:
    TerminalUI(SimulationEngine& simulation, std::atomic<bool>& running, double simulationFrequencyHz);
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
    void handleSelectServoCommand(const std::string& args);
    void handleListServosCommand();
    void handleStatusAllCommand();
    void handleStopAllCommand();
};
