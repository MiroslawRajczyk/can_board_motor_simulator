#ifndef TERMINAL_UI_H
#define TERMINAL_UI_H

#include "MotorController.h"
#include <string>
#include <atomic>

class TerminalUI {
private:
    MotorController& controller_;
    std::atomic<bool>& running_;

public:
    TerminalUI(MotorController& controller, std::atomic<bool>& running);

    void printWelcome();
    void printMotorInfo();
    void printHelp();
    void printStatus();
    void printPrompt();

    void processCommand(const std::string& command);

private:
    void handleControlCommand(const std::string& args);
    void handlePositionCommand(const std::string& args);
    void handleVelocityCommand(const std::string& args);
    void handleStopCommand();
    void handleStatusCommand();
    void handleHelpCommand();
    void handleQuitCommand();
    void handleUnknownCommand(const std::string& cmd);
};

#endif // TERMINAL_UI_H
