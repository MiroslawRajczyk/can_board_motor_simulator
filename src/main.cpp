#include "MotorController.h"
#include "TerminalUI.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <atomic>

class MotorService {
private:
    MotorController controller_;
    std::atomic<bool> running_;
    std::chrono::steady_clock::time_point last_update_;
    const double dt_ = 0.0001; // 0,1ms update cycle (10kHz)

public:
    MotorService() : running_(false) {
        controller_.setPIDGains(5.0, 0.5, 0.1);
    }

    void start() {
        running_ = true;
        last_update_ = std::chrono::steady_clock::now();
    }

    void stop() {
        running_ = false;
        controller_.stop();
    }

    void update() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_);

        if (elapsed.count() >= 100) { // 0.1ms update cycle (100 microseconds) for 10kHz
            controller_.update(dt_);
            last_update_ = now;
        }
    }

    bool isRunning() const {
        return running_;
    }

    std::atomic<bool>& getRunningRef() {
        return running_;
    }

    MotorController& getController() {
        return controller_;
    }

    void simulationLoop() {
        while (running_) {
            update();
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
};

int main() {
    MotorService service;
    TerminalUI ui(service.getController(), service.getRunningRef());

    // Print welcome message and initial info
    ui.printWelcome();

    service.start();

    ui.printMotorInfo();
    ui.printHelp();
    ui.printPrompt();

    // Start the simulation loop in a separate thread
    std::thread simulationThread(&MotorService::simulationLoop, &service);

    // Main service loop - process user commands
    std::string command;
    while (service.isRunning() && std::getline(std::cin, command)) {
        ui.processCommand(command);
    }

    // Stop the service and wait for the simulation thread to finish
    service.stop();
    simulationThread.join();

    std::cout << "Motor service stopped." << std::endl;
    return 0;
}