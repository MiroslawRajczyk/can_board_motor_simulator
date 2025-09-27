#include "Motor.h"
#include "Encoder.h"
#include "TerminalUI.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <atomic>

class MotorService {
private:
    Motor motor_;
    Encoder encoder_;
    std::atomic<bool> running_;
    const double dt_ = 0.0001; // 0,1ms update cycle (10kHz)

public:
    MotorService() :
        motor_(Motor::builder()
               .maxVelocityRPM(160.0)
               .maxControlSignal(1000)
               .timeConstant(0.3)),
        encoder_(Encoder::builder()
                .bitResolution(18)
                .directionInverted(false)),
        running_(false) { }

    void start() {
        running_ = true;
    }

    void stop() {
        running_ = false;
        motor_.setControlSignal(0);
    }

    void update() {
        motor_.update(dt_);
        encoder_.update(motor_.getAngularVelocity(), dt_);
    }

    bool isRunning() const {
        return running_;
    }

    std::atomic<bool>& getRunningRef() {
        return running_;
    }

    Motor& getMotor() {
        return motor_;
    }

    Encoder& getEncoder() {
        return encoder_;
    }

    void simulationLoop() {
        auto next_update = std::chrono::steady_clock::now();

        while (running_) {
            update();
            next_update += std::chrono::microseconds(100);
            std::this_thread::sleep_until(next_update);
        }
    }
};

int main() {
    MotorService service;
    TerminalUI ui(service.getMotor(), service.getEncoder(), service.getRunningRef());

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