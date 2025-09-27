#include "Servo.h"
#include "TerminalUI.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <atomic>

class MotorService {
private:
    Servo servo_;
    std::atomic<bool> running_;
    static constexpr double simulationFrequencyHz_ = 20000.0;

public:
    MotorService() :
        servo_(Servo::builder()
               .maxVelocityRPM(160.0)
               .maxControlSignal(1000)
               .timeConstant(0.3)
               .encoderBitResolution(18)
               .encoderDirectionInverted(false)),
        running_(false) { }

    void start() {
        running_ = true;
    }

    void stop() {
        running_ = false;
        servo_.stop();
    }

    void update() {
        constexpr double dt = 1.0 / simulationFrequencyHz_;
        servo_.update(dt);
    }

    bool isRunning() const {
        return running_;
    }

    std::atomic<bool>& getRunningRef() {
        return running_;
    }

    double getSimulationFrequency() const {
        return simulationFrequencyHz_;
    }

    Motor& getMotor() {
        return servo_.getMotor();
    }

    Encoder& getEncoder() {
        return servo_.getEncoder();
    }

    void simulationLoop() {
        auto next_update = std::chrono::steady_clock::now();
        constexpr auto update_interval = std::chrono::microseconds(static_cast<long>(1000000.0 / simulationFrequencyHz_));

        while (running_) {
            update();
            next_update += update_interval;
            std::this_thread::sleep_until(next_update);
        }
    }
};

int main() {
    MotorService service;
    TerminalUI ui(service.getMotor(), service.getEncoder(), service.getRunningRef(), service.getSimulationFrequency());

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