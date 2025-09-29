#include "Servo.h"
#include "TerminalUI.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <atomic>

class SimulationEngine {
private:
    Servo servo_;
    std::atomic<bool> running_;
    std::thread simulationThread_;
    static constexpr double simulationFrequencyHz_ = 20000.0;

public:
    SimulationEngine() :
        servo_(Servo::builder()
               .maxVelocityRPM(160.0)
               .maxControlSignal(1000)
               .timeConstant(0.3)
               .encoderBitResolution(18)
               .encoderDirectionInverted(false)),
        running_(false) { }

    ~SimulationEngine() {
        stop();
    }

    void start() {
        running_ = true;
        simulationThread_ = std::thread(&SimulationEngine::simulationLoop, this);
    }

    void stop() {
        running_ = false;
        servo_.stop();
        if (simulationThread_.joinable()) {
            simulationThread_.join();
        }
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
    SimulationEngine simulation;
    TerminalUI ui(simulation.getMotor(), simulation.getEncoder(), simulation.getRunningRef(), simulation.getSimulationFrequency());

    ui.printWelcome();

    simulation.start();

    ui.printMotorInfo();
    ui.printHelp();
    ui.printPrompt();

    // Process user commands
    std::string command;
    while (simulation.isRunning() && std::getline(std::cin, command)) {
        ui.processCommand(command);
    }

    simulation.stop();

    std::cout << "Simulation stopped." << std::endl;
    return 0;
}
