#pragma once

#include "Servo.h"
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>

class SimulationEngine {
private:
    std::vector<Servo> servos_;
    std::atomic<bool> running_;
    std::thread simulationThread_;
    static constexpr double simulationFrequencyHz_ = 20000.0;

public:
    SimulationEngine();
    ~SimulationEngine();

    void addServo(Servo&& servo);

    size_t getServoCount() const;

    Servo& getServo(size_t index = 0);
    const Servo& getServo(size_t index = 0) const;

    void start();
    void stop();
    void update();

    bool isRunning() const;
    std::atomic<bool>& getRunningRef();
    double getSimulationFrequency() const;

    Motor& getMotor(size_t index = 0);
    Encoder& getEncoder(size_t index = 0);

private:
    void simulationLoop();
};