#include "SimulationEngine.h"
#include <stdexcept>

SimulationEngine::SimulationEngine() : running_(false) {}

SimulationEngine::~SimulationEngine() {
    stop();
}

void SimulationEngine::addServo(Servo&& servo) {
    servos_.emplace_back(std::move(servo));
}

size_t SimulationEngine::getServoCount() const {
    return servos_.size();
}

Servo& SimulationEngine::getServo(size_t index) {
    if (index >= servos_.size()) {
        throw std::out_of_range("Servo index out of range");
    }
    return servos_[index];
}

const Servo& SimulationEngine::getServo(size_t index) const {
    if (index >= servos_.size()) {
        throw std::out_of_range("Servo index out of range");
    }
    return servos_[index];
}

void SimulationEngine::start() {
    running_ = true;
    simulationThread_ = std::thread(&SimulationEngine::simulationLoop, this);
}

void SimulationEngine::stop() {
    running_ = false;
    for (auto& servo : servos_) {
        servo.stop();
    }
    if (simulationThread_.joinable()) {
        simulationThread_.join();
    }
}

void SimulationEngine::update() {
    constexpr double dt = 1.0 / simulationFrequencyHz_;
    for (auto& servo : servos_) {
        servo.update(dt);
    }
}

bool SimulationEngine::isRunning() const {
    return running_;
}

std::atomic<bool>& SimulationEngine::getRunningRef() {
    return running_;
}

double SimulationEngine::getSimulationFrequency() const {
    return simulationFrequencyHz_;
}

Motor& SimulationEngine::getMotor(size_t index) {
    return getServo(index).getMotor();
}

Encoder& SimulationEngine::getEncoder(size_t index) {
    return getServo(index).getEncoder();
}

void SimulationEngine::simulationLoop() {
    auto next_update = std::chrono::steady_clock::now();
    constexpr auto update_interval = std::chrono::microseconds(static_cast<long>(1000000.0 / simulationFrequencyHz_));

    while (running_) {
        update();
        next_update += update_interval;
        std::this_thread::sleep_until(next_update);
    }
}