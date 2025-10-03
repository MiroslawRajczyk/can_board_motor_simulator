#include "SimulationEngine.h"
#include "ConfigLoader.h"
#include <iostream>
#include <string>

int main() {
    SimulationEngine simulation;

    // Load servo configurations from JSON file
    std::cout << "Loading servo configurations from servos.json..." << std::endl;
    auto servos = ConfigLoader::loadServosFromFile("servos.json");
    
    if (servos.empty()) {
        std::cerr << "No servos loaded! Check servos.json file." << std::endl;
        return 1;
    }
    
    // Add all loaded servos to simulation
    for (auto& servo : servos) {
        simulation.addServo(std::move(servo));
    }

    std::cout << "Starting simulation with " << simulation.getServoCount() << " servos..." << std::endl;
    simulation.start();

    // Start CAN communication for all servos
    for (size_t i = 0; i < simulation.getServoCount(); ++i) {
        simulation.getServo(i).startCAN();
    }

    // Wait for program termination (e.g., Ctrl+C)
    std::cout << "Press Enter to stop the simulation..." << std::endl;
    std::cin.get();

    // Stop CAN communication for all servos
    for (size_t i = 0; i < simulation.getServoCount(); ++i) {
        simulation.getServo(i).stopCAN();
    }

    simulation.stop();
    return 0;
}
