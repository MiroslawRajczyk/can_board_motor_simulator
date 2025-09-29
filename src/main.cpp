#include "SimulationEngine.h"
#include "TerminalUI.h"
#include "CanBoard.h"
#include <iostream>
#include <string>
#include <memory>

int main() {
    SimulationEngine simulation;

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(2.0)
                       .maxControlSignal(100)
                       .timeConstant(0.02)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(4.0)
                       .maxControlSignal(100)
                       .timeConstant(0.022)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false));

    simulation.start();

    std::unique_ptr<CanBoard> canBoard = std::make_unique<CanBoard>(
        simulation.getServo(0),
        0x10,
        "vcan0"
    );

    std::unique_ptr<CanBoard> canBoard2 = std::make_unique<CanBoard>(
        simulation.getServo(1),
        0x11,
        "vcan0"
    );

    // Start the CAN board
    canBoard->start();
    canBoard2->start();

    // TerminalUI ui(simulation, simulation.getRunningRef(), simulation.getSimulationFrequency());

    // Process user commands
    // std::string command;
    // while (simulation.isRunning() && std::getline(std::cin, command)) {
    //     ui.processCommand(command);
    // }
    // Wait for program termination (e.g., Ctrl+C)
    std::cout << "Press Enter to stop the simulation..." << std::endl;
    std::cin.get();
    // Stop CAN board before stopping simulation
    canBoard->stop();
    canBoard2->stop();
    simulation.stop();

    std::cout << "Simulation and CAN board stopped." << std::endl;
    return 0;
}
