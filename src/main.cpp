#include "SimulationEngine.h"
#include "TerminalUI.h"
#include <iostream>
#include <string>

int main() {
    SimulationEngine simulation;

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(120.0)
                       .maxControlSignal(800)
                       .timeConstant(0.2)
                       .encoderBitResolution(16)
                       .encoderDirectionInverted(true));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(200.0)
                       .maxControlSignal(1500)
                       .timeConstant(0.4)
                       .encoderBitResolution(20)
                       .encoderDirectionInverted(false));

    simulation.start();

    TerminalUI ui(simulation, simulation.getRunningRef(), simulation.getSimulationFrequency());

    // Process user commands
    std::string command;
    while (simulation.isRunning() && std::getline(std::cin, command)) {
        ui.processCommand(command);
    }

    simulation.stop();

    std::cout << "Simulation stopped." << std::endl;
    return 0;
}
