#include "SimulationEngine.h"
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

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(4.0)
                       .maxControlSignal(100)
                       .timeConstant(0.025)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(14.0)
                       .maxControlSignal(100)
                       .timeConstant(0.022)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(2.8)
                       .maxControlSignal(100)
                       .timeConstant(0.022)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(15.8)
                       .maxControlSignal(100)
                       .timeConstant(0.022)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(10.0)
                       .maxControlSignal(100)
                       .timeConstant(0.012)
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

    std::unique_ptr<CanBoard> canBoard3 = std::make_unique<CanBoard>(
        simulation.getServo(2),
        0x12,
        "vcan0"
    );

    std::unique_ptr<CanBoard> canBoard4 = std::make_unique<CanBoard>(
        simulation.getServo(3),
        0x13,
        "vcan0"
    );

    std::unique_ptr<CanBoard> canBoard5 = std::make_unique<CanBoard>(
        simulation.getServo(4),
        0x14,
        "vcan0"
    );

    std::unique_ptr<CanBoard> canBoard6 = std::make_unique<CanBoard>(
        simulation.getServo(5),
        0x15,
        "vcan0"
    );

    std::unique_ptr<CanBoard> canBoard7 = std::make_unique<CanBoard>(
        simulation.getServo(6),
        0x16,
        "vcan0"
    );

    canBoard->start();
    canBoard2->start();
    canBoard3->start();
    canBoard4->start();
    canBoard5->start();
    canBoard6->start();
    canBoard7->start();

    // Wait for program termination (e.g., Ctrl+C)
    std::cout << "Press Enter to stop the simulation..." << std::endl;
    std::cin.get();

    canBoard->stop();
    canBoard2->stop();
    canBoard3->stop();
    canBoard4->stop();
    canBoard5->stop();
    canBoard6->stop();
    canBoard7->stop();

    simulation.stop();
    return 0;
}
