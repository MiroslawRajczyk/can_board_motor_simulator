#include "SimulationEngine.h"
#include <iostream>

int main() {
    SimulationEngine simulation;

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(2.0)
                       .maxControlSignal(100)
                       .timeConstant(0.02)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false)
                       .canId(0x10)
                       .canInterface("vcan0"));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(4.0)
                       .maxControlSignal(100)
                       .timeConstant(0.022)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false)
                       .canId(0x11)
                       .canInterface("vcan0"));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(4.0)
                       .maxControlSignal(100)
                       .timeConstant(0.025)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false)
                       .canId(0x12)
                       .canInterface("vcan0"));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(14.0)
                       .maxControlSignal(100)
                       .timeConstant(0.022)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false)
                       .canId(0x13)
                       .canInterface("vcan0"));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(2.8)
                       .maxControlSignal(100)
                       .timeConstant(0.022)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false)
                       .canId(0x14)
                       .canInterface("vcan0"));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(15.8)
                       .maxControlSignal(100)
                       .timeConstant(0.022)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false)
                       .canId(0x15)
                       .canInterface("vcan0"));

    simulation.addServo(Servo::builder()
                       .maxVelocityRPM(10.0)
                       .maxControlSignal(100)
                       .timeConstant(0.012)
                       .encoderBitResolution(18)
                       .encoderDirectionInverted(false)
                       .canId(0x16)
                       .canInterface("vcan0"));

    simulation.start();

    // Wait for program termination (e.g., Ctrl+C)
    std::cout << "Press Enter to stop the simulation..." << std::endl;
    std::cin.get();

    simulation.stop();
    return 0;
}
