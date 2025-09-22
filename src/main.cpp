#include "MotorController.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>

void printMotorState(const MotorController& controller, double time) {
    const Motor& motor = controller.getMotor();
    const Encoder& encoder = controller.getEncoder();
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time: " << std::setw(6) << time << "s | ";
    std::cout << "Pos: " << std::setw(7) << encoder.getPositionRadians() << " rad | ";
    std::cout << "Vel: " << std::setw(7) << encoder.getVelocity() << " rad/s | ";
    std::cout << "Voltage: " << std::setw(6) << motor.getVoltage() << "V | ";
    std::cout << "Current: " << std::setw(6) << motor.getCurrent() << "A | ";
    std::cout << "Torque: " << std::setw(6) << motor.getTorque() << " Nm | ";
    std::cout << "Error: " << std::setw(7) << controller.getCurrentError() << std::endl;
}

void demonstrateOpenLoop(MotorController& controller) {
    std::cout << "\n=== Open Loop Control Demo ===" << std::endl;
    std::cout << "Applying 6V for 2 seconds" << std::endl;
    
    controller.reset();
    controller.setVoltage(6.0);
    
    double dt = 0.01;  // 10ms timestep
    double time = 0.0;
    
    while (time < 2.0) {
        controller.update(dt);
        
        if (static_cast<int>(time * 100) % 20 == 0) {  // Print every 200ms
            printMotorState(controller, time);
        }
        
        time += dt;
    }
}

void demonstratePositionControl(MotorController& controller) {
    std::cout << "\n=== Position Control Demo ===" << std::endl;
    std::cout << "Moving to position 3.14 radians (180 degrees)" << std::endl;
    
    controller.reset();
    controller.setPIDGains(5.0, 0.5, 0.1);  // Tune PID gains
    controller.setPosition(M_PI);  // 180 degrees
    
    double dt = 0.01;
    double time = 0.0;
    
    while (time < 3.0) {
        controller.update(dt);
        
        if (static_cast<int>(time * 100) % 20 == 0) {
            printMotorState(controller, time);
        }
        
        time += dt;
    }
}

void demonstrateVelocityControl(MotorController& controller) {
    std::cout << "\n=== Velocity Control Demo ===" << std::endl;
    std::cout << "Setting velocity to 10 rad/s" << std::endl;
    
    controller.reset();
    controller.setPIDGains(0.5, 0.1, 0.01);  // Different gains for velocity control
    controller.setVelocity(10.0);
    
    double dt = 0.01;
    double time = 0.0;
    
    while (time < 3.0) {
        controller.update(dt);
        
        if (static_cast<int>(time * 100) % 20 == 0) {
            printMotorState(controller, time);
        }
        
        time += dt;
    }
}

void demonstrateWithLoad(MotorController& controller) {
    std::cout << "\n=== Position Control with Load Disturbance ===" << std::endl;
    std::cout << "Moving to 1.57 rad with 0.01 Nm load applied at t=1s" << std::endl;
    
    controller.reset();
    controller.setPIDGains(5.0, 0.5, 0.1);
    controller.setPosition(M_PI/2);  // 90 degrees
    
    double dt = 0.01;
    double time = 0.0;
    double load_torque = 0.0;
    
    while (time < 4.0) {
        // Apply load disturbance after 1 second
        if (time > 1.0) {
            load_torque = 0.01;  // 0.01 Nm load
        }
        
        controller.update(dt, load_torque);
        
        if (static_cast<int>(time * 100) % 20 == 0) {
            printMotorState(controller, time);
            if (time > 1.0 && time < 1.2) {
                std::cout << "  <- Load applied!" << std::endl;
            }
        }
        
        time += dt;
    }
}

int main() {
    std::cout << "Motor and Encoder Simulator" << std::endl;
    std::cout << "===========================" << std::endl;
    
    MotorController controller;
    
    // Display motor parameters
    const Motor& motor = controller.getMotor();
    std::cout << "\nMotor Parameters:" << std::endl;
    std::cout << "Resistance: " << motor.getResistance() << " Ohms" << std::endl;
    std::cout << "Inductance: " << motor.getInductance() << " H" << std::endl;
    std::cout << "Back EMF Constant: " << motor.getBackEmfConstant() << " V*s/rad" << std::endl;
    std::cout << "Torque Constant: " << motor.getTorqueConstant() << " Nm/A" << std::endl;
    std::cout << "Inertia: " << motor.getInertia() << " kg*m^2" << std::endl;
    std::cout << "Friction: " << motor.getFriction() << " Nm*s/rad" << std::endl;
    
    // Run demonstrations
    demonstrateOpenLoop(controller);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    demonstratePositionControl(controller);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    demonstrateVelocityControl(controller);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    demonstrateWithLoad(controller);
    
    std::cout << "\nSimulation complete!" << std::endl;
    return 0;
}