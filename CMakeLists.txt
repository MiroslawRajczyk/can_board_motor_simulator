cmake_minimum_required(VERSION 3.10)
project(MotorSimulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executable
add_executable(motor_simulator
    src/main.cpp
    src/Motor.cpp
    src/Encoder.cpp
    src/MotorController.cpp
)

# Include directories
target_include_directories(motor_simulator PRIVATE include)

# Add compiler flags
target_compile_options(motor_simulator PRIVATE -Wall -Wextra -O2)