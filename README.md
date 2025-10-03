# CAN Board Motor Simulator

A C++ simulation of DC motors with encoders and CAN communication using Linux SocketCAN.

## Features

- **DC Motor Physics Simulation**: Realistic motor behavior with configurable parameters
- **Encoder Simulation**: Absolute rotary encoder with configurable bit resolution
- **CAN Communication**: Full CAN bus integration using Linux SocketCAN
- **Multi-Servo Support**: Simulate multiple servo systems simultaneously
- **Builder Pattern**: Clean, fluent API for configuration

## Prerequisites

- Linux system with SocketCAN support
- CMake 3.10 or higher
- C++17 compatible compiler (GCC 7+, Clang 5+)
- CAN utilities (optional, for testing)

## Setup Virtual CAN Interface

Before running the simulator, you need to set up a virtual CAN interface:

### 1. Load the vcan kernel module
```bash
sudo modprobe vcan
```

### 2. Create virtual CAN interface
```bash
sudo ip link add dev vcan0 type vcan
```

### 3. Bring the interface up
```bash
sudo ip link set up vcan0
```

### 4. Verify the interface is working
```bash
ip link show vcan0
```

You should see output similar to:
```
3: vcan0: <NOARP,UP,LOWER_UP> mtu 72 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
    link/can
```

### 5. (Optional) Install CAN utilities for testing
```bash
sudo apt-get install can-utils
```

## Building the Project

```bash
# Create build directory
mkdir build
cd build

# Configure with CMake
cmake ..

# Build the project
make
```

## Running the Simulator

```bash
# From the project root directory
./build/motor_simulator
```

The simulator will:
- Create 7 servo systems with different configurations
- Start CAN communication on vcan0
- Begin physics simulation
- Wait for user input to stop

## Testing CAN Communication

In another terminal, you can monitor CAN traffic:

```bash
# Monitor all CAN frames
candump vcan0

# Send a test CAN frame (effort command to servo 0x10)
cansend vcan0 10#1050
```

### CAN Frame Format

**Outgoing frames (from simulator):**
- CAN ID: Servo ID (0x10, 0x11, 0x12, etc.)
- Data: `13 EH EL SH SL EF`
  - `13`: Message type (status)
  - `EH EL`: Encoder position (16-bit, high/low bytes)
  - `SH SL`: Speed in RPM × 100 (16-bit signed, high/low bytes)
  - `EF`: Effort/control signal (8-bit signed)

**Incoming frames (to simulator):**
- CAN ID: Target servo ID
- Data: `10 EF`
  - `10`: Message type (effort command)
  - `EF`: Effort value (-100 to +100, or special values: 0=stop with hold, 1/-1=stop without hold)

## Configuration Example

```cpp
auto servo = Servo::builder()
    .maxVelocityRPM(120.0)        // Maximum velocity in RPM
    .maxControlSignal(100)        // Control signal range: -100 to +100
    .timeConstant(0.15)           // Motor response time constant
    .encoderBitResolution(18)     // 262,144 steps per revolution
    .encoderDirectionInverted(false)
    .canId(0x10)                  // CAN ID for this servo
    .canInterface("vcan0");       // CAN interface name
```

## Cleanup

To remove the virtual CAN interface when done:

```bash
sudo ip link delete vcan0
```

## Troubleshooting

**"Interface vcan0 not found":**
- Make sure you've loaded the vcan module: `sudo modprobe vcan`
- Verify the interface exists: `ip link show vcan0`

**"Permission denied" when accessing CAN:**
- Make sure the vcan0 interface is up: `sudo ip link set up vcan0`
- Try running with sudo (not recommended for development)

**No CAN frames visible:**
- The simulator will show "CAN socket is not open" if vcan0 is not available
- It will continue running without CAN communication in this case