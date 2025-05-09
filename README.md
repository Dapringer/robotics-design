# Robotics Design Project

This project is split into two separate PlatformIO projects, each running on a dedicated Arduino board:

## Overview

### 1. **Sensor Board**

- **Purpose**: Handles all ultrasonic sensors and sends sensor data to the Motor Board via Serial communication.
- **Location**: `SensorBoard/`
- **Key Files**:
  - `src/UtraSonicAndLED.ino`: Main logic for reading sensors and sending data.
  - `src/class/Sensor.cpp` and `src/class/Sensor.h`: Sensor class implementation.
  - `platformio.ini`: Configuration for the Sensor Board.

### 2. **Motor Board**

- **Purpose**: Handles motor control and encoders. Receives sensor data from the Sensor Board via Serial communication.
- **Location**: `MotorBoard/`
- **Key Files**:
  - `src/UtraSonicAndLED.ino`: Main logic for motor control and receiving sensor data.
  - `src/class/Motor.cpp` and `src/class/Motor.h`: Motor class implementation.
  - `platformio.ini`: Configuration for the Motor Board.

## Communication

- **Protocol**: Serial communication between the Sensor Board and Motor Board.
- **Data Format**: Sensor Board sends comma-separated values (e.g., `distFront,distRight,distBack,distLeft`).

## How to Use

1. **Sensor Board**:

   - Navigate to the `SensorBoard/` directory.
   - Open the `platformio.ini` file to configure the board and upload settings.
   - Upload the code to the Sensor Board.

2. **Motor Board**:
   - Navigate to the `MotorBoard/` directory.
   - Open the `platformio.ini` file to configure the board and upload settings.
   - Upload the code to the Motor Board.

## Dependencies

- Both projects use the Arduino framework.
- Ensure the required libraries are installed in `~/Documents/Arduino/libraries` or specified in the `platformio.ini` files.

## Testing

- Use the Serial Monitor to debug communication between the boards.
- Verify sensor readings on the Sensor Board and motor behavior on the Motor Board.

## Future Improvements

- Add error handling for Serial communication.
- Implement advanced control algorithms (e.g., PID) for motor synchronization.
