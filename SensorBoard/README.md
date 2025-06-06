# Robotics & Design SensorBoard Arduino Project

## Overview

The SensorBoard project is an Arduino-based system designed to monitor and detect obstacles or walls using ultrasonic sensors. It leverages the PlatformIO framework for streamlined development and is tailored for the Arduino Mega 2560 board. By utilizing four ultrasonic sensors, the system measures distances and employs a wall detection algorithm to identify approaching obstacles with precision, sending appropriate commands to the Motor Board.

## Features

1. **Ultrasonic Sensors**:

   - Four sensors are used, each with a trigger pin and an echo pin.
   - Sensors detect obstacles in all directions (front, back, left, right).
   - Emergency stop is triggered when obstacles are detected within specified thresholds.

2. **Wall Detection Algorithm**:

   - The system uses a circular buffer to store the last ten distance measurements for each sensor.
   - It calculates trends and detects walls based on the number of drops (negative changes) in distance and the average step change.
   - Note: Advanced wall detection is currently disabled for testing, with only emergency breaking functionality enabled.

3. **Interrupt-Based Measurement**:

   - Hardware interrupts are used for precise timing of the echo signal, ensuring accurate distance measurements.

4. **Communication with Motor Board**:

   - Sends commands to the Motor Board via Serial2.
   - Command values: 0 (stop), 2 (back up), 3 (turn left), 4 (turn right).
   - Receives acknowledgments from the Motor Board to confirm command execution.

5. **Human Detection Safety Feature**:
   - Special mode for when a human is detected in front of the robot.
   - Prevents automatic movements when humans are present.

## Hardware Setup

- **Microcontroller**: Arduino Mega 2560
- **Ultrasonic Sensors**: Four sensors connected to the following pins:
  - Front: Trigger Pin `22`, Echo Pin `2`
  - Right: Trigger Pin `24`, Echo Pin `3`
  - Back: Trigger Pin `26`, Echo Pin `18`
  - Left: Trigger Pin `28`, Echo Pin `19`
- **Serial Communication**:
  - Serial: Used for debugging (115200 baud rate)
  - Serial2: Used for communication with Motor Board (9600 baud rate)

## Software Components

1. **`Sensor` Class**:

   - Manages individual ultrasonic sensors.
   - Handles distance measurement, wall detection, and interrupt registration.
   - Provides methods for triggering sensors, reading distances, and printing statuses.
   - Implements advanced wall approach detection algorithm.

2. **Main Program (`SensorBoard.ino`)**:

   - Initializes sensors and handles the main control loop.
   - Reads sensor data and sends appropriate commands to the Motor Board.
   - Implements emergency stopping logic based on distance thresholds.
   - Contains commented wall detection and avoidance code for future implementation.

3. **Communication Protocol**:

   - Sends numeric commands to the Motor Board:
     - `0`: Emergency stop (obstacle detected)
     - `2`: Back up (obstacle ahead)
     - `3`: Turn left (obstacle on right)
     - `4`: Turn right (obstacle on left)
   - Receives status messages from the Motor Board regarding human detection.

4. **PlatformIO Configuration**:
   - The project is configured using `platformio.ini` for the Arduino Mega 2560 board.
   - Serial communication is set to 115200 baud for debugging.
   - Serial2 communication is set to 9600 baud for Motor Board communication.

## Usage

1. **Setup**:

   - Connect the ultrasonic sensors to the specified pins on the Arduino Mega 2560.
   - Connect Serial2 (pins 16 and 17) to the Motor Board's Serial2 pins.
   - Upload the code to the Arduino using PlatformIO.

2. **Run**:

   - Open the Serial Monitor at 115200 baud to view sensor readings and command logs.
   - The system will automatically detect obstacles and send appropriate commands to the Motor Board.

3. **Testing**:
   - For initial testing, only the emergency breaking functionality is enabled.
   - The more advanced wall detection and avoidance features are commented out but can be enabled once basic functionality is verified.

## Things to Pay Attention To

1. **Sensor Placement**:

   - Ensure the ultrasonic sensors are securely mounted and aligned correctly for accurate distance measurements.
   - Front and back sensors should be centered, while left and right sensors should be perpendicular to the direction of travel.

2. **Interrupt Pins**:

   - The echo pins must be connected to interrupt-capable pins on the Arduino Mega 2560 (e.g., pins `2`, `3`, `18`, `19`).
   - Ensure no other interrupts conflict with these pins.

3. **Emergency Stop Distances**:

   - The system uses specific thresholds for emergency stopping:
     - Front: `30` cm
     - Sides: `15` cm
     - Back: `20` cm
   - Adjust these values in the code if needed for your specific environment.

4. **Serial Communication**:

   - Ensure the baud rates match between the Sensor Board and Motor Board (9600 baud).
   - The Serial Monitor should be set to 115200 baud for debugging.

5. **Human Detection Mode**:

   - When the Motor Board signals that a human is detected, the Sensor Board will suspend automatic obstacle avoidance.
   - This is a safety feature to prevent unexpected robot movements around humans.

6. **Wall Detection Algorithm**:
   - The advanced wall detection feature (currently commented out) uses trend analysis to detect approaching walls.
   - It can be enabled once the basic functionality is verified.
