# Robotics & Design SensorBoard Arduino Project

## Overview

The SensorBoard project is an Arduino-based system designed to monitor and detect obstacles or walls using ultrasonic sensors. It leverages the PlatformIO framework for streamlined development and is tailored for the Arduino Mega 2560 board. By utilizing four ultrasonic sensors, the system measures distances and employs a wall detection algorithm to identify approaching obstacles with precision.

## Features

1. **Ultrasonic Sensors**:

   - Four sensors are used, each with a trigger pin and an echo pin.
   - Sensors are configured to detect walls or obstacles within a specific range.

2. **Wall Detection Algorithm**:

   - The system uses a circular buffer to store the last five distance measurements for each sensor.
   - It calculates trends and detects walls based on the number of drops (negative changes) in distance and the average step change.

3. **Interrupt-Based Measurement**:

   - Hardware interrupts are used for precise timing of the echo signal, ensuring accurate distance measurements.

4. **Debugging**:
   - Sensor statuses, including distance, duration, and wall detection results, are printed to the Serial Monitor for debugging.

## Hardware Setup

- **Microcontroller**: Arduino Mega 2560
- **Ultrasonic Sensors**: Four sensors connected to the following pins:
  - Front: Trigger Pin `22`, Echo Pin `2`
  - Right: Trigger Pin `24`, Echo Pin `3`
  - Back: Trigger Pin `26`, Echo Pin `18`
  - Left: Trigger Pin `28`, Echo Pin `19`

## Software Components

1. **`Sensor` Class**:

   - Manages individual ultrasonic sensors.
   - Handles distance measurement, wall detection, and interrupt registration.
   - Provides methods for triggering sensors, reading distances, and printing statuses.

2. **Main Program (`UltraSonicAndLED.ino`)**:

   - Initializes sensors and handles the main control loop.
   - Reads sensor data and determines actions based on wall detection results.

3. **PlatformIO Configuration**:
   - The project is configured using `platformio.ini` for the Arduino Mega 2560 board.
   - Serial communication is set to 9600 baud.

## Usage

1. **Setup**:

   - Connect the ultrasonic sensors to the specified pins on the Arduino Mega 2560.
   - Upload the code to the Arduino using PlatformIO.

2. **Run**:

   - Open the Serial Monitor at 9600 baud to view sensor statuses and wall detection results.

3. **Debugging**:
   - Use the `printStatus` method in the `Sensor` class to monitor sensor readings and wall detection results.

## Things to Pay Attention To

1. **Sensor Placement**:

   - Ensure the ultrasonic sensors are securely mounted and aligned correctly for accurate distance measurement.

2. **Interrupt Pins**:

   - The echo pins must be connected to interrupt-capable pins on the Arduino Mega 2560 (e.g., pins `2`, `3`, `18`, `19`).

3. **Wall Detection Thresholds**:

   - The wall detection algorithm uses specific thresholds for distance and trend analysis. Adjust these thresholds in the `isWallApproaching` method if needed.

4. **Serial Debugging**:

   - The Serial Monitor is essential for debugging. Ensure it is open and set to the correct baud rate (9600).

5. **Power Supply**:

   - Ensure the Arduino and sensors are powered adequately to avoid inconsistent readings.

6. **Interrupt Conflicts**:
   - Avoid using the same interrupt pins for other peripherals to prevent conflicts.
