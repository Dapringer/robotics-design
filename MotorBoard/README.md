# Robotics Design MotorBoard Project

This project is designed to control a robotic system using an ATmega2560 microcontroller. The system includes motor control, ultrasonic sensors, and LED indicators. The code is written in C++ and uses the PlatformIO ecosystem for development and deployment.

## Project Structure

- **platformio.ini**: Configuration file for PlatformIO, specifying the environment and build settings.
- **include/**: Contains additional header files (currently empty).
- **lib/**: Placeholder for external libraries (currently empty).
- **src/**: Contains the main source code for the project.
  - **UltraSonicAndLED.ino**: Main Arduino sketch for the project.
  - **class/**: Contains class definitions and implementations.
    - **Motor.cpp**: Implementation of the Motor class.
    - **Motor.h**: Header file for the Motor class.
- **test/**: Placeholder for test files (currently empty).

## Features

1. **Motor Control**:
   - Two motors are controlled using PWM signals.
   - Encoders are used for feedback to measure speed and distance.

2. **Ultrasonic Sensors**:
   - Detect obstacles and measure distances.

3. **LED Indicators**:
   - Provide visual feedback for system status.

## Setting Up the Project

1. **Install PlatformIO**:
   - Install the PlatformIO extension in Visual Studio Code.

2. **Clone the Repository**:
   - Clone this repository to your local machine.

3. **Open the Project**:
   - Open the project folder in Visual Studio Code.

4. **Select the Environment**:
   - Ensure the `platformio.ini` file is configured for the ATmega2560 microcontroller.

## Uploading the Code

1. **Check the COM Port**:
   - Connect your ATmega2560 board to your computer via USB.
   - Open the PlatformIO terminal and run the following command to list available COM ports:
     ```
     platformio device list
     ```
   - Identify the COM port for your board (e.g., `COM6`).

2. **Update the COM Port**:
   - Open the `platformio.ini` file and update the `upload_port` parameter with the correct COM port:
     ```
     upload_port = COM6
     ```

3. **Upload the Code**:
   - Use the PlatformIO toolbar or run the following command in the terminal:
     ```
     platformio run --target upload
     ```