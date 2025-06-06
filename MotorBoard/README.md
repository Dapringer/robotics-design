# Robotics Design MotorBoard Arduino Project

This project is designed to control a robotic system using an Arduino Mega (ATmega2560) microcontroller. The MotorBoard is responsible for controlling the robot's motors based on commands received from both the Sensor Board (via Serial2) and a Raspberry Pi (via Serial or I2C).

## Project Structure

- **platformio.ini**: Configuration file for PlatformIO, specifying the environment and build settings.
- **include/**: Contains additional header files (currently empty).
- **lib/**: Placeholder for external libraries (currently empty).
- **src/**: Contains the main source code for the project.
  - **MotorBoard.ino**: Main Arduino sketch containing the motor control logic and communication handling.
  - **class/**: Contains class definitions and implementations.
    - **Motor.cpp**: Implementation of the Motor class for controlling DC motors with encoder feedback.
    - **Motor.h**: Header file for the Motor class.
- **test/**: Placeholder for test files (currently empty).

## Features

1. **Motor Control**:

   - Two motors are controlled using PWM signals with directional control.
   - Encoders are used for feedback to measure speed.
   - Supports variable speed control including proportional speed adjustments.
   - Emergency stop functionality for immediate response to obstacles.

2. **Multi-Source Command Processing**:

   - Processes commands from the Sensor Board via Serial2.
   - Processes commands from a Raspberry Pi via Serial or I2C.
   - Prioritizes emergency commands from the Sensor Board.

3. **Command System for Comm. with PI**:

   - Standardized command values for different movement types:
     - 100: Stop both motors
     - 101: Disable wall detection (human detected in front)
     - 102: Turn left
     - 103: Turn right
     - 104: Move forward
     - 105: Move backward
     - -10 to 10: Proportional speed control

4. **Command System for Comm. with SensorBoard**:
   - Standardized command values for different movement types:
     - **0**: Stop both motors
     - **1**: Move forward
     - **2**: Move backward
     - **3**: Turn left
     - **4**: Move right

## Hardware Setup

- **Microcontroller**: Arduino Mega 2560
- **Motor Pins**:
  - Left Motor: PWM Pin `8`, Direction Pin `9`
  - Right Motor: PWM Pin `10`, Direction Pin `11`
- **Encoder Pins**:
  - Left Motor: Encoder A Pin `2`, Encoder B Pin `3`
  - Right Motor: Encoder A Pin `18`, Encoder B Pin `19`
- **I2C Pins if needed**:
  - SDA Pin `20`
  - SCL Pin `21`

## Setting Up the Project

1. **Install PlatformIO**:

   - Install the PlatformIO extension in Visual Studio Code.

2. **Clone the Repository**:

   - Clone this repository to your local machine.

3. **Open the Project**:

   - Open the project folder in Visual Studio Code.

4. **Select the Environment**:
   - Ensure the `platformio.ini` file is configured for the Arduino Mega 2560 microcontroller.

## Uploading the Code

1. **Check the COM Port**:

   - Connect your Arduino Mega 2560 board to your computer via USB.
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

## Software Components

1. **`Motor` Class**:

   - Manages individual DC motors with encoder feedback.
   - Provides methods for controlling motor direction and speed.
   - Implements emergency stop functionality.
   - Calculates motor speed based on encoder pulses.

2. **Main Program (`MotorBoard.ino`)**:

   - Initializes motors and sets up communication channels.
   - Processes commands from multiple sources (Sensor Board and Raspberry Pi).
   - Implements various movement functions including proportional speed control.
   - Manages the human detection flag for safety purposes.

3. **Communication Protocol**:
   - Receives commands from the Sensor Board via Serial2.
   - Receives commands from the Raspberry Pi via Serial or I2C.
   - Sends acknowledgments back to the Sensor Board via Serial2.
   - Uses standardized command values for different movement types.

## Operation Details

1. **Command Priority**:

   - Emergency stop commands from the Sensor Board have the highest priority.
   - When not in emergency mode, the board follows commands from the Raspberry Pi.

2. **Proportional Speed Control**:

   - Values between -10 and 10 enable proportional speed control.
   - Positive values cause the robot to turn right (right motor slower).
   - Negative values cause the robot to turn left (left motor slower).
   - The magnitude determines the degree of turning.

3. **Encoder Feedback**:

   - Encoders provide speed feedback for precise motor control.
   - Speed is calculated based on encoder pulses per unit time.

4. **Human Detection Flag**:
   - When a human is detected (command 101), the flag is set.
   - This is communicated to the Sensor Board to suspend automatic obstacle avoidance.

## Things to Pay Attention To

1. **Motor Connections**:

   - Ensure motors are connected to the correct PWM and direction pins.
   - Check encoder connections for proper speed measurement.

2. **I2C Addressing**:

   - The Motor Board uses I2C address 0x08 for Raspberry Pi communication.
   - It uses a secondary address 0x09 for Sensor Board communication (if I2C is used).

3. **Serial Communication**:

   - Serial (USB): 115200 baud for communication with Raspberry Pi.
   - Serial2: 9600 baud for communication with the Sensor Board.

4. **Power Requirements**:

   - Motors require adequate power. Ensure your power supply can handle the peak current demands.
   - Use a separate power supply for motors if needed to prevent Arduino brownouts.

5. **Speed Parameters**:
   - MAX_SPEED: 150 (maximum motor speed)
   - MIN_SPEED: 50 (minimum motor speed)
   - TURNING_SPEED: 100 (speed used for turning)
   - These can be adjusted in the code based on your specific motors and requirements.
