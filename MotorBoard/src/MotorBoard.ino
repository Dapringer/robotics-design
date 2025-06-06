/**
 * MotorBoard.ino
 *
 * Main file for the Motor Board of the robotics-design project.
 * This board is responsible for controlling the robot's motors based on commands
 * received from either the Sensor Board (via Serial2) or a Raspberry Pi (via Serial).
 *
 * The board prioritizes commands from the Sensor Board for emergency stops,
 * but otherwise follows commands from the Raspberry Pi for normal operation.
 *
 * Communication:
 * - Serial: Communication with Raspberry Pi (115200 baud)
 * - Serial2: Communication with Sensor Board (9600 baud)
 *
 * Command Values for Serial Communication with Raspberry Pi:
 * - 100: Stop both motors
 * - 101: Disable wall detection (human detected in front)
 * - 102: Turn left
 * - 103: Turn right
 * - 104: Move forward
 * - 105: Move backward
 * - -10 to 10: Proportional speed control
 *
 * @author: Movement and Localization Outdoor Robot Team
 */
#include <stdlib.h>
#include <Arduino.h>
#include <Wire.h>

#include "class/Motor.h"

// Define the motor pins
// Motor 1: Left motor
// Motor 2: Right motor
#define M1_PWM 8
#define M1_DIR 9
#define M2_PWM 10
#define M2_DIR 11

// Define the encoder pins
// Motor 1: Left motor encoder
// Motor 2: Right motor encoder
#define Motor1EncA 2
#define Motor1EncB 3
#define Motor2EncA 18
#define Motor2EncB 19

// Define the speed and turning parameters
#define MAX_SPEED 150     // Maximum motor speed (0-255)
#define MIN_SPEED 50      // Minimum motor speed to ensure movement
#define TURNING_SPEED 100 // Speed used for turning operations

// Initialize the motors
Motor motorLeft(1, M1_PWM, M1_DIR, Motor1EncA, Motor1EncB);
Motor motorRight(2, M2_PWM, M2_DIR, Motor2EncA, Motor2EncB);

// Control flags
bool humanInFront = false; // Flag to indicate if a human is detected in front

// Add a global variable to store the last received Serial value from PI
float lastSerialPIValue = 100.0; // Default to stop command

// processSerialData function to process the received Serial data
// This function is called in the loop() function to check if new data is available
// If new data is available, it processes the data and executes the corresponding command
// It also handles the case where the received data is a float value for proportional speed control
// It uses the atof function to convert the string to a float value
// and checks if the value is within the range of -10 to 10
// If the value is within this range, it drives the robot with proportional speed
void processSerialData()
{
  if (Serial.available())
  {

    String command = Serial.readStringUntil('\n'); // Read the command
    float receivedValue = command.toFloat();       // Convert string to float
    Serial.print("Received value from Serial: ");
    Serial.println(receivedValue); // Print the received value to Serial Monitor

    // Save the received value to the global variable
    lastSerialPIValue = receivedValue;

    if (Serial2.available() && receivedValue != 100)
    {
      Serial2.println("Ignoring Serial data from PI, Serial2 from Arduino is available.");
      return;
    }

    // If the received value is within the range of -10 to 10, drive with proportional speed
    // Otherwise, convert it to an integer for command execution
    int intValue = (int)receivedValue;

    if (receivedValue > -10 && receivedValue < 10)
    {
      driveWithProportionalSpeed(receivedValue);
      return;
    }

    Serial.println("Executing command based on received value: " + String(receivedValue));
    // Execute different functions based on the dataReceived
    switch (intValue)
    {
    case 100: // Stop both motors
      driveRobot(0);
      humanInFront = false; // Reset the humanInFront flag
      Serial2.println("humanInFront = false");
      break;
    case 101: // Disable Walldetection in
      driveRobot(0);
      Serial.println("Human detected in front, stopping robot!");
      humanInFront = true; // Reset the humanInFront flag
      Serial2.println("humanInFront = true");
      break;
    case 102: // Turn left
      driveRobot(3);
      humanInFront = false; // Reset the humanInFront flag
      Serial2.println("humanInFront = false");
      break;
    case 103: // Turn right
      driveRobot(4);
      humanInFront = false; // Reset the humanInFront flag
      Serial2.println("humanInFront = false");
      break;
    case 104: // Move forward
      driveRobot(1);
      humanInFront = false; // Reset the humanInFront flag
      Serial2.println("humanInFront = false");
      break;
    case 105: // Move backward
      driveRobot(2);
      humanInFront = false; // Reset the humanInFront flag
      Serial2.println("humanInFront = false");
      break;
    default: // Invalid command
      Serial2.println("Invalid command");
      break;
    }
  }
}

// Update the encoder count and speed for the left motor
// This function is called in the interrupt service routine (ISR) for the left motor encoder
void updateLeftEncoder()
{
  motorLeft.encoderCount_++;
  motorLeft.getSpeed();
}

// Update the encoder count and speed for the right motor
// This function is called in the interrupt service routine (ISR) for the right motor encoder
void updateRightEncoder()
{
  motorRight.encoderCount_++;
  motorRight.getSpeed();
}

void setup()
{
  Serial.begin(115200); // Initialize Serial communication at 115200 baud rate
  Serial2.begin(9600);

  attachInterrupt(digitalPinToInterrupt(Motor1EncA), updateLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Motor1EncB), updateLeftEncoder, RISING);

  attachInterrupt(digitalPinToInterrupt(Motor2EncA), updateRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Motor2EncB), updateRightEncoder, RISING);
  Serial.println("Arduino Motor Board Setup");
}

void loop()
{
  // Always listen to Serial from PI and process it conditionally
  processSerialData(); // Read Serial data to update the value if available

  // If Serial2 is available, prioritize Serial commands
  // This allows the Sensor Board to send commands to stop the motors in case of emergency
  // If the last received value is 100, it means stop command, so we don't process Serial2 commands
  if (Serial2.available() && lastSerialPIValue != 100)
  {
    String command = Serial2.readStringUntil('\n'); // Read the command
    Serial.print("Received command from Serial2: ");
    Serial.println(command);                  // Print the command to Serial Monitor
    processSerialCommandFromSensors(command); // Process the received command
  }

  delay(100);
}

// Function to process the received command from the Arduino SensorBoard and execute the corresponding action
// This function takes a string command as input and checks it against predefined commands
// It uses the drive method of the Motor class to control the motors
void processSerialCommandFromSensors(String command)
{
  command.trim();                  // Remove any leading/trailing whitespace or newline characters
  int direction = command.toInt(); // Convert the command to an integer

  switch (direction)
  {
  case 0: // Stop
    motorLeft.emergencyStop();
    motorRight.emergencyStop();
    Serial.println("Stopping because of command 0 from Serial2");
    break;
  case 1: // Drive forward
    motorLeft.drive(MAX_SPEED, 1);
    motorRight.drive(MAX_SPEED, 1);
    Serial.println("Driving Forward because of command 1 from Serial2");
    break;
  case 2: // Drive backward
    motorLeft.drive(MAX_SPEED, 2);
    motorRight.drive(MAX_SPEED, 2);
    Serial.println("Driving Backward because of command 2 from Serial2");
    break;
  case 3: // Turn left
    motorLeft.drive(0, 2);
    motorRight.drive(TURNING_SPEED, 1);
    Serial.println("Turning Left because of command 3 from Serial2");
    break;
  case 4: // Turn right
    motorLeft.drive(TURNING_SPEED, 1);
    motorRight.drive(0, 2);
    Serial.println("Turning Right because of command 4 from Serial2");
    break;
  default:
    Serial.println("Invalid Direction Command from Serial2: " + command);
    break;
  }
}

// Function to drive the robot with proportional speed based on the received value
// This function takes a float value as input and adjusts the motor speeds accordingly
void driveWithProportionalSpeed(float value)
{
  // Define the maximum speed difference
  const int maxSpeedDifference = 100;

  // Calculate the speed adjustment based on the received value
  int speedAdjustment = map(value, -10, 10, -maxSpeedDifference, maxSpeedDifference);

  // Calculate the motor speeds
  int leftMotorSpeed = constrain(MAX_SPEED + speedAdjustment, MIN_SPEED, MAX_SPEED);
  int rightMotorSpeed = constrain(MAX_SPEED - speedAdjustment, MIN_SPEED, MAX_SPEED);

  // Drive the motors
  motorLeft.drive(leftMotorSpeed, 1);   // Forward
  motorRight.drive(rightMotorSpeed, 1); // Forward

  // Debug output
  Serial.print("Driving with proportional speed  | ");
  Serial.print("Left Motor Speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print("  |  Right Motor Speed: ");
  Serial.println(rightMotorSpeed);
}

// Function to drive the robot in a specific direction
// This function takes an integer direction as input and drives the motors accordingly
// The direction can be 0 (stop), 1 (forward), 2 (backward), 3 (turn left), or 4 (turn right)
// The function uses the drive method of the Motor class to control the motors
void driveRobot(int direction)
{
  if (direction == 3)
  {
    // Turn Left
    motorLeft.drive(0, 2);
    motorRight.drive(TURNING_SPEED, 1);
  }
  else if (direction == 4)
  {
    // Turn Right
    motorLeft.drive(TURNING_SPEED, 1);
    motorRight.drive(0, 2);
  }
  else
  {
    motorLeft.drive(MAX_SPEED, direction);
    motorRight.drive(MAX_SPEED, direction);
    // Serial.println("Driving robot in direction: " + String(direction));
  }
}