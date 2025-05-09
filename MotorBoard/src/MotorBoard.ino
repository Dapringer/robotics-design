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

// Define the I2C pins and address
#define I2C_SDA 20
#define I2C_SCL 21
#define I2C_ADDRESS 0x08

// Define the speed and turning parameters
#define MAX_SPEED 150
#define MIN_SPEED 50
#define TURNING_SPEED 150
// #define TURNING_TIME 500

// Initialize the motors
Motor motorLeft(1, M1_PWM, M1_DIR, Motor1EncA, Motor1EncB);
Motor motorRight(2, M2_PWM, M2_DIR, Motor2EncA, Motor2EncB);

// Initialize the I2C address and buffer
const byte arduAddress = I2C_ADDRESS;
char dataReceived[32];
volatile bool newDataAvailable = false;

bool stopOverwrite = false;
bool humanInFront = false;

// receiveEvent function to handle incoming I2C data
// This function is called when data is received from the I2C master
// It reads the data and sets the newDataAvailable flag to true
// The data is stored in the dataReceived array
void receiveEvent(int howMany)
{
  int i = 0;
  while (Wire.available() > 0 && i < sizeof(dataReceived) - 1)
  {
    char c = Wire.read();
    if (c != '\0')
    { // skip null characters
      dataReceived[i++] = c;
    }
  }
  dataReceived[i] = '\0';
  newDataAvailable = true;
}

// processI2CData function to process the received I2C data
// This function is called in the loop() function to check if new data is available
// If new data is available, it processes the data and executes the corresponding command
// It also handles the case where the received data is a float value for proportional speed control
// It uses the atof function to convert the string to a float value
// and checks if the value is within the range of -10 to 10
// If the value is within this range, it drives the robot with proportional speed
void processI2CData()
{
  if (newDataAvailable)
  {
    newDataAvailable = false; // Reset the flag immediately
    Serial.print("Arduino received (polling): ");
    Serial.println(dataReceived);

    float receivedValue = atof(dataReceived); // Always safe, even if non-numeric

    // Check if it's a command (whole number like 100–105)
    int intValue = (int)receivedValue;

    if (receivedValue > -10 && receivedValue < 10 && receivedValue != 0)
    {
      Serial.println("Driving with proportional speed...");
      driveWithProportionalSpeed(receivedValue);
      return;
    }

    Serial.println("Executing command based on received value...");
    // Execute different functions based on the dataReceived
    // Check if the received data is a command to stop the motors switch (dataReceived)
    switch (intValue)
    {
    case 100: // Stop both motors
      driveRobot(0);
      humanInFront = false; // Reset the humanInFront flag
      stopOverwrite = true; // Set the stopOverwrite flag to true
      Serial2.println("humanInFront = false");
      Serial2.println("stopOverwrite = true");
      break;
    case 101: // Disable Walldetection in Front
      driveRobot(0);
      Serial.println("Human detected in front, stopping robot!");
      humanInFront = true;
      stopOverwrite = false; // Set the stopOverwrite flag to true
      Serial2.println("humanInFront = true");
      Serial2.println("stopOverwrite = false");
      break;
    case 102: // Turn left
      driveRobot(3);
      humanInFront = false;  // Reset the humanInFront flag
      stopOverwrite = false; // Reset the stopOverwrite flag
      Serial2.println("humanInFront = false");
      Serial2.println("stopOverwrite = false");
      break;
    case 103: // Turn right
      driveRobot(4);
      humanInFront = false;  // Reset the humanInFront flag
      stopOverwrite = false; // Reset the stopOverwrite flag
      Serial2.println("humanInFront = false");
      Serial2.println("stopOverwrite = false");
      break;
    case 104: // Move forward
      driveRobot(1);
      humanInFront = false;  // Reset the humanInFront flag
      stopOverwrite = false; // Reset the stopOverwrite flag
      Serial2.println("humanInFront = false");
      Serial2.println("stopOverwrite = false");
      break;
    case 105: // Move backward
      driveRobot(2);
      humanInFront = false;  // Reset the humanInFront flag
      stopOverwrite = false; // Reset the stopOverwrite flag
      Serial2.println("humanInFront = false");
      Serial2.println("stopOverwrite = false");
      break;
    default:                 // Invalid command
      stopOverwrite = false; // Reset the stopOverwrite flag
      Serial2.println("stopOverwrite = false");
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
  Serial.begin(9600);
  Serial2.begin(9600);
  Wire.begin(arduAddress);      // Join the I2C bus as a slave
  Wire.onReceive(receiveEvent); // Register the function for received data
  Serial.println("Arduino I2C Slave (Polling) ready...");

  attachInterrupt(digitalPinToInterrupt(Motor1EncA), updateLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Motor1EncB), updateLeftEncoder, RISING);

  attachInterrupt(digitalPinToInterrupt(Motor2EncA), updateRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Motor2EncB), updateRightEncoder, RISING);
}

void loop()
{
  // Check if data is available from Sensor Board
  if (Serial2.available() > 0)
  {
    String command = Serial2.readStringUntil('\n'); // Read the command
    processSerialCommand(command);                  // Process the received command
  }
  // Process I2C data if available
  // processI2CData();

  // // If stopOverwrite is true skip wall detection and listen for new I2C commands
  // if (stopOverwrite)
  // {
  //   return;
  // }

  delay(100);
}

// Function to process the received command and execute the corresponding action
// This function takes a string command as input and checks it against predefined commands
// It uses the drive method of the Motor class to control the motors
// The function also sends an acknowledgment back to the sender using Serial2
// The acknowledgment is sent in the format "ACK:<command>"
void processSerialCommand(String command)
{
  command.trim(); // Remove any leading/trailing whitespace or newline characters

  if (command == "CMD:STOP")
  {
    motorLeft.emergencyStop();
    motorRight.emergencyStop();
    Serial.println("Emergency Stop Activated");
    Serial2.println("ACK:STOP"); // Send acknowledgment
  }
  else if (command == "CMD:LEFT")
  {
    motorLeft.drive(0, 2);              // Stop left motor
    motorRight.drive(TURNING_SPEED, 1); // Turn right motor forward
    Serial.println("Turning Left");
    Serial2.println("ACK:LEFT"); // Send acknowledgment
  }
  else if (command == "CMD:RIGHT")
  {
    motorLeft.drive(TURNING_SPEED, 1); // Turn left motor forward
    motorRight.drive(0, 2);            // Stop right motor
    Serial.println("Turning Right");
    Serial2.println("ACK:RIGHT"); // Send acknowledgment
  }
  else if (command == "CMD:FORWARD")
  {
    motorLeft.drive(MAX_SPEED, 1); // Move both motors forward
    motorRight.drive(MAX_SPEED, 1);
    Serial.println("Moving Forward");
    Serial2.println("ACK:FORWARD"); // Send acknowledgment
  }
  else if (command == "CMD:BACKWARD")
  {
    motorLeft.drive(MAX_SPEED, 2); // Move both motors backward
    motorRight.drive(MAX_SPEED, 2);
    Serial.println("Moving Backward");
    Serial2.println("ACK:BACKWARD"); // Send acknowledgment
  }
  else
  {
    Serial.print("Unknown Command: ");
    Serial.println(command);
    Serial2.println("ACK:UNKNOWN"); // Send acknowledgment for unknown command
  }
}

// Function to drive the robot with proportional speed based on the received value
// This function takes a float value as input and adjusts the motor speeds accordingly
void driveWithProportionalSpeed(float value)
{
  // Define the maximum speed difference
  const int maxSpeedDifference = 100; // Adjust this value as needed

  // Calculate the speed adjustment based on the received value
  int speedAdjustment = map(value, -10, 10, -maxSpeedDifference, maxSpeedDifference);

  // Calculate the motor speeds
  int leftMotorSpeed = constrain(MAX_SPEED + speedAdjustment, MIN_SPEED, MAX_SPEED);
  int rightMotorSpeed = constrain(MAX_SPEED - speedAdjustment, MIN_SPEED, MAX_SPEED);

  // Drive the motors
  motorLeft.drive(leftMotorSpeed, 1);   // Forward
  motorRight.drive(rightMotorSpeed, 1); // Forward

  // Debug output
  Serial.print("Left Motor Speed: ");
  Serial.println(leftMotorSpeed);
  Serial.print("Right Motor Speed: ");
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