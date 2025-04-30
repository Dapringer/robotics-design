#include <stdlib.h>
#include <Arduino.h>
#include <Wire.h>

#include "class/Motor.h"
#include "class/Sensor.h"

#define PIN_TRIGGER_F 22
#define PIN_ECHO_F 2
#define PIN_TRIGGER_R 24
#define PIN_ECHO_R 3
#define PIN_TRIGGER_B 26
#define PIN_ECHO_B 18
#define PIN_TRIGGER_L 28
#define PIN_ECHO_L 19

#define EnA 8
#define In1 9
#define In2 10
#define In3 11
#define In4 12
#define EnB 13

#define Motor1EncA 2
#define Motor1EncB 3
#define Motor2EncA 4
#define Motor2EncB 5

#define I2C_SDA 20
#define I2C_SCL 21
#define I2C_ADDRESS 0x08

#define MAX_SPEED 150
#define MIN_SPEED 50
#define TURNING_SPEED 150
#define TURNING_TIME 500

#define EMERGENCY_STOP_DISTANCE_SIDES 10
#define EMERGENCY_STOP_DISTANCE_FRONT 20

Motor motorRight(1, In1, In2, EnA, Motor1EncA, Motor1EncB);
Motor motorLeft(2, In3, In4, EnB, Motor2EncA, Motor2EncB);

Sensor sensorFront(1, PIN_TRIGGER_F, PIN_ECHO_F, true);
Sensor sensorRight(2, PIN_TRIGGER_R, PIN_ECHO_R, true);
Sensor sensorBack(3, PIN_TRIGGER_B, PIN_ECHO_B, true);
Sensor sensorLeft(4, PIN_TRIGGER_L, PIN_ECHO_L, true);

bool wallFront = false;
bool wallRight = false;
bool wallLeft = false;
bool wallBack = false;

float distFront = 0;
float distRight = 0;
float distLeft = 0;
float distback = 0;

const byte arduAddress = I2C_ADDRESS; // I2C address of the Arduino board
char dataReceived[32];                // Buffer to store received data
volatile bool newDataAvailable = false;

bool stopOverwrite = false;
bool humanInFront = false;

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

  // Serial.print("Raw data received: ");
  // for (int j = 0; j < i; j++)
  // {
  //   Serial.print((char)dataReceived[j]);
  // }
  // Serial.println();
}

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
      break;
    case 101: // Disable Walldetection in Front
      humanInFront = true;
      driveRobot(0);
      Serial.println("Human detected in front, stopping robot!");
      stopOverwrite = false; // Set the stopOverwrite flag to true
      break;
    case 102: // Turn left
      driveRobot(3);
      humanInFront = false;  // Reset the humanInFront flag
      stopOverwrite = false; // Reset the stopOverwrite flag
      break;
    case 103: // Turn right
      driveRobot(4);
      humanInFront = false;  // Reset the humanInFront flag
      stopOverwrite = false; // Reset the stopOverwrite flag
      break;
    case 104: // Move forward
      driveRobot(1);
      humanInFront = false;  // Reset the humanInFront flag
      stopOverwrite = false; // Reset the stopOverwrite flag
      break;
    case 105: // Move backward
      driveRobot(2);
      humanInFront = false;  // Reset the humanInFront flag
      stopOverwrite = false; // Reset the stopOverwrite flag
      break;
    default:                 // Invalid command
      stopOverwrite = false; // Reset the stopOverwrite flag
      break;
    }
  }
}

void updateLeftEncoder()
{
  motorLeft.encoderCount_++;
  // Serial.print("Left Encoder Count: ");
  // Serial.println(motorLeft.encoderCount_);
  // Serial.print("Left Motor Speed: ");
  motorLeft.getSpeed();
}

void updateRightEncoder()
{
  motorRight.encoderCount_++;
  Serial.print("Right Encoder Count: ");
  Serial.println(motorRight.encoderCount_);
}

void setup()
{
  Serial.begin(9600);
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
  // Process I2C data if available
  processI2CData();

  // // If stopOverwrite is true skip wall detection and listen for new I2C commands
  // if (stopOverwrite)
  // {
  //   return;
  // }

  // readSensors();

  // if (wallFront || (distFront < EMERGENCY_STOP_DISTANCE_FRONT && distFront > 0))
  // {
  //   // Wall detected in front → back up
  //   Serial.println("Wall detected in front! Backing up...");
  //   backUpFromWall();
  // }
  // else if (wallRight && wallLeft)
  // {
  //   // Both sensors detect a wall – avoid the closer one
  //   if (distRight < distLeft)
  //   {
  //     // Right is closer → steer left
  //     Serial.println("Steering left to avoid wall on the right... Right closer than left");
  //     avoidWall(3);
  //   }
  //   else
  //   {
  //     // Left is closer → steer right
  //     Serial.println("Steering right to avoid wall on the left... Left closer than right");
  //     avoidWall(4);
  //   }
  // }
  // else if (wallRight)
  // {
  //   // Only right detects wall → steer left
  //   Serial.println("Steering left to avoid wall on the right...");
  //   avoidWall(3);
  // }
  // else if (wallLeft)
  // {
  //   // Only left detects wall → steer right
  //   Serial.println("Steering right to avoid wall on the left...");
  //   avoidWall(4);
  // }
  // else if (distFront > EMERGENCY_STOP_DISTANCE_FRONT &&
  //          (distRight == 0 || distRight > EMERGENCY_STOP_DISTANCE_SIDES) &&
  //          (distLeft == 0 || distLeft > EMERGENCY_STOP_DISTANCE_SIDES))
  // {
  //   // No wall approaching → wait for I2C Command
  // }
  // else
  // {
  //   // Emergency stop if any condition is unsafe
  //   motorLeft.emergencyStop();
  //   motorRight.emergencyStop();
  //   Serial.println("Emergency stop activated!");
  // }

  // Serial.println("------------------");
  delay(100);
}

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

// Reading sensors and updating the wall detection status
void readSensors()
{
  wallFront = sensorFront.isWallApproaching();
  wallRight = sensorRight.isWallApproaching();
  wallLeft = sensorLeft.isWallApproaching();
  wallBack = sensorBack.isWallApproaching();

  distFront = sensorFront.getDistance();
  distRight = sensorRight.getDistance();
  distLeft = sensorLeft.getDistance();
  distback = sensorBack.getDistance();
}

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
    Serial.println("Driving robot in direction: " + String(direction));
  }
}

// Avoid wall by turning left or right for a short time
void avoidWall(int direction)
{
  unsigned long turnStartTime = millis();

  for (; millis() - turnStartTime < TURNING_TIME;)
  {
    driveRobot(direction);
  }

  driveRobot(0);
}

// Back up from wall if detected in front. Sequence is as follows:
// 1. If Human in Front, do nothing
// 2. Stop motors for 1 second
// 3. If no wall in Back, back up for 3 seconds
// 4. If wall detected, emergency Stop
// 5. Otherwise turn randomly for 2 seconds
// 6. Stop motors
void backUpFromWall()
{
  if (humanInFront)
  {
    Serial.println("Human detected in front, not backing up!");
    return;
  }

  float distFront = sensorFront.getDistance();

  motorLeft.emergencyStop();
  motorRight.emergencyStop();
  delay(1000);

  bool wallBack = sensorBack.isWallApproaching();
  if (!wallBack)
  {
    Serial.println("Backing up...");
    unsigned long startTime = millis();

    for (; millis() - startTime < 3000;)
    {
      driveRobot(2);

      wallBack = sensorBack.isWallApproaching();
      if (wallBack)
      {
        Serial.println("Obstacle detected while backing up!");
        motorLeft.emergencyStop();
        motorRight.emergencyStop();
        break;
      }

      delay(50);
    }

    if (!wallBack)
    {
      Serial.println("Turning randomly...");
      unsigned long turnStartTime = millis();

      // Turn left or right randomly for 2 seconds
      int turnDirection = random(0, 2) == 0 ? 4 : 3; // 4 = right, 3 = left
      for (; millis() - turnStartTime < 2000;)
      {
        driveRobot(turnDirection);
        delay(50); // Small delay to avoid busy-waiting
      }

      driveRobot(0);
    }
  }
  else
  {
    Serial.println("Cannot back up, obstacle detected!");
  }
}
