#include <stdlib.h>
#include <Arduino.h>
#include <Wire.h>

#include "class/Motor.h"
#include "class/Sensor.h"

#define SENSOR_TRGGER 4
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

#define I2C_SDA 20
#define I2C_SCL 21
#define I2C_ADDRESS 0x08
#define I2C_INTERRUPT_PIN 18

#define I2C_SDA 20
#define I2C_SCL 21
#define I2C_ADDRESS 0x08
#define I2C_INTERRUPT_PIN 18

#define MAX_SPEED 150
#define MIN_SPEED 100
#define TURNING_SPEED 150
#define TURNING_TIME 500

#define EMERGENCY_STOP_DISTANCE_SIDES 10
#define EMERGENCY_STOP_DISTANCE_FRONT 20


// const int SENSOR_MAX_RANGE = 300; // in cm
const int MIN_DIST = 15;
unsigned long duration;
unsigned int distance;

Motor motorRight(1, In1, In2, EnA);
Motor motorLeft(2, In3, In4, EnB);

Sensor sensorFront(1, PIN_TRIGGER_F, PIN_ECHO_F, true);
Sensor sensorRight(2, PIN_TRIGGER_R, PIN_ECHO_R, true);
Sensor sensorBack(3, PIN_TRIGGER_B, PIN_ECHO_B, true);
Sensor sensorLeft(4, PIN_TRIGGER_L, PIN_ECHO_L, true);

const byte arduAddress = I2C_ADDRESS; // I2C address of the Arduino board
int dataReceived = 0;
volatile bool newDataAvailable = false;
bool stopOverwrite = false;

bool humanInFront = false;

void receiveEvent(int howMany)
{
  if (Wire.available())
  {
    dataReceived = Wire.read(); // Receive one byte as an integer
    newDataAvailable = true;
  }
}

void processI2CData()
{
  if (newDataAvailable)
  {
    newDataAvailable = false; // Reset the flag immediately
    Serial.print("Arduino received (polling): ");
    Serial.println(dataReceived);

    // Execute different functions based on the dataReceived
    if (dataReceived > -1)
    {
      // Check if the received data is a command to stop the motors
      switch (dataReceived)
      {
      case 0: // Stop both motors
        driveRobot(0);
        humanInFront = false; // Reset the humanInFront flag
        stopOverwrite = true; // Set the stopOverwrite flag to true
        break;
      case 1: // Move forward
        driveRobot(1);
        humanInFront = false;  // Reset the humanInFront flag
        stopOverwrite = false; // Reset the stopOverwrite flag
        break;
      case 2: // Move backward
        driveRobot(2);
        humanInFront = false;  // Reset the humanInFront flag
        stopOverwrite = false; // Reset the stopOverwrite flag
        break;
      case 3: // Turn left
        driveRobot(3);
        humanInFront = false;  // Reset the humanInFront flag
        stopOverwrite = false; // Reset the stopOverwrite flag
        break;
      case 4: // Turn right
        driveRobot(4);
        humanInFront = false;  // Reset the humanInFront flag
        stopOverwrite = false; // Reset the stopOverwrite flag
        break;
      case 10: // Disable Walldetection in Front
        humanInFront = true;
        driveRobot(0);
        Serial.println("Human detected in front, stopping robot!");
        stopOverwrite = false; // Set the stopOverwrite flag to true
        break;
      default: // Invalid command
        Serial.println("Invalid command received!");
        stopOverwrite = false; // Reset the stopOverwrite flag
        break;
      }
    }
  }
}

void setup()
{
  Serial.begin(9600);
  Wire.begin(arduAddress);      // Join the I2C bus as a slave
  Wire.onReceive(receiveEvent); // Register the function for received data
  Serial.println("Arduino I2C Slave (Polling) ready...");
}

void loop()
{

  processI2CData(); // Process I2C data if available

  if (stopOverwrite)
  {
    // If stopOverwrite is true, stop the robot and skip wall detection
    return;
  }

  bool wallFront = sensorFront.isWallApproaching();
  bool wallRight = sensorRight.isWallApproaching();
  bool wallLeft = sensorLeft.isWallApproaching();
  bool wallBack = sensorBack.isWallApproaching();

  float distFront = sensorFront.getDistance();
  float distRight = sensorRight.getDistance();
  float distLeft = sensorLeft.getDistance();
  float distback = sensorBack.getDistance();

  if (wallFront || (distFront < EMERGENCY_STOP_DISTANCE_FRONT && distFront > 0))
  {
    // Wall detected in front → back up
    Serial.println("Wall detected in front! Backing up...");
    backUpFromWall();
  }
  else if (humanInFront && wallFront)
  {
    Serial.println("Wall detected in front! Backing up...");
    backUpFromWall();
  }
  else if (wallRight && wallLeft)
  {
    // Both sensors detect a wall – avoid the closer one
    if (distRight < distLeft)
    {
      // Right is closer → steer left
      Serial.println("Steering left to avoid wall on the right... Right closer than left");
      turnAwayFromWall(3);
    }
    else
    {
      // Left is closer → steer right
      Serial.println("Steering right to avoid wall on the left... Left closer than right");
      turnAwayFromWall(4);
    }
  }
  else if (wallRight)
  {
    // Only right detects wall → steer left
    Serial.println("Steering left to avoid wall on the right...");
    turnAwayFromWall(3);
  }
  else if (wallLeft)
  {
    // Only left detects wall → steer right
    Serial.println("Steering right to avoid wall on the left...");
    turnAwayFromWall(4);
  }
  else if (distFront > EMERGENCY_STOP_DISTANCE_FRONT &&
           (distRight == 0 || distRight > EMERGENCY_STOP_DISTANCE_SIDES) &&
           (distLeft == 0 || distLeft > EMERGENCY_STOP_DISTANCE_SIDES))
  {
    // No wall approaching → wait for I2C Command
  }
  else
  {
    // Emergency stop if any condition is unsafe
    motorLeft.emergencyStop();
    motorRight.emergencyStop();
    Serial.println("Emergency stop activated!");
  }

  // Serial.println("------------------");
  delay(50);
}

void driveRobot(int direction)
{
  if (direction == 3)
  {
    motorLeft.drive(0, 2);
    motorRight.drive(TURNING_SPEED, 1);
  }
  else if (direction == 4)
  {
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

void turnAwayFromWall(int direction)
{
  unsigned long turnStartTime = millis();

  // Turn left or right randomly for 2 seconds
  for (; millis() - turnStartTime < TURNING_TIME;)
  {
    driveRobot(direction);
  }

  driveRobot(0); // Stop after turning
}

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

      driveRobot(0); // Stop after turning
    }
  }
  else
  {
    Serial.println("Cannot back up, obstacle detected!");
  }
}
