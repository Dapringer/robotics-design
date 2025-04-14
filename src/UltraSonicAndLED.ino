#include <stdlib.h>
#include <Arduino.h>
#include <Wire.h>

#include "class/Motor.h"
#include "class/Sensor.h"

#define SENSOR_TRGGER 4
#define PIN_TRIGGER_F 12
#define PIN_ECHO_F 2
#define PIN_TRIGGER_R 21
#define PIN_ECHO_R 3
#define PIN_TRIGGER_B 21
#define PIN_ECHO_B 18
#define PIN_TRIGGER_L 21
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

#define MAX_SPEED 100
#define MIN_SPEED 50
#define EMERGENCY_STOP_DISTANCE_SIDES 5
#define EMERGENCY_STOP_DISTANCE_FRONT 15

const int SENSOR_MAX_RANGE = 300; // in cm
const int MIN_DIST = 15;
unsigned long duration;
unsigned int distance;

Motor motorRight(1, In1, In2, EnA);
Motor motorLeft(2, In3, In4, EnB);

Sensor sensorFront(1, SENSOR_TRGGER, PIN_ECHO_F, true);
Sensor sensorRight(2, SENSOR_TRGGER, PIN_ECHO_R, true);
Sensor sensorBack(3, SENSOR_TRGGER, PIN_ECHO_B, false);
Sensor sensorLeft(4, SENSOR_TRGGER, PIN_ECHO_L, true);

const byte arduAddress = I2C_ADDRESS; // I2C address of the Arduino board
bool dataReady = false;
int dataReceived = 0;

void handleI2CInterrupt()
{
  printf("I2C interrupt triggered\n");
  dataReady = true;
}

void processI2CData()
{
  if (dataReady) // Check if data is ready to be processed
  {
    dataReady = false; // Reset the flag

    if (Wire.available())
    { // Check if data is available to read
      /**
       * Optionally, add a if statement that says "if (dataReceived == 1) { ... }"
       * to handle specific data received from the I2C bus.
       * */
      dataReceived = Wire.read(); // Read the data from the I2C bus
      Serial.print("Data received: ");
      Serial.println(dataReceived);

      if (dataReceived)
      {
        // Check if the received data is a command to stop the motors
        switch (dataReceived)
        {
        case 0: // Stop both motors
          driveRobot(0);
          break;
        case 1: // Move forward
          driveRobot(1);
          break;
        case 2: // Move backward
          driveRobot(2);
          break;
        case 3: // Turn left
          driveRobot(3);
          break;
        case 4: // Turn right
          driveRobot(4);
          break;
        default: // Invalid command
          Serial.println("Invalid command received!");
          break;
        }
      }
    }

    dataReady = false;
  }
}

void setup()
{
  Serial.begin(9600);
  // Wire.begin(arduAddress);                                                                // Initialize I2C with specified SDA and SCL pins
  // pinMode(I2C_INTERRUPT_PIN, INPUT_PULLUP);                                               // Set the interrupt pin as input with pull-up resistor
  // attachInterrupt(digitalPinToInterrupt(I2C_INTERRUPT_PIN), handleI2CInterrupt, FALLING); // Attach interrupt to I2C interrupt pin
}

void loop()
{

  // processI2CData(); // Process I2C data if available

  bool wallFront = sensorFront.isWallApproaching();
  bool wallRight = sensorRight.isWallApproaching();
  bool wallLeft = sensorLeft.isWallApproaching();

  float distFront = sensorFront.getDistance();
  float distRight = sensorRight.getDistance();
  float distLeft = sensorLeft.getDistance();

  if (wallFront || distFront < EMERGENCY_STOP_DISTANCE_FRONT)
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
      driveRobot(3);
    }
    else
    {
      // Left is closer → steer right
      driveRobot(4);
    }
  }
  else if (wallRight)
  {
    // Only right detects wall → steer left
    driveRobot(3);
  }
  else if (wallLeft)
  {
    // Only left detects wall → steer right
    driveRobot(4);
  }
  else if (distFront > EMERGENCY_STOP_DISTANCE_FRONT &&
           (distRight == 0 || distRight > EMERGENCY_STOP_DISTANCE_SIDES) &&
           (distLeft == 0 || distLeft > EMERGENCY_STOP_DISTANCE_SIDES))
  {
    // No wall approaching → go straight
    driveRobot(1);
  }
  else
  {
    // Emergency stop if any condition is unsafe
    motorLeft.emergencyStop();
    motorRight.emergencyStop();
    Serial.println("Emergency stop activated!");
  }

  Serial.println("------------------");
  delay(50);
}

void driveRobot(int direction)
{
  if (direction == 3)
  {
    motorLeft.drive(MIN_SPEED, 1);
    motorRight.drive(MAX_SPEED, 1);
  }
  else if (direction == 4)
  {
    motorLeft.drive(MAX_SPEED, 1);
    motorRight.drive(MIN_SPEED, 1);
  }
  else
  {
    motorLeft.drive(MAX_SPEED, direction);
    motorRight.drive(MAX_SPEED, direction);
  }
}

void backUpFromWall()
{
  float distFront = sensorFront.getDistance();

  if (distFront > MIN_DIST)
  {
    driveRobot(1);
  }
  else
  {
    // Stop when closer than 15 cm
    motorLeft.emergencyStop();
    motorRight.emergencyStop();
    delay(1000);

    bool wallBack = sensorBack.isWallApproaching();
    if (!wallBack)
    {
      Serial.println("Backing up...");
      unsigned long startTime = millis();

      for (; millis() - startTime < 2000;)
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
}