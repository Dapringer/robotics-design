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
#define EMERGENCY_STOP_DISTANCE 5

const int SENSOR_MAX_RANGE = 300; // in cm
const int MIN_DIST = 15;
unsigned long duration;
unsigned int distance;

Motor motorRight(1, In1, In2, EnA);
Motor motorLeft(2, In3, In4, EnB);

Sensor sensorFront(1, SENSOR_TRGGER, PIN_ECHO_F, true);
Sensor sensorRight(2, SENSOR_TRGGER, PIN_ECHO_R, true);
Sensor sensorBack(3, SENSOR_TRGGER, PIN_ECHO_B, true);
Sensor sensorLeft(4, SENSOR_TRGGER, PIN_ECHO_L, false);

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

      if (dataReceived) {
        // Check if the received data is a command to stop the motors
        if (dataReceived == 0) {
          motorRight.emergencyStop(); // Stop the right motor
          motorLeft.emergencyStop();  // Stop the left motor
          Serial.println("Motors stopped due to I2C command.");
        } 
      }
    }

    dataReady = false;
  }
}

void setup()
{
  Serial.begin(9600);
  Wire.begin(arduAddress);                                                                // Initialize I2C with specified SDA and SCL pins
  pinMode(I2C_INTERRUPT_PIN, INPUT_PULLUP);                                               // Set the interrupt pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(I2C_INTERRUPT_PIN), handleI2CInterrupt, FALLING); // Attach interrupt to I2C interrupt pin
}

void loop()
{

  processI2CData(); // Process I2C data if available

  bool wallFront = sensorFront.isWallApproaching();
  bool wallRight = sensorRight.isWallApproaching();
  bool wallBack = sensorBack.isWallApproaching();
  bool wallLeft = sensorLeft.isWallApproaching();

  float distFront = sensorFront.getDistance();
  float distRight = sensorRight.getDistance();
  float distBack = sensorBack.getDistance();
  float distLeft = sensorLeft.getDistance();

  if (wallFront)
  {
    // Approach the wall slowly until closer than 15 cm
    if (distFront > MIN_DIST)
    {
      motorLeft.drive(MIN_SPEED, 1);
      motorRight.drive(MIN_SPEED, 1);
    }
    else
    {
      // Stop when closer than 15 cm
      motorLeft.emergencyStop();
      motorRight.emergencyStop();
      delay(1000);

      // Check if there's space to back up
      if (!wallBack)
      {
        // Back up
        motorLeft.drive(MAX_SPEED, 2);
        motorRight.drive(MAX_SPEED, 2);
        delay(2000);

        // Turn in a random direction
        if (random(0, 2) == 0)
        {
          // Turn right
          motorLeft.drive(MAX_SPEED, 1);
          motorRight.drive(MIN_SPEED, 1);
        }
        else
        {
          // Turn left
          motorLeft.drive(MIN_SPEED, 1);
          motorRight.drive(MAX_SPEED, 1);
        }
        delay(1000);

        // Stop after turning
        motorLeft.drive(0, 0);
        motorRight.drive(0, 0);
      }
      else
      {
        Serial.println("Cannot back up, obstacle detected!");
      }
    }
  }
  else if (wallRight && wallLeft)
  {
    // Both sensors detect a wall – avoid the closer one
    if (distRight < distLeft)
    {
      // Right is closer → steer left
      motorRight.drive(MAX_SPEED, 1);
      motorLeft.drive(MIN_SPEED, 1);
    }
    else
    {
      // Left is closer → steer right
      motorRight.drive(MIN_SPEED, 1);
      motorLeft.drive(MAX_SPEED, 1);
    }
  }
  else if (wallRight)
  {
    // Only right detects wall → steer left
    motorRight.drive(MAX_SPEED, 1);
    motorLeft.drive(MIN_SPEED, 1);
  }
  else if (wallLeft)
  {
    // Only left detects wall → steer right
    motorRight.drive(MIN_SPEED, 1);
    motorLeft.drive(MAX_SPEED, 1);
  }
  else if (distFront > 0 && distFront > EMERGENCY_STOP_DISTANCE && distRight > EMERGENCY_STOP_DISTANCE && distLeft > EMERGENCY_STOP_DISTANCE)
  {
    // No wall approaching → go straight
    motorRight.drive(MAX_SPEED, 1);
    motorLeft.drive(MAX_SPEED, 1);
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