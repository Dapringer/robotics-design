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

#define MAX_SPEED 150
#define MIN_SPEED 75
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
  // sensorFront.printStatus();
  // sensorFront.isWallApproaching();
  // sensorRight.printStatus();
  // sensorRight.isWallApproaching();
  // sensorBack.printStatus();
  // sensorBack.isWallApproaching();
  // sensorLeft.printStatus();
  // sensorLeft.isWallApproaching();

  processI2CData(); // Process I2C data if available

  delay(50);
}