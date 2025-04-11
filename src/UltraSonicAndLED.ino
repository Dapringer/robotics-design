#include <string.h>
#include <stdlib.h>
#include <Arduino.h>

#include "class/Motor.h"
#include "class/Sensor.h"

#define SENSOR_TRGGER 21
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

#define MAX_SPEED 150
#define MIN_SPEED 75
#define EMERGENCY_STOP_DISTANCE 5

using namespace std;

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

void setup()
{
  Serial.begin(9600);
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

  bool wallFront = sensorFront.isWallApproaching();
  bool wallRight = sensorRight.isWallApproaching();
  bool wallLeft = sensorLeft.isWallApproaching();
  float distFront = sensorFront.getDistance();
  float distRight = sensorRight.getDistance();
  float distLeft = sensorLeft.getDistance();

  if ((distRight > 0 && distRight <= EMERGENCY_STOP_DISTANCE) || (distLeft > 0 && distLeft <= EMERGENCY_STOP_DISTANCE) || (distFront > 0 && distFront <= 10))
  {
    motorLeft.emergencyStop();
    motorRight.emergencyStop();
    Serial.println("Emergency stop activated!");
    return;
  }

  if (wallRight && wallLeft)
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
  else
  {
    // No wall approaching → go straight
    motorRight.drive(MAX_SPEED, 1);
    motorLeft.drive(MAX_SPEED, 1);
  }

  Serial.println("------------------");
  delay(50);
}