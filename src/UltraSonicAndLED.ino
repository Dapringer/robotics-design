#include <string.h>
#include <stdlib.h>
#include <Arduino.h>

#include "class/Motor.h"
#include "class/Sensor.h"

#define SENSOR_TRGGER 21
#define PIN_TRIGGER_F 12
#define PIN_ECHO_F A1
#define PIN_TRIGGER_R 21
#define PIN_ECHO_R A2
#define PIN_TRIGGER_B 10
#define PIN_ECHO_B A3
#define PIN_TRIGGER_L 10
#define PIN_ECHO_L A4

#define EnA 2
#define In1 3
#define In2 4
#define In3 5
#define In4 6
#define EnB 7

#define MAX_SPEED 175
#define MIN_SPEED 100

using namespace std;

const int SENSOR_MAX_RANGE = 300; // in cm
const int MIN_DIST = 15;
unsigned long duration;
unsigned int distance;

Motor motorRight(1, In1, In2, EnA);
Motor motorLeft(2, In3, In4, EnB);

Sensor sensorFront(1, SENSOR_TRGGER, PIN_ECHO_F);
Sensor sensorRight(2, SENSOR_TRGGER, PIN_ECHO_R);
Sensor sensorBack(3, SENSOR_TRGGER, PIN_ECHO_B);
Sensor sensorLeft(4, SENSOR_TRGGER, PIN_ECHO_L);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // sensorFront.printStatus();
  // sensorFront.isWallApproaching();
  sensorRight.printStatus();
  sensorRight.isWallApproaching();
  // sensorBack.printStatus();
  // sensorBack.isWallApproaching();
  //sensorLeft.printStatus();
  //sensorLeft.isWallApproaching();

  if (sensorRight.isWallApproaching())
  {
    Serial.println("Sensor 2: Wall Approaching!");
    motorRight.drive(MAX_SPEED, 1);
    motorLeft.drive(MIN_SPEED, 1);
  }
  else if (sensorLeft.isWallApproaching())
  {
    // Serial.println("Sensor 4: Wall Approaching!");
    motorRight.drive(MIN_SPEED, 1);
    motorLeft.drive(MAX_SPEED, 1);
  }
  else
  {
    motorRight.drive(MAX_SPEED, 1);
    motorLeft.drive(MAX_SPEED, 1);
  };

  // Serial.println("------------------");
}