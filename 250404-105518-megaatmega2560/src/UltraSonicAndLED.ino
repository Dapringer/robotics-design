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
#define PIN_ECHO_L A8

#define EnA 2
#define In1 3
#define In2 4
#define In3 5
#define In4 6
#define EnB 7

using namespace std;

const int SENSOR_MAX_RANGE = 300; // in cm
const int MIN_DIST = 15;
unsigned long duration;
unsigned int distance;

Motor motor1(1, In1, In2, EnA);
Motor motor2(2, In3, In4, EnB);

Sensor sensor1(1, SENSOR_TRGGER, PIN_ECHO_F);
Sensor sensor2(2, SENSOR_TRGGER, PIN_ECHO_R);
Sensor sensor3(3, SENSOR_TRGGER, PIN_ECHO_B);
Sensor sensor4(4, SENSOR_TRGGER, PIN_ECHO_L);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  sensor1.printStatus();
  sensor1.isWallApproaching();
  sensor2.printStatus();
  sensor2.isWallApproaching();
  sensor3.printStatus();
  sensor3.isWallApproaching();
  sensor4.printStatus();
  sensor4.isWallApproaching();

  motor1.drive(97, 1);
  motor2.drive(100, 1);
  Serial.println("------------------");
  delay(100);
}