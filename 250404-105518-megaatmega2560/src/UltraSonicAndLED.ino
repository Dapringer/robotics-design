#include <string.h>
#include <stdlib.h>
#include <Arduino.h>

#include "class/Motor.h"
#include "class/Sensor.h"

#define PIN_TRIGGER_F 12
#define PIN_ECHO_F 13
#define PIN_TRIGGER_B 10
#define PIN_ECHO_B 11

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

Sensor sensor1(1, PIN_TRIGGER_F, PIN_ECHO_F);
Sensor sensor2(2, PIN_TRIGGER_B, PIN_ECHO_B);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  sensor1.printStatus();

  delay(100);
}