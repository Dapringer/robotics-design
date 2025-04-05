#include <string.h>
#include <stdlib.h>
#include <Arduino.h>

#include "Motor.h"
#include "Sensor.h"

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

const int SENSOR_MAX_RANGE = 300;  // in cm
const int MIN_DIST = 15;
unsigned long duration;
unsigned int distance;

void setup() {
  Serial.begin(9600);

  Motor motor1(In1, In2, EnA);
  Motor motor2(In3, In4, EnB);

  Sensor sensor1(PIN_TRIGGER_F, PIN_ECHO_F);
  Sensor sensor2(PIN_TRIGGER_B, PIN_ECHO_B);

  sensor1.printStatus();
  sensor2.printStatus();

  pinMode(PIN_TRIGGER_F, OUTPUT);
  pinMode(PIN_ECHO_F, INPUT);
  // pinMode(PIN_TRIGGER_R, OUTPUT);
  // pinMode(PIN_ECHO_R, INPUT);
  pinMode(PIN_TRIGGER_B, OUTPUT);
  pinMode(PIN_ECHO_B, INPUT);
  // pinMode(PIN_TRIGGER_L, OUTPUT);
  // pinMode(PIN_ECHO_L, INPUT);


  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(EnB, OUTPUT);
}

void loop() {

  

  delay(100);
}