#include <string.h>

#define PIN_TRIGGER_F 12
#define PIN_ECHO_F 13
#define PIN_TRIGGER_R 8
#define PIN_ECHO_R 9
#define PIN_TRIGGER_B 10
#define PIN_ECHO_B 11
// #define PIN_TRIGGER_B 10
// #define PIN_ECHO_B 11

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

struct Motor {
  int id;
  int enPin;
  int inPin1;
  int inPin2;
} motor1, motor2;


struct UltraSonicSensor {
  int id;
  int triggerPin;
  int echoPin;
} us_front, us_right, us_back, us_left;


void setup() {
  Serial.begin(9600);


  motor1 = { 1, EnA, In1, In2 };
  motor2 = { 2, EnB, In3, In4 };

  us_front = { 1, PIN_TRIGGER_F, PIN_ECHO_F };
  us_right = { 2, PIN_TRIGGER_R, PIN_ECHO_R };
  us_back = { 3, PIN_TRIGGER_B, PIN_ECHO_B };

  pinMode(us_front.triggerPin, OUTPUT);
  pinMode(us_front.echoPin, INPUT);
  pinMode(us_right.triggerPin, OUTPUT);
  pinMode(us_right.echoPin, INPUT);
  pinMode(us_back.triggerPin, OUTPUT);
  pinMode(us_back.echoPin, INPUT);
  // pinMode(PIN_TRIGGER_L, OUTPUT);
  // pinMode(PIN_ECHO_L, INPUT);


  pinMode(motor1.enPin, OUTPUT);
  pinMode(motor1.inPin1, OUTPUT);
  pinMode(motor1.inPin2, OUTPUT);
  pinMode(motor2.inPin1, OUTPUT);
  pinMode(motor2.inPin2, OUTPUT);
  pinMode(motor2.enPin, OUTPUT);
}

void loop() {

  int distance_front = readUltraSonic(us_front);
  int distance_right = readUltraSonic(us_right);
  int distance_back = readUltraSonic(us_back);
  // int distance_left = readUltraSonic(us_left);

  if (!isTooNear(distance_front) && !isTooNear(distance_right) && !isTooNear(distance_back)) {
    driveMotors(97, motor1, 0);
    driveMotors(100, motor2, 0);
  } else {
    emergencyBreak();
  }

  delay(100);
}

bool isTooNear(int distance) {
  if (distance < MIN_DIST) {
    return true;
  } else {
    return false;
  }
}

void emergencyBreak() {
  driveMotors(97, motor1, 1);
  driveMotors(100, motor2, 1);

  Serial.println("Emergency Stop!");
}


void driveMotors(int motorSpeed, struct Motor m, int direction) {
  Serial.print("Motor Speed: ");
  Serial.print(motorSpeed);
  Serial.print(" | Motor ID: ");
  Serial.print(m.id);
  Serial.print(" | Motor Direction: ");
  Serial.println(direction);
  switch (direction) {
    case 0:
      digitalWrite(m.inPin1, HIGH);
      digitalWrite(m.inPin2, LOW);
      analogWrite(m.enPin, motorSpeed);
      break;
    case 1:
      digitalWrite(m.inPin1, LOW);
      digitalWrite(m.inPin2, HIGH);
      analogWrite(m.enPin, motorSpeed);
      break;
    default:
      return;
  }
}


int readUltraSonic(struct UltraSonicSensor us) {
  digitalWrite(us.triggerPin, LOW);
  delayMicroseconds(2);

  digitalWrite(us.triggerPin, HIGH);
  delayMicroseconds(10);

  duration = pulseIn(us.echoPin, HIGH);

  distance = (duration / 58);
  Serial.print("Distance to object from Sensor ");
  Serial.print(us.id);
  Serial.print(". Measured Distance: ");
  Serial.println(distance);

  return distance;
}
