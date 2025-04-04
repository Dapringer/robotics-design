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

const int SENSOR_MAX_RANGE = 300;  // in cm
unsigned long duration;
unsigned int distance;

void setup() {
  Serial.begin(9600);

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

  int distance_front = readUltraSonic(1);
  // int distance_right = readUltraSonic(2);
  int distance_back = readUltraSonic(3);
  // int distance_left = readUltraSonic(4);

  if (!isTooNear(distance_front) && !isTooNear(distance_back)) {
    driveMotors(255, 1);
    driveMotors(255, 2);
  } else {
    emergencyBreak();
  }

  delay(100);
}

bool isTooNear(int distance) {
  if (distance > 5) {
    return false;
  } else {
    return true;
  }
}

void emergencyBreak() {
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);

  Serial.println("Emergency Stop!");
}


void driveMotors(int motorSpeed, int motorID) {
  Serial.print("Motor Speed: ");
  Serial.print(motorSpeed);
  Serial.print(" | Motor ID: ");
  Serial.println(motorID);
  switch (motorID) {
    case 1:
      digitalWrite(In1, HIGH);
      digitalWrite(In2, LOW);
      analogWrite(EnA, motorSpeed);
      break;
    case 2:
      digitalWrite(In3, HIGH);
      digitalWrite(In4, LOW);
      analogWrite(EnB, motorSpeed);
      break;
    default:
      return;
  }
}


int readUltraSonic(int sensorID) {
  switch (sensorID) {
    case 1:
      digitalWrite(PIN_TRIGGER_F, LOW);
      delayMicroseconds(2);

      digitalWrite(PIN_TRIGGER_F, HIGH);
      delayMicroseconds(10);

      duration = pulseIn(PIN_ECHO_F, HIGH);
      break;
      // case 2:
      //   digitalWrite(PIN_TRIGGER_R, LOW);
      //   delayMicroseconds(2);

      //   digitalWrite(PIN_TRIGGER_R, HIGH);
      //   delayMicroseconds(10);

      //   duration = pulseIn(PIN_ECHO_R, HIGH);
      break;
    case 3:
      digitalWrite(PIN_TRIGGER_B, LOW);
      delayMicroseconds(2);

      digitalWrite(PIN_TRIGGER_B, HIGH);
      delayMicroseconds(10);

      duration = pulseIn(PIN_ECHO_B, HIGH);
      // case 4:
      //     digitalWrite(PIN_TRIGGER_L, LOW);
      //   delayMicroseconds(2);

      //   digitalWrite(PIN_TRIGGER_L, HIGH);
      //   delayMicroseconds(10);

      //   duration = pulseIn(PIN_ECHO_L, HIGH);
      break;
    default:
      break;
  }

  distance = (duration / 58);
  Serial.print("Distance to object from Sensor ");
  Serial.print(sensorID);
  Serial.print(". Measured Distance: ");
  Serial.println(distance);

  return distance;
}
