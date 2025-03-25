#define PIN_TRIGGER 12
#define PIN_ECHO 11
#define LED_PIN 13

#define EnA 10
#define EnB 5
#define In1 9
#define In2 8
#define In3 7
#define In4 6



const int SENSOR_MAX_RANGE = 300;  // in cm
unsigned long duration;
unsigned int distance;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_TRIGGER, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(LED_PIN, OUTPUT);


  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);

}

void loop() {
  int distance = detectDistance();

  if (distance > 50 && distance < SENSOR_MAX_RANGE) {
    digitalWrite(LED_PIN, LOW);
    driveMotors(255, 1);
    driveMotors(255, 2);
  } else {
    emergencyBreak();
  }

  delay(100);
}

void emergencyBreak() {
  Serial.print("Emergency Stop!");
  digitalWrite(LED_PIN, HIGH);

  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}


int detectDistance() {
  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_TRIGGER, HIGH);
  delayMicroseconds(10);

  duration = pulseIn(PIN_ECHO, HIGH);
  distance = duration / 58;
  
  Serial.println("Distance to object: " + String(distance) + " cm");
  return distance;
}


void driveMotors(int motorSpeed, int motorID) {
  switch (motorID) {
    case 1:
      digitalWrite(In1, HIGH);
        digitalWrite(In2, LOW);
      analogWrite(EnA, motorSpeed);
    case 2:
      digitalWrite(In3, HIGH);
        digitalWrite(In4, LOW);
      analogWrite(EnB, motorSpeed);
    default:
      return;
  }
}
