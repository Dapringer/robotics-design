#include <Arduino.h>
#include <string.h>

#include "Sensor.h"

// Constructor
Sensor::Sensor(int id, int triggerPin, int echoPin)
    : id_(id), triggerPin_(triggerPin), echoPin_(echoPin), duration_(0), distance_(0)
{
    pinMode(triggerPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
}

// Getter methods
int Sensor::getId() const { return id_; }
int Sensor::getTriggerPin() const { return triggerPin_; }
int Sensor::getEchoPin() const { return echoPin_; }
unsigned long Sensor::getDuration() const { return duration_; }
float Sensor::getDistance() const { return distance_; }

// Method to read distance
void Sensor::readDistance()
{
    digitalWrite(triggerPin_, LOW);
    delayMicroseconds(2);

    digitalWrite(triggerPin_, HIGH);
    delayMicroseconds(10);

    digitalWrite(triggerPin_, LOW);
    duration_ = pulseIn(echoPin_, HIGH);
    distance_ = duration_ / 58.0; // Convert duration to distance in cm
}

// Method to print sensor status
void Sensor::printStatus() const
{
    Serial.print("Sensor ID: ");
    Serial.print(id_);
    Serial.print(", Trigger Pin: ");
    Serial.print(triggerPin_);
    Serial.print(", Echo Pin: ");
    Serial.print(echoPin_);
    Serial.print(", Duration: ");
    Serial.print(duration_);
    Serial.print(", Distance: ");
    Serial.println(distance_);
}
