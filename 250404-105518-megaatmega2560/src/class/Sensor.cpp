#include <Arduino.h>
#include <string.h>

#include "Sensor.h"

// Constructor
Sensor::Sensor(int id, int triggerPin, int echoPin)
    : id_(id), triggerPin_(triggerPin), echoPin_(echoPin), duration_(0), distance_(0), previousDistance_(0)
{
    pinMode(triggerPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
}

bool Sensor::isWallApproaching()
{
    readDistance();
    if (previousDistance_ > distance_)
    {
        wallApproaching_ = true;
    }
    else
    {
        wallApproaching_ = false;
    }

    previousDistance_ = distance_;
    return wallApproaching_;
}

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
void Sensor::printStatus()
{
    Serial.print("      Sensor ID: ");
    Serial.print(id_);
    Serial.print("      |      Trigger Pin: ");
    Serial.print(triggerPin_);
    Serial.print("      |      Echo Pin: ");
    Serial.print(echoPin_);
    Serial.print("      |      Duration: ");
    Serial.print(duration_);
    Serial.print("ms      |      Distance: ");
    Serial.print(distance_);
    Serial.print("cm      |      Wall Approaching: ");
    Serial.println(wallApproaching_ ? "Yes" : "No");
}
