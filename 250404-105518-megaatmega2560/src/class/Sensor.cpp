#include <Arduino.h>
#include <string.h>

#include "Sensor.h"

// Constructor
Sensor::Sensor(int id, int triggerPin, int echoPin)
    : id_(id), triggerPin_(triggerPin), echoPin_(echoPin), duration_(0), distance_(0)
{
    pinMode(triggerPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
    distance_ = readDistance();
}

bool Sensor::isWallApproaching()
{
    wallApproaching_ = false;
    int distance = readDistance();
    Serial.print("Distance: ");
    Serial.println(distance);
    Serial.print("Previous Distance: ");
    Serial.println(this->distance_);

    if (this->distance_ > distance)
    {
        wallApproaching_ = true;
    }
    else
    {
        wallApproaching_ = false;
    }

    this->distance_ = distance;
    return wallApproaching_;
}

// Method to read distance
int Sensor::readDistance()
{
    digitalWrite(triggerPin_, LOW);
    delayMicroseconds(2);

    digitalWrite(triggerPin_, HIGH);
    delayMicroseconds(10);

    digitalWrite(triggerPin_, LOW);
    duration_ = pulseIn(echoPin_, HIGH);
    distance_ = duration_ / 58.0; // Convert duration to distance in cm

    return distance_;
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
    Serial.print("      |      Distance: ");
    Serial.print(readDistance());
    Serial.print("      |      Wall Approaching: ");
    Serial.println(wallApproaching_ ? "Yes" : "No");
}
