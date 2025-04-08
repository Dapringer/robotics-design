#include <Arduino.h>
#include <string.h>

#include "Sensor.h"

// Constructor
Sensor::Sensor(int id, int triggerPin, int echoPin)
    : id_(id), triggerPin_(triggerPin), echoPin_(echoPin), duration_(0), distance_(0), previousDistance_(0), measurementIndex_(0)
{
    pinMode(triggerPin_, OUTPUT);
    pinMode(echoPin_, INPUT);

    // Initialize the measurement array
    for (int i = 0; i < 5; i++)
    {
        measurements_[i] = 0;
    }
}

bool Sensor::isWallApproaching()
{
    

    readDistance();
    // Update the measurement array
    measurements_[measurementIndex_] = distance_;
    measurementIndex_ = (measurementIndex_ + 1) % 5; // Circular buffer

    // Calculate the trend
    float trend = 0;
    for (int i = 1; i < 5; i++)
    {
        trend += measurements_[i] - measurements_[i - 1];
    }

    // Early exit: Significant drop in distance
    if (distance_ < 15 && distance_ > 0) // Immediate detection for very close objects
    {
        Serial.println("Immediate wall detected!");
        wallApproaching_ = true;
        return wallApproaching_;
    }

    // Persist the trend detection
    static int consecutiveTrendCount = 0; // Track consecutive trend detections
    if (trend < -1 && distance_ < 60 && distance_ > 0)
    {
        consecutiveTrendCount++;
        if (consecutiveTrendCount >= 2) // Require 2 consecutive detections
        {
            wallApproaching_ = true;
        }
    }
    else
    {
        consecutiveTrendCount = 0; // Reset if trend is not detected
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
