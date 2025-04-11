#include <Arduino.h>
#include <string.h>

#include "Sensor.h"

// Constructor
Sensor::Sensor(int id, int triggerPin, int echoPin)
    : id_(id), triggerPin_(triggerPin), echoPin_(echoPin), duration_(0), distance_(0), measurementIndex_(0)
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
    if (distance_ > 50 || distance_ <= 0)
    {
        Serial.println("Invalid distance measurement. Ignoring.");
        return false;
    }

    measurements_[measurementIndex_] = distance_;
    measurementIndex_ = (measurementIndex_ + 1) % 5; // Circular buffer

    // Calculate the trend as total change and count how many drops
    int drops = 0;
    float totalTrend = 0;

    int newestIndex = (measurementIndex_ - 1 + 5) % 5; // Most recent value

    // Loop over all 4 steps (between 5 values)
    for (int i = 0; i < 4; i++)
    {
        int index1 = (measurementIndex_ + i) % 5;
        int index2 = (measurementIndex_ + i + 1) % 5;
        float stepChange = measurements_[index2] - measurements_[index1];
        if (stepChange < 0)
            drops++; // Count drops (negative changes)

        totalTrend += stepChange;
    }

    // Calculate average step change
    float avgStepChange = totalTrend / 4.0;

    // Print the trend info
    Serial.print("Measurement Index: ");
    Serial.print(measurementIndex_);
    Serial.print(" | Newest Index: ");
    Serial.print(newestIndex);
    Serial.print(" | Measurements: ");
    for (int i = 0; i < 5; i++)
    {
        Serial.print(measurements_[i]);
        if (i < 4)
        {
            Serial.print(", ");
        }
    }
    Serial.print(" | Trend: ");
    Serial.print(avgStepChange);
    Serial.print(" | Drops: ");
    Serial.print(drops);
    Serial.print(" | Distance: ");
    Serial.print(distance_);
    Serial.print(" | ");

    // Smart detection criteria:
    // If we have 3 or more drops and the average step change is negative enough, it's a wall.
    if (drops >= 3 && avgStepChange < -1 && distance_ < 50)
    {
        wallApproaching_ = true;
        Serial.println("Wall approaching detected (smart mode)!");
    }
    else
    {
        wallApproaching_ = false;
        Serial.println("No wall detected.");
    }

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

unsigned int Sensor::getDistance() const
{
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
    Serial.print("ms      |      Distance: ");
    Serial.print(distance_);
    Serial.print("cm      |      Wall Approaching: ");
    Serial.println(wallApproaching_ ? "Yes" : "No");
}
