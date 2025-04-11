#include <Arduino.h>
#include <string.h>

#include "Sensor.h"

Sensor *Sensor::sensors_[4] = {nullptr, nullptr, nullptr, nullptr}; // Initialize the static array

volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd = 0;
volatile bool echoDone = false;

// Constructor
Sensor::Sensor(int id, int triggerPin, int echoPin, bool useInterrupt)
    : id_(id), triggerPin_(triggerPin), echoPin_(echoPin), useInterrupt_(useInterrupt), duration_(0), distance_(0), measurementIndex_(0)
{
    pinMode(triggerPin_, OUTPUT);
    pinMode(echoPin_, INPUT);

    for (int i = 0; i < 5; i++)
    {
        measurements_[i] = 0;
    }

    if (useInterrupt_)
    {
        registerSensor(this); // Register this sensor for interrupts
        switch (id_)
        {
        case 0:
            attachInterrupt(digitalPinToInterrupt(echoPin_), echoISR0, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(echoPin_), echoISR1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(echoPin_), echoISR2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(echoPin_), echoISR3, CHANGE);
            break;
        }
    }
}

// Register a sensor
void Sensor::registerSensor(Sensor *sensor)
{
    if (sensor->id_ >= 0 && sensor->id_ < 4)
    {
        sensors_[sensor->id_] = sensor;
    }
    else
    {
        Serial.println("Error: Sensor ID out of range!");
    }
}

// Handle interrupt for a specific sensor
void Sensor::handleInterrupt(int sensorId)
{
    if (sensors_[sensorId] != nullptr)
    {
        Sensor *sensor = sensors_[sensorId];
        if (digitalRead(sensor->getEchoPin()) == HIGH)
        {
            sensor->duration_ = micros();
            // Serial.print("Sensor ");
            // Serial.print(sensorId);
            // Serial.println(": Echo start detected");
        }
        else
        {
            sensor->distance_ = (micros() - sensor->duration_) / 58.0;
            // Serial.print("Sensor ");
            // Serial.print(sensorId);
            // Serial.print(": Distance calculated: ");
            // Serial.println(sensor->distance_);
        }
    }
}

// ISR for sensor 0
void Sensor::echoISR0()
{
    handleInterrupt(0);
}

// ISR for sensor 1
void Sensor::echoISR1()
{
    handleInterrupt(1);
}

// ISR for sensor 2
void Sensor::echoISR2()
{
    handleInterrupt(2);
}

// ISR for sensor 3
void Sensor::echoISR3()
{
    handleInterrupt(3);
}

// Trigger method for interrupt-based sensors
void Sensor::trigger()
{
    digitalWrite(triggerPin_, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin_, LOW);
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
    if (useInterrupt_)
    {
        trigger();
    }
    else
    {
        digitalWrite(triggerPin_, LOW);
        delayMicroseconds(2);

        digitalWrite(triggerPin_, HIGH);
        delayMicroseconds(10);

        digitalWrite(triggerPin_, LOW);
        duration_ = pulseIn(echoPin_, HIGH);
        distance_ = duration_ / 58.0; // Convert duration to distance in cm
    }
}

unsigned int Sensor::getDistance() const
{
    return distance_;
}

// Getter for echoPin_
int Sensor::getEchoPin() const
{
    return echoPin_;
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
