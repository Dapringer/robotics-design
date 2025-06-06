/**
 * Sensor.cpp
 *
 * Implementation of the Sensor class for ultrasonic distance measurement
 * and wall approach detection.
 *
 * @author: Movement and Localization Outdoor Robot Team
 */
#include <Arduino.h>
#include <string.h>

#include "Sensor.h"

// Wall detection algorithm constants
const float MIN_AVERAGE_STEP_CHANGE = -1.0; // Minimum step change to indicate approaching wall
const int MINIMUM_DROPS = 5;                // Minimum number of drops to indicate approaching wall
const int MAXIMUM_DISTANCE = 100;           // Maximum valid distance in cm
const int MINIMUM_DISTANCE = 10;            // Minimum valid distance in cm
int MEASUREMENTS_SIZE = 10;                 // Size of measurement buffer

// Initialize the static array of sensors
Sensor *Sensor::sensors_[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};

// Variables for interrupt-based measurement
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd = 0;
volatile bool echoDone = false;

/**
 * Constructor for the Sensor class
 * Initializes sensor pins and variables, and sets up interrupts if needed
 *
 * @param id Unique identifier for the sensor (1-4)
 * @param triggerPin Pin for triggering the ultrasonic sensor
 * @param echoPin Pin for receiving the echo signal
 * @param useInterrupt Flag to use interrupt-based measurement
 */
Sensor::Sensor(int id, int triggerPin, int echoPin, bool useInterrupt)
    : id_(id), triggerPin_(triggerPin), echoPin_(echoPin), useInterrupt_(useInterrupt), duration_(0), distance_(0), measurementIndex_(0)
{
    // Configure pin modes
    pinMode(triggerPin_, OUTPUT);
    pinMode(echoPin_, INPUT);

    // Initialize measurement buffer
    MEASUREMENTS_SIZE = sizeof(measurements_) / sizeof(measurements_[0]);
    for (size_t i = 0; i < MEASUREMENTS_SIZE; i++)
    {
        measurements_[i] = 0;
    }

    // Set up interrupts if requested
    if (useInterrupt_)
    {
        registerSensor(this); // Register this sensor for interrupts

        // Attach the appropriate interrupt handler based on sensor ID
        switch (id_)
        {
        case 1:
            attachInterrupt(digitalPinToInterrupt(echoPin_), echoISR1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(echoPin_), echoISR2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(echoPin_), echoISR3, CHANGE);
            break;
        case 4:
            attachInterrupt(digitalPinToInterrupt(echoPin_), echoISR4, CHANGE);
            break;
        }
    }
}

// Register a sensor
// This method is called in the constructor to register the sensor in the static array
// The sensor ID must be between 1 and 4 (inclusive) to be valid
// If the ID is out of range, an error message is printed to the Serial Monitor
// The static array is used to store the sensor objects for later use in the interrupt handlers
// The static array is initialized to nullptr for all elements, and the constructor fills it with the sensor objects
void Sensor::registerSensor(Sensor *sensor)
{
    if (sensor->id_ >= 1 && sensor->id_ < 5)
    {
        sensors_[sensor->id_] = sensor;
    }
    else
    {
        Serial.println("Error: Sensor ID out of range!");
    }
}

// Handle interrupt for a specific sensor
// This method is called in the interrupt service routines (ISRs) to handle the echo signal
// It checks if the sensor ID is valid and updates the duration and distance values accordingly
// The duration is measured in microseconds and converted to distance in centimeters
// The distance is calculated using the formula: distance = duration / 58.0
void Sensor::handleInterrupt(int sensorId)
{
    if (sensors_[sensorId] != nullptr)
    {
        Sensor *sensor = sensors_[sensorId];
        if (digitalRead(sensor->getEchoPin()) == HIGH)
        {
            sensor->duration_ = micros();
        }
        else
        {
            sensor->distance_ = (micros() - sensor->duration_) / 58.0;
        }
    }
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

// ISR for sensor 4
void Sensor::echoISR4()
{
    handleInterrupt(4);
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

// Method to check if a wall is approaching
// This method reads the distance and updates the measurement array if the distance is valid (higher than 100cm and not 10cm)
// It calculates the trend as total change and counts how many drops (negative changes) occur in the measurements
// If there are 5 or more drops and the average step change is negative enough, it indicates a wall is approaching
// The method returns true if a wall is detected, otherwise false
// It also prints the sensor ID, measurement index, newest index, measurements, trend, drops, and distance to the Serial Monitor for debugging purposes
bool Sensor::isWallApproaching()
{
    readDistance();
    // Update the measurement array if distance is valid (higher than 100cm and not 10cm)
    if (distance_ > MAXIMUM_DISTANCE || distance_ <= MINIMUM_DISTANCE)
    {
        return false;
    }

    measurements_[measurementIndex_] = distance_;
    measurementIndex_ = (measurementIndex_ + 1) % MEASUREMENTS_SIZE;

    // Calculate the trend as total change and count how many drops
    int drops = 0;
    float totalTrend = 0;

    int newestIndex = (measurementIndex_ - 1 + MEASUREMENTS_SIZE) % MEASUREMENTS_SIZE; // Most recent value

    // Loop over all steps in the measurements array using a circular buffer approach
    for (int i = 0; i < MEASUREMENTS_SIZE - 1; i++)
    {
        int index1 = (measurementIndex_ + i) % MEASUREMENTS_SIZE;
        int index2 = (measurementIndex_ + i + 1) % MEASUREMENTS_SIZE;
        float stepChange = measurements_[index2] - measurements_[index1];
        if (stepChange < 0)
            drops++; // Count drops (negative changes)

        totalTrend += stepChange;
    }

    // Calculate average step change
    float avgStepChange = totalTrend / (MEASUREMENTS_SIZE - 1);

    // Smart detection criteria:
    // If we have 3 or more drops and the average step change is negative enough, it's a wall.
    if (drops >= MINIMUM_DROPS && avgStepChange < MIN_AVERAGE_STEP_CHANGE && distance_ < MAXIMUM_DISTANCE)
    {
        wallApproaching_ = true;
        Serial.print("Sensor ID: ");
        Serial.print(id_);
        Serial.print(" | Measurement Index: ");
        Serial.print(measurementIndex_);
        Serial.print(" | Newest Index: ");
        Serial.print(newestIndex);
        Serial.print(" | Measurements: ");
        for (size_t i = 0; i < MEASUREMENTS_SIZE; i++)
        {
            Serial.print(measurements_[i]);
            if (i < (MEASUREMENTS_SIZE)-1)
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
        Serial.print("Wall approaching detected (smart mode) by Sensor ID: ");
        Serial.println(id_);
    }
    else
    {
        wallApproaching_ = false;
    }

    return wallApproaching_;
}

// Method to read distance
// This method triggers the ultrasonic sensor and reads the echo time to calculate the distance
// If using interrupts, it calls the trigger method to start the measurement
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
        distance_ = duration_ / 58.0;
    }
}

// Getter for distance_
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
    Serial.print("  |   Distance: ");
    Serial.print(distance_);
    Serial.print(" | Measurement Index: ");
    Serial.print(measurementIndex_);
    Serial.print(" | Measurements: ");
    for (size_t i = 0; i < MEASUREMENTS_SIZE; i++)
    {
        Serial.print(measurements_[i]);
        if (i < (MEASUREMENTS_SIZE)-1)
        {
            Serial.print(", ");
        }
    }
    Serial.print("   |   Wall Approaching: ");
    Serial.println(wallApproaching_ ? "Yes" : "No");
}

// Add a public method to reset measurements and index
void Sensor::resetMeasurements()
{
    Serial.print("Resetting measurements for Sensor ID: ");
    Serial.println(id_);
    for (size_t i = 0; i < MEASUREMENTS_SIZE; i++)
    {
        measurements_[i] = 0;
    }
    measurementIndex_ = 0;
    distance_ = 0;
}
