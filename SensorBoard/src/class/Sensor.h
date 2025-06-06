/**
 * Sensor.h
 *
 * This file defines the Sensor class, which encapsulates all functionality
 * related to ultrasonic distance sensors for obstacle detection.
 *
 * Features:
 * - Distance measurement
 * - Wall approach detection algorithm
 * - Interrupt-based timing for precise measurements
 * - Sensor status reporting
 *
 * @author: Movement and Localization Outdoor Robot Team
 */
#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

/**
 * Sensor class for managing ultrasonic distance sensors
 */
class Sensor
{
private:
    int id_;                 // Unique identifier for the sensor
    bool wallApproaching_;   // Flag indicating if a wall is approaching
    int triggerPin_;         // Pin for triggering the ultrasonic sensor
    int echoPin_;            // Pin for receiving the echo signal
    unsigned long duration_; // Duration of the echo signal
    unsigned int distance_;  // Calculated distance in centimeters
    float measurements_[10]; // Array to store the last 10 measurements
    int measurementIndex_;   // Index for the circular buffer
    bool useInterrupt_;      // Flag to use interrupt-based measurement

    static Sensor *sensors_[5]; // Fixed-size array for 4 sensors (index 0 unused)

public:
    /**
     * Constructor for Sensor class
     *
     * @param id Unique identifier for the sensor (1-4)
     * @param triggerPin Pin for triggering the ultrasonic sensor
     * @param echoPin Pin for receiving the echo signal
     * @param useInterrupt Flag to use interrupt-based measurement
     */
    Sensor(int id, int triggerPin, int echoPin, bool useInterrupt_);

    /**
     * Check if a wall is approaching based on measurement trends
     *
     * @return true if a wall is approaching, false otherwise
     */
    bool isWallApproaching();

    /**
     * Trigger the sensor and read the distance
     */
    void readDistance();

    /**
     * Get the current distance measurement
     *
     * @return Distance in centimeters
     */
    unsigned int getDistance() const;

    /**
     * Print sensor status to Serial for debugging
     */
    void printStatus();

    /**
     * Trigger the ultrasonic sensor
     */
    void trigger();

    /**
     * Get the echo pin number
     *
     * @return Echo pin number
     */
    int getEchoPin() const;

    /**
     * Reset the measurement array and index
     */
    void resetMeasurements();

    /**
     * Register a sensor in the static array
     *
     * @param sensor Pointer to the sensor to register
     */
    static void registerSensor(Sensor *sensor);

    /**
     * Handle interrupt for a specific sensor
     *
     * @param sensorId ID of the sensor (1-4)
     */
    static void handleInterrupt(int sensorId);

    // Interrupt Service Routines (ISRs) for each sensor
    static void echoISR1();
    static void echoISR2();
    static void echoISR3();
    static void echoISR4();
};

#endif // SENSOR_H