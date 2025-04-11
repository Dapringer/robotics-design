#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
class Sensor
{
private:
    int id_;
    bool wallApproaching_;
    int triggerPin_;
    int echoPin_;
    unsigned long duration_;
    unsigned int distance_;
    float measurements_[5]; // Array to store the last 10 measurements
    int measurementIndex_;  // Index for the circular buffer
    bool useInterrupt_;

    static Sensor *sensors_[4]; // Fixed-size array for 4 sensors

public:
    Sensor(int id, int triggerPin, int echoPin, bool useInterrupt_);

    bool isWallApproaching();
    void readDistance();
    unsigned int getDistance() const;
    void printStatus();

    void trigger();
    int getEchoPin() const;

    static void registerSensor(Sensor *sensor); // Register a sensor
    static void handleInterrupt(int sensorId);              // Handle interrupt for all sensors
    
    // Interrupt Service Routines (ISRs) for each sensor
    static void echoISR0();
    static void echoISR1();
    static void echoISR2();
    static void echoISR3();
};

#endif // SENSOR_H