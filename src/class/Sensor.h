#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

class Sensor {
private:
    int id_;
    bool wallApproaching_;
    int triggerPin_;
    int echoPin_;
    unsigned long duration_;
    unsigned int distance_;
    unsigned int previousDistance_;
    float measurements_[5]; // Array to store the last 10 measurements
    int measurementIndex_;   // Index for the circular buffer

public:
    Sensor(int id, int triggerPin, int echoPin);

    bool isWallApproaching();
    void readDistance();
    unsigned int getDistance() const;
    void printStatus();
};

#endif // SENSOR_H