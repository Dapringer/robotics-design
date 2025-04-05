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

public:
    Sensor(int id, int triggerPin, int echoPin);

    void read();
    unsigned int getDistance() const;

    void printStatus() const;
};

#endif // SENSOR_H