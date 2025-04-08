// filepath: c:\Users\frede\Documents\GitHub\robotics-design\UltraSonicAndLED\class\Motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor
{
private:
    int id_;
    int inPin1_;
    int inPin2_;
    int enPin_;

    enum Direction
    {
        FORWARD,
        BACKWARD
    };

    enum Speed
    {
        SLOW = 85,    // 33% of 255
        MEDIUM = 170, // 66% of 255
        FAST = 255    // 100% of 255
    };

public:
    Motor(int id, int inPin1, int inPin2, int enPin);

    void drive(Speed speed, Direction direction);
    void emergencyStop();
};

#endif // MOTOR_H