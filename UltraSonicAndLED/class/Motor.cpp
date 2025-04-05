#include "Motor.h"

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

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
    Motor(int id, int inPin1, int inPin2, int enPin) : id_(id), inPin1_(inPin1), inPin2_(inPin2), enPin_(enPin)
    {
        pinMode(inPin1_, OUTPUT);
        pinMode(inPin2_, OUTPUT);
        pinMode(enPin_, OUTPUT);
    }

    void drive(Speed speed, Direction direction)
    {
        switch (direction)
        {
        case FORWARD:
            digitalWrite(inPin1_, HIGH);
            digitalWrite(inPin2_, LOW);
            break;
        case BACKWARD:
            digitalWrite(inPin1_, LOW);
            digitalWrite(inPin2_, HIGH);
            break;
        default:
            return;
        }
        analogWrite(enPin_, speed);
    }

    void emergencyStop()
    {
        digitalWrite(inPin1_, LOW);
        digitalWrite(inPin2_, LOW);
        analogWrite(enPin_, 0);
    }
}