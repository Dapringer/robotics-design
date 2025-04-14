#include "Motor.h"

#include <Arduino.h>

#define MOTOR_2_OFFSET 30 // Offset for motor 2 speed

Motor::Motor(int id, int inPin1, int inPin2, int enPin)
    : id_(id), inPin1_(inPin1), inPin2_(inPin2), enPin_(enPin)
{
    pinMode(inPin1_, OUTPUT);
    pinMode(inPin2_, OUTPUT);
    pinMode(enPin_, OUTPUT);
}

void Motor::drive(int speed, int direction)
{
    if (speed < 0 || speed > 255)
    {
        Serial.println("Invalid speed value. Must be between 0 and 255 when driving front or back.");
        return;
    }
    if (direction < 0 || direction > 4)
    {
        Serial.println("Invalid direction value. Must be between 0 and 4.");
        return;
    }
    switch (direction)
    {
    case 0:
        // Stop
        digitalWrite(inPin1_, LOW);
        digitalWrite(inPin2_, LOW);
        break;
    case 1:
        // Forward
        digitalWrite(inPin1_, HIGH);
        digitalWrite(inPin2_, LOW);
        break;
    case 2:
        // Backward
        digitalWrite(inPin1_, LOW);
        digitalWrite(inPin2_, HIGH);
        break;
    default:
        return;
    }
    if (direction == 0)
    {
        analogWrite(enPin_, 0);
        return;
    }
    else
    {
        switch (id_)
        {
        case 1:
            analogWrite(enPin_, speed);
            break;
        case 2:
            analogWrite(enPin_, speed + MOTOR_2_OFFSET);

            break;
        default:
            return;
        }
    }
}

void Motor::emergencyStop()
{
    digitalWrite(inPin1_, LOW);
    digitalWrite(inPin2_, LOW);
    analogWrite(enPin_, 0);
}