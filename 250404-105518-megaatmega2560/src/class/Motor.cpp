#include "Motor.h"

#include <Arduino.h>

Motor::Motor(int id, int inPin1, int inPin2, int enPin)
    : id_(id), inPin1_(inPin1), inPin2_(inPin2), enPin_(enPin)
{
    pinMode(inPin1_, OUTPUT);
    pinMode(inPin2_, OUTPUT);
    pinMode(enPin_, OUTPUT);
}

void Motor::drive(Speed speed, Direction direction)
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

void Motor::emergencyStop()
{
    digitalWrite(inPin1_, LOW);
    digitalWrite(inPin2_, LOW);
    analogWrite(enPin_, 0);
}