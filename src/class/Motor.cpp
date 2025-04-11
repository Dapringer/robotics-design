#include "Motor.h"

#include <Arduino.h>

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
        Serial.println("Invalid speed value. Must be between 0 and 255.");
        return;
    }
    switch (direction)
    {
    case 1:
        digitalWrite(inPin1_, HIGH);
        digitalWrite(inPin2_, LOW);
        break;
    case 2:
        digitalWrite(inPin1_, LOW);
        digitalWrite(inPin2_, HIGH);
        break;
    default:
        return;
    }
    switch (id_)
    {
    case 1:
        analogWrite(enPin_, speed);
        break;
    case 2:
        analogWrite(enPin_, speed + 30);

        break;
    default:
        return;
    }
}

void Motor::emergencyStop()
{
    digitalWrite(inPin1_, LOW);
    digitalWrite(inPin2_, LOW);
    analogWrite(enPin_, 0);
}