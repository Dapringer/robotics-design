#include "Motor.h"

#include <Arduino.h>

#define MOTOR_2_OFFSET 30         // Offset for motor 2 speed
#define ENCODER_PULSES_PER_REV 32 // Number of pulses per revolution of the motor

Motor::Motor(int id, int inPin1, int inPin2, int enPin, int encoderAPin, int encoderBPin)
    : id_(id), inPin1_(inPin1), inPin2_(inPin2), enPin_(enPin), encoderAPin_(encoderAPin), encoderBPin_(encoderBPin), encoderCount_(0), lastSpeedCalcTime_(0), speed_(0.0f)
{
    pinMode(inPin1_, OUTPUT);
    pinMode(inPin2_, OUTPUT);
    pinMode(enPin_, OUTPUT);
    pinMode(encoderAPin_, INPUT);
    pinMode(encoderBPin_, INPUT);
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

float Motor::getSpeed()
{
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastSpeedCalcTime_;

    if (encoderCount_ >= ENCODER_PULSES_PER_REV)
    {
        // Calculate speed in rotations per second (RPS)
        speed_ = (float)encoderCount_ / ENCODER_PULSES_PER_REV / (elapsedTime / 1000.0f) * 60.0f;
        encoderCount_ = 0;                // Reset the encoder count after calculating speed
        lastSpeedCalcTime_ = currentTime; // Update the last speed calculation time
        speed_ = speed_ / 131;
        Serial.print("Motor ID: ");
        Serial.print(id_);
        Serial.print(" |  Speed: ");
        Serial.println(speed_);
    }

    return speed_;
}