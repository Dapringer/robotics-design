#include "Motor.h"

#include <Arduino.h>

#define ENCODER_PULSES_PER_REV 32 // Number of pulses per revolution of the motor

Motor::Motor(int id, int pwmPin, int dirPin, int encoderAPin, int encoderBPin)
    : id_(id), pwmPin_(pwmPin), dirPin_(dirPin), encoderAPin_(encoderAPin), encoderBPin_(encoderBPin), encoderCount_(0), lastSpeedCalcTime_(0), speed_(0.0f)
{
    pinMode(pwmPin_, OUTPUT);
    pinMode(dirPin_, OUTPUT);
    pinMode(encoderAPin_, INPUT);
    pinMode(encoderBPin_, INPUT);
}

void Motor::drive(int speed, int direction)
{
    if (speed < 0 || speed > 255)
    {
        Serial.println("Invalid speed value. Must be between 0 and 255.");
        return;
    }
    if (direction < 0 || direction > 2)
    {
        Serial.println("Invalid direction value. Must be 0 (stop), 1 (forward), or 2 (backward).");
        return;
    }

    switch (direction)
    {
    case 0: // Stop
        digitalWrite(dirPin_, LOW);
        analogWrite(pwmPin_, 0);
        break;

    case 1: // Forward
        analogWrite(pwmPin_, speed);
        analogWrite(dirPin_, 0);
        break;

    case 2: // Backward
        analogWrite(dirPin_, speed);
        analogWrite(pwmPin_, 0);
        break;

    default:
        Serial.println("Invalid direction value. Must be 0 (stop), 1 (forward), or 2 (backward).");
        break;
    }
}

// Emergency stop function to stop the motor immediately
// This function sets the direction pin to LOW and the PWM pin to 0
void Motor::emergencyStop()
{
    digitalWrite(dirPin_, LOW);
    analogWrite(pwmPin_, 0);
}

// Function to calculate the speed of the motor based on encoder counts
// This function is called in the interrupt service routine (ISR) for the encoder
// It calculates the speed in rotations per minute (RPM) based on the elapsed time since the last calculation
// and the number of encoder counts since the last calculation
float Motor::getSpeed()
{
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastSpeedCalcTime_;

    if (encoderCount_ >= ENCODER_PULSES_PER_REV)
    {
        speed_ = (float)encoderCount_ / ENCODER_PULSES_PER_REV / (elapsedTime / 1000.0f) * 60.0f;
        encoderCount_ = 0;                // Reset the encoder count after calculating speed
        lastSpeedCalcTime_ = currentTime; // Update the last speed calculation time
        speed_ = speed_ / 131;
        // Serial.print("Motor ID: ");
        // Serial.print(id_);
        // Serial.print(" |  Speed: ");
        // Serial.println(speed_);
    }

    return speed_;
}