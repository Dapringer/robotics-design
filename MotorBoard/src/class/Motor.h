/**
 * Motor.h
 *
 * This file defines the Motor class, which encapsulates all functionality
 * related to controlling a DC motor with encoder feedback.
 *
 * Features:
 * - Directional control (forward, backward, stop)
 * - Speed control via PWM
 * - Encoder-based speed measurement
 * - Emergency stop functionality
 *
 * @author: Movement and Localization Outdoor Robot Team
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

/**
 * Motor class for controlling DC motors with encoder feedback
 */
class Motor
{
private:
    int id_;                          // Unique identifier for the motor
    int pwmPin_;                      // PWM pin for speed control
    int dirPin_;                      // Direction pin
    int encoderAPin_;                 // Encoder phase A pin
    int encoderBPin_;                 // Encoder phase B pin
    unsigned long lastSpeedCalcTime_; // Last time speed was calculated
    float speed_;                     // Speed in rotations per second (RPM)

public:
    /**
     * Constructor for Motor class
     *
     * @param id Unique identifier for the motor
     * @param pwmPin PWM pin for speed control
     * @param dirPin Direction pin
     * @param encoderAPin Encoder phase A pin
     * @param encoderBPin Encoder phase B pin
     */
    Motor(int id, int pwmPin, int dirPin, int encoderAPin, int encoderBPin);

    /**
     * Drive the motor with specified speed and direction
     *
     * @param speed Speed value (0-255)
     * @param direction Direction (0=stop, 1=forward, 2=backward)
     */
    void drive(int speed, int direction);

    /**
     * Emergency stop the motor immediately
     */
    void emergencyStop();

    /**
     * Calculate and return the current motor speed
     *
     * @return Speed in RPM
     */
    float getSpeed();

    // Public variables for encoder management
    int encoderCount_; // Counter for encoder pulses
    int phaseA_;       // Current state of encoder phase A
    int phaseB_;       // Current state of encoder phase B
};

#endif // MOTOR_H