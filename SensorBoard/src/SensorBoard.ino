/**
 * SensorBoard.ino
 *
 * Main file for the Sensor Board of the robotics-design project.
 * This board is responsible for ultrasonic sensor readings, obstacle detection,
 * and sending control commands to the Motor Board.
 *
 * The board uses four ultrasonic sensors to detect obstacles in all directions
 * and sends commands to the Motor Board to stop, turn, or navigate around obstacles.
 *
 * Communication:
 * - Serial: Debug information (115200 baud)
 * - Serial2: Commands to Motor Board (9600 baud)
 *
 * Command Values Sent to Motor Board:
 * - 0: Emergency stop (obstacle detected)
 * - 1: Move forward (no obstacles detected)
 * - 2: Back up (obstacle ahead)
 * - 3: Turn left (obstacle on right)
 * - 4: Turn right (obstacle on left)
 *
 * Note: Advanced wall detection and avoidance algorithms are currently commented out
 * for testing purposes. Only emergency breaking functionality is enabled.
 *
 * @author: Movement and Localization Outdoor Robot Team
 */
#include <stdlib.h>
#include <Arduino.h>
#include <Wire.h>

#include "class/Sensor.h"

// Pin definitions for ultrasonic sensors
#define PIN_TRIGGER_F 22 // Front sensor trigger pin
#define PIN_ECHO_F 2     // Front sensor echo pin
#define PIN_TRIGGER_R 24 // Right sensor trigger pin
#define PIN_ECHO_R 3     // Right sensor echo pin
#define PIN_TRIGGER_B 26 // Back sensor trigger pin
#define PIN_ECHO_B 18    // Back sensor echo pin
#define PIN_TRIGGER_L 28 // Left sensor trigger pin
#define PIN_ECHO_L 19    // Left sensor echo pin

// Constants for emergency stop distances (in cm)
#define EMERGENCY_STOP_DISTANCE_SIDES 15 // Side sensor threshold
#define EMERGENCY_STOP_DISTANCE_FRONT 30 // Front sensor threshold
#define EMERGENCY_STOP_DISTANCE_BACK 20  // Back sensor threshold

// Timing constants (in milliseconds)
#define TURNING_TIME 500 // Duration for turning maneuvers
#define BACKUP_TIME 3000 // Duration for backing up

// Initialize the sensors
Sensor sensorFront(1, PIN_TRIGGER_F, PIN_ECHO_F, true);
Sensor sensorRight(2, PIN_TRIGGER_R, PIN_ECHO_R, true);
Sensor sensorBack(3, PIN_TRIGGER_B, PIN_ECHO_B, true);
Sensor sensorLeft(4, PIN_TRIGGER_L, PIN_ECHO_L, true);

// variables to store wall detection status
bool wallFront = false;
bool wallRight = false;
bool wallLeft = false;
bool wallBack = false;

// variables to store distance readings
float distFront = 0;
float distRight = 0;
float distLeft = 0;
float distBack = 0;

// bool stopOverwrite = false;
bool humanInFront = false;

void setup()
{
  Serial.begin(9600);  // Initialize Serial communication for debugging
  Serial2.begin(9600); // Initialize Serial2 for communication with Motor Board
}

void loop()
{
  // Check if data is available from Motor Board
  if (Serial2.available() > 0)
  {
    String command = Serial2.readStringUntil('\n'); // Read the command
    command.trim();                                 // Trim whitespace
    if (command == "humanInFront = false")          // Check for specific command
    {
      humanInFront = false; // Set humanInFront to false
    }
    else if (command == "humanInFront = true")
    {
      humanInFront = true; // Set humanInFront to true
    }
  }

  readSensors();

  if (distFront < 20)
  {
    Serial2.println(0);
    Serial.print("Emergency stop activated!  |  ");
    Serial.println("distFront: " + String(distFront));
  }
  else if (distRight <= EMERGENCY_STOP_DISTANCE_SIDES && distRight != 0)
  {
    Serial2.println(0);
    Serial.print("Emergency stop activated!  |  ");
    Serial.println("distRight: " + String(distRight));
  }
  else if (distLeft <= EMERGENCY_STOP_DISTANCE_SIDES && distLeft != 0)
  {
    Serial2.println(0);
    Serial.print("Emergency stop activated!  |  ");
    Serial.println("distLeft: " + String(distLeft));
  }
  else if (distBack <= EMERGENCY_STOP_DISTANCE_BACK && distBack != 0)
  {
    Serial2.println(0);
    Serial.print("Emergency stop activated!  |  ");
    Serial.println("distBack: " + String(distBack));
  }

  /**
   * WALL DETECTION AND AVOIDANCE CODE (CURRENTLY DISABLED)
   *
   * The following code block contains advanced wall detection and avoidance algorithms
   * that are currently commented out for testing purposes. In the current phase of
   * development, we are focusing only on emergency breaking functionality.
   *
   * Features in this section include:
   * - Wall approach detection using sensor trend analysis
   * - Obstacle avoidance by turning in the appropriate direction
   * - Backup and turn maneuvers when a wall is detected in front
   * - Prioritized obstacle avoidance based on proximity
   */
  // if (wallFront || (distFront < EMERGENCY_STOP_DISTANCE_FRONT && distFront > 0))
  // {
  //   // Wall detected in front → back up
  //   Serial.println("Wall detected in front! Backing up...");
  //   sensorFront.printStatus();
  //   backUpFromWall();
  // }
  // else if (wallRight && wallLeft)
  // {
  //   // Both sensors detect a wall – avoid the closer one
  //   if (distRight < distLeft)
  //   {
  //     // Right is closer → steer left
  //     Serial.println("Steering left to avoid wall on the right... Right closer than left");
  //     avoidWall(3);
  //   }
  //   else
  //   {
  //     // Left is closer → steer right
  //     Serial.println("Steering right to avoid wall on the left... Left closer than right");
  //     avoidWall(4);
  //   }
  // }
  // else if (wallRight)
  // {
  //   // Only right detects wall → steer left
  //   Serial.println("Steering left to avoid wall on the right...");
  //   avoidWall(3);
  // }
  // else if (wallLeft)
  // {
  //   // Only left detects wall → steer right
  //   Serial.println("Steering right to avoid wall on the left...");
  //   avoidWall(4);
  // }
  // else if ((distFront == 0 || distFront > EMERGENCY_STOP_DISTANCE_FRONT) &&
  //          (distRight == 0 || distRight > EMERGENCY_STOP_DISTANCE_SIDES) &&
  //          (distLeft == 0 || distLeft > EMERGENCY_STOP_DISTANCE_SIDES))
  // {
  //   // No wall approaching → wait for I2C Command
  // }
  // else
  // {
  //   // Emergency stop if any condition is unsafe
  //   Serial2.println(0);
  //   Serial.print("Emergency stop activated!  |  ");
  //   Serial.println("distFront: " + String(distFront));
  // }

  // Serial.println("------------------");
  delay(100);
}

// Reading sensors and updating the wall detection status
void readSensors()
{
  // wallFront = sensorFront.isWallApproaching();
  // wallRight = sensorRight.isWallApproaching();
  // wallLeft = sensorLeft.isWallApproaching();
  // wallBack = sensorBack.isWallApproaching();
  sensorFront.readDistance();
  sensorRight.readDistance();
  sensorBack.readDistance();
  sensorLeft.readDistance();

  distFront = sensorFront.getDistance();
  distRight = sensorRight.getDistance();
  distLeft = sensorLeft.getDistance();
  distBack = sensorBack.getDistance();
}

// Avoid wall by turning left or right for a short time
void avoidWall(int direction)
{
  unsigned long turnStartTime = millis();

  for (; millis() - turnStartTime < TURNING_TIME;)
  {
    Serial2.println(direction);
  }

  Serial2.println(0);
}

// Back up from wall if detected in front. Sequence is as follows:
// 1. If Human in Front, do nothing
// 2. Stop motors for 1 second
// 3. If no wall in Back, back up for 3 seconds
// 4. If wall detected, emergency Stop
// 5. Otherwise turn randomly for 2 seconds
// 6. Stop motors
void backUpFromWall()
{
  if (humanInFront)
  {
    Serial.println("Human detected in front, not backing up!");
    return;
  }

  float distFront = sensorFront.getDistance();

  Serial2.println(0);
  delay(500);

  bool wallBack = sensorBack.isWallApproaching();
  if (!wallBack)
  {
    Serial.println("Backing up...");
    unsigned long startTime = millis();

    for (; millis() - startTime < BACKUP_TIME;)
    {
      Serial2.println(2); // Backward command with 3 seconds execution time

      wallBack = sensorBack.isWallApproaching();
      if (wallBack)
      {
        Serial.println("Obstacle detected while backing up!");
        Serial2.println(0);
        break;
      }

      delay(50);
    }

    if (!wallBack)
    {
      Serial.println("Turning randomly...");
      unsigned long turnStartTime = millis();

      // Turn left or right randomly for 2 seconds
      int turnDirection = random(0, 1) == 0 ? 4 : 3; // 4 = right, 3 = left
      for (; millis() - turnStartTime < TURNING_TIME;)
      {
        Serial2.println(turnDirection); // Turn command with 2 seconds execution time
        delay(50);                      // Small delay to avoid busy-waiting
      }

      Serial2.println(0);
    }
  }
  else
  {
    Serial.println("Cannot back up, obstacle detected!");
  }

  // Reset the measurement arrays for all sensors after backing up
  sensorBack.resetMeasurements();
  sensorFront.resetMeasurements();
  sensorLeft.resetMeasurements();
  sensorRight.resetMeasurements();
}
