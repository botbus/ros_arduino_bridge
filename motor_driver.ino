/***************************************************************
   Motor driver definitions

   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   *************************************************************/

#include "include/commands.h"
#include "include/motor_driver.h"
#include "include/diff_controller.h"

constexpr int RIGHT_MOTOR_BACKWARD{7};
constexpr int LEFT_MOTOR_BACKWARD{9};
constexpr int RIGHT_MOTOR_FORWARD{8};
constexpr int LEFT_MOTOR_FORWARD{10};
constexpr int RIGHT_MOTOR_ENABLE{6};
constexpr int LEFT_MOTOR_ENABLE{11};

void initMotorController()
{
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
}

void setMotorSpeed(int i, int spd)
{
  unsigned char reverse = (spd < 0);
  spd = constrain(abs(spd), 0, 255);

  int forwardPin = (i == LEFT) ? LEFT_MOTOR_FORWARD : RIGHT_MOTOR_FORWARD;
  int backwardPin = (i == LEFT) ? LEFT_MOTOR_BACKWARD : RIGHT_MOTOR_BACKWARD;

  analogWrite(forwardPin, reverse ? 0 : spd);
  analogWrite(backwardPin, reverse ? spd : 0);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
