
#include "include/motor_driver.h"

MotorDriver::MotorDriver()
    : rightMotorBackward_(7),
      leftMotorBackward_(9),
      rightMotorForward_(8),
      leftMotorForward_(10),
      rightMotorEnable_(6),
      leftMotorEnable_(11)
{
  digitalWrite(rightMotorEnable_, HIGH);
  digitalWrite(leftMotorEnable_, HIGH);
}

void MotorDriver::setMotorSpeed(int i, double spd, bool reverse)
{
  // unsigned char reverse = (spd < 0);
  spd = constrain(abs(spd), 0, 255);

  int forwardPin = (i == LEFT) ? leftMotorForward_ : rightMotorForward_;
  int backwardPin = (i == LEFT) ? leftMotorBackward_ : rightMotorBackward_;
  spd = static_cast<int>(spd);
  analogWrite(forwardPin, reverse ? 0 : spd);
  analogWrite(backwardPin, reverse ? spd : 0);
}

void MotorDriver::setMotorSpeeds(double leftSpeed, double rightSpeed, bool reverseLeft, bool reverseRight)
{
  // Serial.print("Speed:   ");
  // Serial.print(leftSpeed, 0);
  // Serial.print(" ");
  // Serial.println(rightSpeed, 0);

  setMotorSpeed(LEFT, leftSpeed, reverseLeft);
  setMotorSpeed(RIGHT, rightSpeed, reverseRight);
}
