/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include "Arduino.h"
class MotorDriver
{
private:
   const int rightMotorBackward_;
   const int leftMotorBackward_;
   const int rightMotorForward_;
   const int leftMotorForward_;
   const int rightMotorEnable_;
   const int leftMotorEnable_;

public:
   MotorDriver();
   void setMotorSpeed(int i, double spd, bool reverse);
   void setMotorSpeeds(double leftSpeed, double rightSpeed, bool reverseLeft, bool reverseRight);
};
#endif