/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
extern const int RIGHT_MOTOR_BACKWARD;
extern const int LEFT_MOTOR_BACKWARD;
extern const int RIGHT_MOTOR_FORWARD;
extern const int LEFT_MOTOR_FORWARD;
extern const int RIGHT_MOTOR_ENABLE;
extern const int LEFT_MOTOR_ENABLE;

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

#endif