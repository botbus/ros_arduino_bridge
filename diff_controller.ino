// /* Functions and type-defs for PID control.

//    Taken mostly from Mike Ferguson's ArbotiX code which lives at:

//    http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
// */
// #include "include/commands.h"
// #include "include/encoder_driver.h"
// #include "include/motor_driver.h"
// #include "include/diff_controller.h"
// #include "include/setup_definitions.h"
// /* PID Parameters */
// int Kp = 20;
// int Kd = 12;
// int Ki = 5;
// int Ko = 50;
// configPID leftPID;
// configPID rightPID;
// unsigned char moving = 0; // is the base in motion?

// /*a*
//  * Initialize PID variables to zero to prevent startup spikes
//  * when turning PID on to start moving
//  *
//  * In particular, assign both currentEnc and PrevEnc the current encoder value
//  * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
//  *
//  * Note that the assumption here is that PID is only turned on
//  * when going from stop to moving, that's why we can init everything on zero.
//  *
//  * @note: This function is called when going from stop to moving
//  */
// void resetPID()
// {
//   leftPID.targetVal = 0.0;
//   leftPID.currentEnc = readEncoder(LEFT);
//   leftPID.PrevEnc = leftPID.currentEnc;
//   leftPID.output = 0;
//   leftPID.PrevInput = 0;
//   leftPID.Iterm = 0;
//   leftPID.Pterm = 0;
//   leftPID.Dterm = 0;
//   rightPID.targetVal = 0.0;
//   rightPID.currentEnc = readEncoder(RIGHT);
//   rightPID.PrevEnc = rightPID.currentEnc;
//   rightPID.output = 0;
//   rightPID.PrevInput = 0;
//   rightPID.Iterm = 0;
//   rightPID.Pterm = 0;
//   rightPID.Dterm = 0;
// }

// /* PID routine to compute the next motor commands */
// void doPID(configPID *p)
// {
//   int input = p->currentEnc - p->PrevEnc;
//   int Perror = p->targetVal - input;
//   int output = (Kp * Perror - Kd * (input - p->PrevInput) + p->Iterm) / Ko;
//   output = constrain(output, -MAX_PWM, MAX_PWM);

//   p->PrevEnc = p->currentEnc;
//   p->Iterm += Ki * Perror;
//   p->output = output;
//   p->PrevInput = input;
// }

// /* Read the encoder values and call the PID routine */
// void updatePID()
// {
//   if (!moving)
//   {
//     if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0)
//       resetPID();
//     return;
//   }

//   /* Read the encoders */
//   leftPID.currentEnc = readEncoder(LEFT);
//   rightPID.currentEnc = readEncoder(RIGHT);

//   /* Compute PID update for each motor */
//   doPID(&rightPID);
//   doPID(&leftPID);

//   /* Set the motor speeds accordingly */
//   setMotorSpeeds(leftPID.output, rightPID.output);
// }
