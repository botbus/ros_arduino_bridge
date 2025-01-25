/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

extern const int LEFT_ENC_PIN_A;
extern const int LEFT_ENC_PIN_B;
extern const int RIGHT_ENC_PIN_A;
extern const int RIGHT_ENC_PIN_B;

long readEncoder(int i);
void resetEncoders();

#endif