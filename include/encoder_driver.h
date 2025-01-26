/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

// below can be changed, but should be PORTD pins;
// otherwise additional changes in the code are required
extern const int LEFT_ENC_PIN_A; // pin 2
extern const int LEFT_ENC_PIN_B; // pin 3

// below can be changed, but should be PORTC pins
extern const int RIGHT_ENC_PIN_A; // pin A4
extern const int RIGHT_ENC_PIN_B; // pin A5

long readEncoder(int i);
void resetEncoders();

#endif