/* *************************************************************
   Encoder definitions

   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   ************************************************************ */

// #ifdef USE_BASE

// #ifdef ROBOGAIA
//   /* The Robogaia Mega Encoder shield */
//   #include "MegaEncoderCounter.h"

//   /* Create the encoder shield object */
//   MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode

//   /* Wrap the encoder reading function */
//   long readEncoder(int i) {
//     if (i == LEFT) return encoders.YAxisGetCount();
//     else return encoders.XAxisGetCount();
//   }

//   /* Wrap the encoder reset function */
//   void resetEncoder(int i) {
//     if (i == LEFT) return encoders.YAxisReset();
//     else return encoders.XAxisReset();
//   }
// #elif defined(ARDUINO_ENC_COUNTER)
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
static const int ENC_STATES[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0}; // encoder lookup table

// /* Interrupt routine for LEFT encoder, taking care of actual counting */
// ISR(PCINT2_vect)
// {
//   static uint8_t enc_last = 0;

//   enc_last <<= 2;                     // shift previous state two places
//   enc_last |= (PIND & (3 << 2)) >> 2; // read the current state into lowest 2 bits

//   left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
// }
// /* Interrupt routine for RIGHT encoder, taking care of actual counting */
// ISR(PCINT1_vect)
// {
//   static uint8_t enc_last = 0;

//   enc_last <<= 2;                     // shift previous state two places
//   enc_last |= (PINC & (3 << 4)) >> 4; // read the current state into lowest 2 bits

//   right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
// }
#include "Arduino.h"
#include "include/commands.h"
#include "include/diff_controller.h"
#include "include/encoder_driver.h"
bool run_left_ISR{false};
bool run_right_ISR{false};

const int LEFT_ENC_PIN_A{4}; // pin 2
const int LEFT_ENC_PIN_B{5}; // pin 3

// below can be changed, but should be PORTC pins
const int RIGHT_ENC_PIN_A{3}; // pin A4
const int RIGHT_ENC_PIN_B{2}; // pin A5
void initMotorPins()
{
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), PIN_ISR_LEFT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), PIN_ISR_RIGHT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), PIN_ISR_LEFT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), PIN_ISR_RIGHT, CHANGE);
}
void PIN_ISR_LEFT()
{
  run_left_ISR = true;
}
void PIN_ISR_RIGHT()
{
  run_right_ISR = true;
}

void RUN_PIN_ISR_LEFT()
{
  noInterrupts(); // Disable interrupts
  static uint8_t enc_last = 0;

  enc_last <<= 2;

  uint8_t current_state = (digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B);
  enc_last |= current_state;
  left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  run_left_ISR = false;
  Serial.print("LEFT A: ");
  Serial.print(digitalRead(LEFT_ENC_PIN_A));
  Serial.print("LEFT B: ");
  Serial.println(digitalRead(LEFT_ENC_PIN_B));

  interrupts();
}
void RUN_PIN_ISR_RIGHT()
{
  noInterrupts();
  static uint8_t r_enc_last = 0;

  r_enc_last <<= 2;

  uint8_t current_state = (digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B);
  r_enc_last |= current_state;
  right_enc_pos += ENC_STATES[(r_enc_last & 0x0f)];
  run_right_ISR = false;
  // Serial.println("RIGHT RUNNING");
   Serial.print("RIGT A: ");
  Serial.print(digitalRead(RIGHT_ENC_PIN_A));
  Serial.print("RIGT B: ");
  Serial.println(digitalRead(RIGHT_ENC_PIN_B));
  interrupts();
}

/* Wrap the encoder reading function */
long readEncoder(int i)
{
  if (i == LEFT)
    return left_enc_pos;
  else
    return right_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i)
{
  if (i == LEFT)
  {
    left_enc_pos = 0L;
    return;
  }
  else
  {
    right_enc_pos = 0L;
    return;
  }
}
// #else
//   #error A encoder driver must be selected!
// #endif

/* Wrap the encoder reset function */
void resetEncoders()
{
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

// #endif
