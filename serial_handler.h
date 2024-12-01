#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

// A pair of varibles to help parse serial commands (thanks Fergs)
extern int arg;
extern int indx;
// Variable to hold an input character
extern char chr;
extern bool run_left_ISR;
extern bool run_right_ISR;
// Variable to hold the current single-character command
extern char cmd;

// Character arrays to hold the first and second arguments
extern char argv1[16];
extern char argv2[16];

// The arguments converted to integers
extern long arg1;
extern long arg2;

void resetCommand();
int runCommand();
void handleSerial(char chr);

#endif