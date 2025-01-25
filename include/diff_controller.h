// #ifndef DIFF_CONTROLLER_H
// #define DIFF_CONTROLLER_H

// typedef struct
// {
//   float targetVal; // target speed in ticks per frame
//   long currentEnc; // encoder count
//   long PrevEnc;    // last encoder count
//   int PrevInput;   // last input
//   int Iterm;       // integrated term
//   int Pterm;
//   int Dterm;
//   long output; // last motor setting
// } configPID;

// extern configPID leftPID;
// extern configPID rightPID;

// /* PID Parameters */
// extern int Kp;
// extern int Kd;
// extern int Ki;
// extern int Ko;

// extern unsigned char moving; // is the base in motion?
// void resetPID();
// void doPID(configPID *p);
// void updatePID();
// void initMotorPins();
// #endif