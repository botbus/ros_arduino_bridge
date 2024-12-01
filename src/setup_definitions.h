#ifndef SETUP_DEFINITIONS_H
#define SETUP_DEFINITIONS_H


constexpr long BAUDRATE = 115200;
const int MAX_PWM = 255;
/* Run the PID loop at 30 times per second */
constexpr int PID_RATE = 30; // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
extern unsigned long nextPID;

/* Stop the robot if it hasn't received a movement command
 in this number of milliseconds */
constexpr long AUTO_STOP_INTERVAL = 2000;
extern long lastMotorCommand;




#endif