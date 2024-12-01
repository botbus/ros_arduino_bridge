#ifndef  DIFF_CONTROLLER_H
#define  DIFF_CONTROLLER_H

// extern const int MAX_PWM;

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

extern SetPointInfo leftPID;
extern SetPointInfo rightPID;

/* PID Parameters */
extern int Kp;
extern int Kd;
extern int Ki;
extern int Ko;

extern unsigned char moving; // is the base in motion?
void resetPID();
void doPID(SetPointInfo* p);
void updatePID();
void initMotorPins();
#endif