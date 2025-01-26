#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H
#include <string>
#include <cstdlib> // for strtok_r
class SerialHandler
{

private:
    char cmd;
    int parseArg = 0;
    int indx = 0;
    char chr;
    char argv1[16];
    char argv2[16];
    long arg1;
    long arg2;

    int MAN{0};
    int AUTO{1};
    double MAX_PWM{255.0};
    double MIN_PWM{0.0};
    long previousTime;
    int PID_RATE{20};
    bool debug = false;
    bool next = false;
    double slowPID[3] ={0.5, 0.6, 0.01};
    double midPID[3] = {0.7, 0.6, 0.02};
    double fastPID[3] ={1.6, 2.0, 0.085};



public:
    SerialHandler();
    void resetCommand();
    int runCommand();
    void resetPID();
    void read();
    void doPID();
    double encToPWM(double input, double &prevInput, long &prevTime, int encoderTicksPerRev);
    // void testPID();
};
#endif