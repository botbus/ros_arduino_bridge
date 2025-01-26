#ifndef SETUP_DEFINITIONS_H
#define SETUP_DEFINITIONS_H
#include <FreeRTOS.h>
#include <string>
#include <task.h>
#include <semphr.h>
#include <atomic>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <PID_v1.h>

constexpr long BAUDRATE = 115200;
constexpr int PID_RATE = 30;
const int PID_INTERVAL = 1000 / PID_RATE;
extern unsigned long nextPID;
constexpr long AUTO_STOP_INTERVAL = 1000;
long lastMotorCommand{AUTO_STOP_INTERVAL};
unsigned long nextPID = PID_INTERVAL;
unsigned long readTime = 0;
#define STACK_SIZE 512
#define READ_INTERVAL 3
StaticTask_t xTaskBuffer_motorIMU;
StaticTask_t xTaskBuffer_wheelENC;
StackType_t xStack_motorIMU[STACK_SIZE];
StackType_t xStack_wheelENC[STACK_SIZE];
SemaphoreHandle_t xSemaphore = NULL;
StaticSemaphore_t xMutexBuffer;
TaskHandle_t motorIMUTask, wheelENCTask;

struct Wheel
{
    double input;
    double output;
    double setpoint;
    double prevInput;
    long prevTime;
    bool reverse;
    int ticksPerRev;
};

Wheel left;
Wheel right;

int MAX_PWM = 255;
double Kp = 1.6;
double Ki = 2.0;
double Kd = 0.085;

PID leftWheelPID(&left.input, &left.output, &left.setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
PID rightWheelPID(&right.input, &right.output, &right.setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);

#endif