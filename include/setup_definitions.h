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
const int MAX_PWM = 255;
constexpr int PID_RATE = 30;
const int PID_INTERVAL = 1000 / PID_RATE;
extern unsigned long nextPID;
constexpr long AUTO_STOP_INTERVAL = 1000;
extern long lastMotorCommand;
unsigned long nextPID = PID_INTERVAL;
unsigned long readTime = 0;
#define STACK_SIZE 512
#define READ_INTERVAL 20
StaticTask_t xTaskBuffer_motorIMU;
StaticTask_t xTaskBuffer_wheelENC;
StackType_t xStack_motorIMU[STACK_SIZE];
StackType_t xStack_wheelENC[STACK_SIZE];
SemaphoreHandle_t xSemaphore = NULL;
StaticSemaphore_t xMutexBuffer;
TaskHandle_t motorIMUTask, wheelENCTask;
double Kp = 2, Ki = 5, Kd = 1;

struct Wheel
{
    int input;
    int output;
    int setpoint;
};
#endif