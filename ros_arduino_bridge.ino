/*********************************************************************
 *  ROSArduinoBridge

    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org

    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson

    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
// #include "commands.h"
#include "include/imu_handler.h"
#include "include/setup_definitions.h"

#include "include/diff_controller.h"
#include "include/serial_handler.h"
#include "include/motor_driver.h"
#include "include/encoder_driver.h"
unsigned long nextPID = PID_INTERVAL;




/* Dimensions of the buffer that the task being created will use as its stack.
  NOTE:  This is the number of words the stack will hold, not the number of
  bytes.  For example, if each stack item is 32-bits, and this is set to 100,
  then 400 bytes (100 * 32-bits) will be allocated. */
#define STACK_SIZE 200

/* Structure that will hold the TCB of the task being created. */
StaticTask_t xTaskBuffer_imu;
StaticTask_t xTaskBuffer_motor;

/* Buffer that the task being created will use as its stack.  Note this is
  an array of StackType_t variables.  The size of StackType_t is dependent on
  the RTOS port. */
StackType_t xStack_imu[STACK_SIZE];
StackType_t xStack_motor[STACK_SIZE];

SemaphoreHandle_t xSemaphore = NULL;
StaticSemaphore_t xMutexBuffer;

TaskHandle_t motorTask, imuTask;

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial)
    ;
  delay(1000);
  Serial.println("starting system");
  initIMU();
  initMotorPins();
  // xSemaphore = xSemaphoreCreateMutexStatic(&xMutexVuffer);
  xSemaphore = xSemaphoreCreateMutexStatic(&xMutexBuffer);

  motorTask = xTaskCreateStatic(motor_driver, "motor_driver", STACK_SIZE, NULL, configMAX_PRIORITIES - 1, xStack_motor, &xTaskBuffer_motor);
  vTaskCoreAffinitySet(motorTask, 1 << 0);  // Core 0

  imuTask = xTaskCreateStatic(imu_driver, "imu_driver", STACK_SIZE, NULL, configMAX_PRIORITIES - 1, xStack_imu, &xTaskBuffer_imu);
  vTaskCoreAffinitySet(imuTask, 1 << 1);  // Core 1
}



// void setup1() {
//   while (!Serial)
//     ;

// }

void imu_driver(void *pvParameters) {
  (void)pvParameters;
  xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
  Serial.println("started imu driver task");
  xSemaphoreGive(xSemaphore);
  const TickType_t xDelay = 3 / portTICK_PERIOD_MS;
  delay(100);
  while (1)
  {//Serial.println("reading imu");
  //  readIMU(xDelay);
  
  }
}
void loop() {
  // Serial.println("asd");
}
void motor_driver(void *pvParameters) {
  (void)pvParameters;
  xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
  Serial.println("started motor driver task");
  xSemaphoreGive(xSemaphore);
  const TickType_t yDelay = 3 / portTICK_PERIOD_MS;
  initMotorController();
  resetPID();
  delay(100);
  while (1) {
    if (run_left_ISR)
      RUN_PIN_ISR_LEFT();
    if (run_right_ISR)
      RUN_PIN_ISR_LEFT();
//Serial.println("Reading motor driver");
    if (xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY) == pdTRUE) {
      while (Serial.available() > 0) {
        // Read the next character
        chr = Serial.read();

        // Terminate a command with a CR
        if (chr == 13) {
          if (arg == 1)
            argv1[indx] = '\0';
          else if (arg == 2)
            argv2[indx] = '\0';
          runCommand();
          resetCommand();
        }
        // Use spaces to delimit parts of the command
        else if (chr == ' ') {
          // Step through the arguments
          if (arg == 0)
            arg = 1;
          else if (arg == 1) {
            argv1[indx] = '\0';
            arg = 2;
            indx = 0;
          }
          continue;
        } else {
          if (arg == 0) {
            // The first arg is the single-letter command
            cmd = chr;
          } else if (arg == 1) {
            // Subsequent arguments can be more than one character
            argv1[indx] = chr;
            indx++;
          } else if (arg == 2) {
            argv2[indx] = chr;
            indx++;
          }
        }
      }


      if (millis() > nextPID) {
        updatePID();
        nextPID += PID_INTERVAL;
      }
//      if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
  //      setMotorSpeeds(0, 0);
     //   moving = 0;
   //   }

      xSemaphoreGive(xSemaphore);
      vTaskDelay(yDelay);
    }
  }
}
