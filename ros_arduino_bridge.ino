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
#include <string>
#include <task.h>
#include <semphr.h>
#include <atomic>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "include/commands.h"
#include "include/imu_handler.h"
#include "include/setup_definitions.h"

#include "include/diff_controller.h"
#include "include/serial_handler.h"
#include "include/motor_driver.h"
#include "include/encoder_driver.h"
unsigned long nextPID = PID_INTERVAL;

#define STACK_SIZE 512

StaticTask_t xTaskBuffer_motorIMU;
StaticTask_t xTaskBuffer_wheelENC;

StackType_t xStack_motorIMU[STACK_SIZE];
StackType_t xStack_wheelENC[STACK_SIZE];

SemaphoreHandle_t xSemaphore = NULL;
StaticSemaphore_t xMutexBuffer;

TaskHandle_t motorIMUTask, wheelENCTask;

void setup()
{
  Serial.begin(BAUDRATE);
  while (!Serial)
    ;
  delay(1000);
  Serial.println("starting system");
  initIMU();
  initMotorPins();

  xSemaphore = xSemaphoreCreateMutexStatic(&xMutexBuffer);

  motorIMUTask = xTaskCreateStatic(motorIMUdriver, "motorIMUdriver", STACK_SIZE, NULL, configMAX_PRIORITIES - 1, xStack_motorIMU, &xTaskBuffer_motorIMU);
  vTaskCoreAffinitySet(motorIMUTask, 1 << 0);

  wheelENCTask = xTaskCreateStatic(wheelENC, "wheelENC", STACK_SIZE, NULL, configMAX_PRIORITIES - 1, xStack_wheelENC, &xTaskBuffer_wheelENC);
  vTaskCoreAffinitySet(wheelENCTask, 1 << 1);
}

void loop()
{
}

void motorIMUdriver(void *pvParameters)
{
  (void)pvParameters;
  xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
  Serial.println("started motor driver task");
  xSemaphoreGive(xSemaphore);
  const TickType_t yDelay = 100 / portTICK_PERIOD_MS;
  initMotorController();
  resetPID();
  delay(100);
  while (1)
  {
    std::string sharedBuffer{};
    sharedBuffer = readIMU();
    sharedBuffer += "\"ENC\":[";
    sharedBuffer += std::to_string(readEncoder(LEFT));
    sharedBuffer += ",";
    sharedBuffer += std::to_string(readEncoder(RIGHT));
    sharedBuffer += "]}|";
    Serial.flush();
    Serial.println(sharedBuffer.c_str());
    while (Serial.available() > 0)
    {
      // Read the next character
      chr = Serial.read();

      // Terminate a command with a CR
      if (chr == 13)
      {
        if (arg == 1)
          argv1[indx] = '\0';
        else if (arg == 2)
          argv2[indx] = '\0';
        runCommand(sharedBuffer);
        resetCommand();
      }
      // Use spaces to delimit parts of the command
      else if (chr == ' ')
      {
        // Step through the arguments
        if (arg == 0)
          arg = 1;
        else if (arg == 1)
        {
          argv1[indx] = '\0';
          arg = 2;
          indx = 0;
        }
        continue;
      }
      else
      {
        if (arg == 0)
        {
          // The first arg is the single-letter command
          cmd = chr;
        }
        else if (arg == 1)
        {
          // Subsequent arguments can be more than one character
          argv1[indx] = chr;
          indx++;
        }
        else if (arg == 2)
        {
          argv2[indx] = chr;
          indx++;
        }
      }
    }

    if (millis() > nextPID)
    {
      updatePID();
      nextPID += PID_INTERVAL;
    }
    // Check to see if we have exceeded the auto-stop interval
    if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
    {
     
      setMotorSpeeds(0, 0);
      moving = 0;
    }
  }
}
