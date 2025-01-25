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

#include "include/commands.h"
#include "include/imu_handler.h"
#include "include/setup_definitions.h"
#include "include/diff_controller.h"
#include "include/serial_handler.h"
#include "include/motor_driver.h"
#include "include/encoder_driver.h"

Wheel left;
Wheel right;
PID leftWheelPID(&left.input, &left.output, &left.setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
PID rightWheelPID(&right.input, &right.output, &right.setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
void setup()
{
  Serial.begin(BAUDRATE);
  while (!Serial)
    ;
  delay(1000);
  Serial.println("starting system");
  leftWheelPID.SetMode(AUTOMATIC);
  rightWheelPID.SetMode(AUTOMATIC);

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
  initIMU();

  while (1)
  {
    if (millis() - readTime >= READ_INTERVAL)
    {
      std::string sharedBuffer = readIMU();
      sharedBuffer += jsonFormat("ENC", readEncoder(LEFT), readEncoder(RIGHT));
      sharedBuffer.pop_back();
      sharedBuffer += "}|";
      Serial.flush();
      Serial.println(sharedBuffer.c_str());
      readTime = millis();
    }

    while (Serial.available() > 0)
    {
      chr = Serial.read();

      if (chr == 13)
      {
        if (parseArg == 1)
          argv1[indx] = '\0';
        else if (parseArg == 2)
          argv2[indx] = '\0';
        runCommand();
        resetCommand();
      }
      else if (chr == ' ')
      {
        if (parseArg == 0)
          parseArg = 1;
        else if (parseArg == 1)
        {
          argv1[indx] = '\0';
          parseArg = 2;
          indx = 0;
        }
      }
      else
      {
        if (parseArg == 0)
          cmd = chr;
        else if (parseArg == 1 && indx < sizeof(argv1) - 1)
          argv1[indx++] = chr;
        else if (parseArg == 2 && indx < sizeof(argv2) - 1)
          argv2[indx++] = chr;
      }
    }
    if (millis() - nextPID > PID_INTERVAL)
    {
      doPID();
      nextPID = millis();
    }

    if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
    {
      left.setpoint = 0;
      right.setpoint = 0;
    }
  }
}

void doPID()
{
  left.input = readEncoder(LEFT);
  left.setpoint = setpointLeft;
  right.input = readEncoder(RIGHT);
  right.setpoint = setpointRight;
  leftWheelPID.Compute();
  rightWheelPID.Compute();
  analogWrite(LEFT, left.output);
  analogWrite(RIGHT, right.output);
}