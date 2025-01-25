
#include <string>
#include "include/commands.h"
#include "include/setup_definitions.h"
#include "include/motor_driver.h"
#include "include/diff_controller.h"
#include "include/encoder_driver.h"
#include "include/serial_handler.h"

char cmd;
// A pair of varibles to help parse serial commands (thanks Fergs)
int parseArg = 0;
int indx = 0;
// Variable to hold an input character
char chr;
// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;
long lastMotorCommand{AUTO_STOP_INTERVAL};
/* Clear the current command parameters */
void resetCommand()
{
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1)); // sets argv1 array elements to 0's throughout
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  parseArg = 0;
  indx = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand()
{
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd)
  {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;

  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;

  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;

  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    // Serial.println("OK");
    break;

  case DIGITAL_WRITE:
    if (arg2 == 0)
      digitalWrite(arg1, LOW);
    else if (arg2 == 1)
      digitalWrite(arg1, HIGH);
    // Serial.println("OK");
    break;

  case PIN_MODE:
    if (arg2 == 0)
      pinMode(arg1, INPUT);
    else if (arg2 == 1)
      pinMode(arg1, OUTPUT);
    // Serial.println("OK");
    break;

    // case READ_ENCODERS:
    // // Serial.print("ENC:{")
    //   // Serial.print(readEncoder(LEFT));
    //   // Serial.print(",");
    //   // Serial.println(readEncoder(RIGHT));
    //   // Serial.println("}")

    //   break;

  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    resetICM();
    // Serial.println("OK");
    break;

  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0)
    {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else
      moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    // Serial.println("OK");
    break;
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    // Serial.println("OK");
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != NULL)
    {
      pid_args[i] = atoi(str);
      i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    // Serial.println("OK");
    break;

  default:
    // Serial.println("Invalid Command");
    return -1;
  }
  return 0;
}
