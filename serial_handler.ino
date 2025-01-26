#include "include/serial_handler.h"

SerialHandler::SerialHandler()
{
  leftWheelPID.SetMode(AUTO);
  rightWheelPID.SetMode(AUTO);
  leftWheelPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  rightWheelPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  leftWheelPID.SetSampleTime(PID_RATE);
  rightWheelPID.SetSampleTime(PID_RATE);
  left.ticksPerRev = 454;
  right.ticksPerRev = 424;
}
void SerialHandler::resetCommand()
{
  // Serial.println("Resetting command");
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1)); // sets argv1 array elements to 0's throughout
  memset(argv2, 0, sizeof(argv2));
  parseArg = 0;
  indx = 0;
}
void SerialHandler::resetPID()
{
  left.setpoint = 0;
  right.setpoint = 0;
}
/* Run a command. Commands are defined in commands.h */
int SerialHandler::runCommand()
{
  // Serial.print("Running command: ");
  // Serial.println(cmd);
  switch (cmd)
  {
  case RESET_ENCODERS:
    resetEncoders();
    left.setpoint = 0;
    right.setpoint = 0;
    left.prevInput = 0;
    right.prevInput = 0;
    break;
#ifdef TEST_MODE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    debug = !debug;
    break;
  case GET_PID_PARAMS:
    Serial.print("PID params: ");
    Serial.print(leftWheelPID.GetKp());
    Serial.print(" ");
    Serial.print(leftWheelPID.GetKi());
    Serial.print(" ");
    Serial.println(leftWheelPID.GetKd());
    break;
#endif
  case MOTOR_PID_PWM:
  {
    double leftTarget = atof(argv1);
    double rightTarget = atof(argv2);
    if (leftTarget == 0)
    {
      ; // left reverse to stay as is
    }
    else if (leftTarget > 0)
    {
      left.reverse = false; // target is positive, set reverse to false
    }
    else // target is negative, set reverse to true
    {
      left.reverse = true;
    }
    if (rightTarget == 0)
    {
      ;
    }
    else if (rightTarget > 0)
    {
      right.reverse = false;
    }
    else
    {
      right.reverse = true;
    }
    left.setpoint = abs(leftTarget);
    right.setpoint = abs(rightTarget);
    leftWheelPID.SetMode(AUTO);
    rightWheelPID.SetMode(AUTO);
    lastMotorCommand = millis();

    break;
  }
#ifdef TEST_MODE
  case MOTOR_RAW_PWM:

    leftWheelPID.SetMode(MAN);
    rightWheelPID.SetMode(MAN);
    lastMotorCommand = millis();
    mymotor.setMotorSpeeds((double)atoi(argv1), (double)atoi(argv2), atoi(argv1) < 0, atoi(argv2) < 0);
    break;

  case SET_PID_PARAMS:
  {
    float pid_args[3];
    char *p = argv1;
    for (int i = 0; i < 3; ++i)
    {
      pid_args[i] = atof(strtok_r(p, ":", &p));
    }
    Serial.print("Setting pid: ");
    Serial.print(pid_args[0], 3);
    Serial.print(" ");
    Serial.print(pid_args[1], 3);
    Serial.print(" ");
    Serial.println(pid_args[2], 3);
    rightWheelPID.SetTunings((double)pid_args[0], (double)pid_args[1], (double)pid_args[2]);
    leftWheelPID.SetTunings((double)pid_args[0], (double)pid_args[1], (double)pid_args[2]);
  }
  break;
#endif
  default:
    return -1;
  }
  return 0;
}

void SerialHandler::read()
{
  static char *argv[] = {argv1, argv2};

  while (Serial.available() > 0)
  {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13)
    {
      if (parseArg == 1)
        argv1[indx] = '\0';
      else if (parseArg == 2)
        argv2[indx] = '\0';
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ')
    {
      // Step through the arguments
      if (parseArg == 0)
        parseArg = 1;
      else if (parseArg == 1)
      {
        argv1[indx] = '\0';
        parseArg = 2;
        indx = 0;
      }
      continue;
    }
    else
    {
      if (parseArg == 0)
      {
        // The first parseArg is the single-letter command
        cmd = chr;
      }
      else if (parseArg == 1)
      {
        // Subsequent arguments can be more than one character
        argv1[indx] = chr;
        indx++;
      }
      else if (parseArg == 2)
      {
        argv2[indx] = chr;
        indx++;
      }
    }
  }
}
void SerialHandler::doPID()
{
  long currentTime = millis();
  if (currentTime - previousTime >= PID_RATE && leftWheelPID.GetMode() == AUTO && rightWheelPID.GetMode() == AUTO)
  {
    long leftEnc = readEncoder(LEFT);
    long rightEnc = readEncoder(RIGHT);
    left.input = encToPWM(leftEnc, left.prevInput, left.prevTime, left.ticksPerRev);      // target value is in pwm input is in ENC, output is in pwm
    right.input = encToPWM(rightEnc, right.prevInput, right.prevTime, right.ticksPerRev); // target value is in pwm input is in ENC, output is in pwmreadEncoder(RIGHT);
    if (abs(left.input) - abs(right.input) < 5)
      left.input = right.input = min(right.input, left.input);
    int diffLeft = abs(left.input - left.setpoint);
    int diffRight = abs(right.input - right.setpoint);
    
    if (diffLeft >= 0 && diffLeft <= 10) {
        leftWheelPID.SetTunings(slowPID[0], slowPID[1], slowPID[2]);
    } 
    else if (diffLeft > 10 && diffLeft <= 30) {
        leftWheelPID.SetTunings(midPID[0], midPID[1], midPID[2]);
    } 
    else if (diffLeft > 40 && diffLeft <= 255) {
        leftWheelPID.SetTunings(fastPID[0], fastPID[1], fastPID[2]);
    }


    if (diffRight >= 0 && diffRight <= 10) {
        rightWheelPID.SetTunings(slowPID[0], slowPID[1], slowPID[2]);
    } 
    else if (diffRight > 10 && diffRight <= 30) {
        rightWheelPID.SetTunings(midPID[0], midPID[1], midPID[2]);
    } 
    else if (diffRight > 30 && diffRight <= 255) {
        rightWheelPID.SetTunings(fastPID[0], fastPID[1], fastPID[2]);
    }



    leftWheelPID.Compute();
    rightWheelPID.Compute();
    left.prevInput = leftEnc;
    right.prevInput = rightEnc;


    // making sure the revs match if they are supposed to match
    if (abs(left.output) - abs(right.output) < 5)
      left.output = right.output = min(right.output, left.output);

    // // remove jittering at lower speeds
    // if ((left.output < 50 && left.setpoint < 50) || (left.input < 50 && left.setpoint < 50))
    //   left.output = left.input = left.setpoint = 0;
    // if ((right.output < 50 && right.setpoint < 50) || (right.input < 50 && right.setpoint < 50))
    //   right.output = right.input = right.setpoint = 0; 

#ifdef TEST_MODE
    if (debug)
    {
      Serial.print("PID: ");
      Serial.print("Input: ");
      Serial.print(left.input, 0);
      Serial.print(" ");
      Serial.print(right.input, 0);
      Serial.print("  Target: ");
      Serial.print(left.setpoint, 0);
      Serial.print(" ");
      Serial.print(right.setpoint, 0);
      Serial.print("  Output: ");
      Serial.print(left.output, 0);
      Serial.print(" ");
      Serial.println(right.output, 0);
    }
#endif
    mymotor.setMotorSpeeds(left.output, right.output, left.reverse, right.reverse);
    previousTime = currentTime;
  }
}
double SerialHandler::encToPWM(double input, double &prevInput, long &prevTime, int encoderTicksPerRev)
{
  const double maxRPM = 200.0;

  long currentTime = millis();
  long deltaTime = currentTime - prevTime;
  prevTime = currentTime;
  if (deltaTime <= 0)
  {
    return 0;
  }

  double rpm = abs(((input - prevInput) * 60.0 * 1000.0) / (deltaTime * encoderTicksPerRev));
  // rpm = input > prevInput ? rpm : -rpm;
  // prevInput = input;
  // Convert RPM to PWM (scale factor)
  double pwm = (rpm / maxRPM) * MAX_PWM;

  // Constrain PWM output to valid range
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  return pwm;
}

// void SerialHandler::testPID()
// {
//   static bool prevUp = false;
//   static bool up = true;
//   static int index{};
//   long currentTime = millis();
//   static long startTime = millis();
//   static std::vector<std::vector<double>> PID = {

//       {1.6, 2.0, 0.085},
//       {1.6, 2.1, 0.085},
//       {1.6, 2.2, 0.085},
//   };

//   if (next)
//   {
//     index = (index + 1) % PID.size();
//     up = true;
//     prevUp = false;
//     left.setpoint = 255;
//     right.setpoint = 255;
//     next = false;
//   }

//   if (prevUp != up && up == true)
//   {
//     prevUp = up;
//     std::ostringstream oss;
//     oss << "PID params: " << PID[index][0] << " " << PID[index][1] << " " << PID[index][2];
//     Serial.println(oss.str().c_str());
//     leftWheelPID.SetTunings(PID[index][0], PID[index][1], PID[index][2]);
//     rightWheelPID.SetTunings(PID[index][0], PID[index][1], PID[index][2]);
//   }

//   doPID();

//   static std::vector<double> leftInputs; // Vector to store the last 10 input values for left
//   leftInputs.push_back(left.input);      // Push the current input to the vector

//   if (leftInputs.size() > 10)
//   {
//     leftInputs.erase(leftInputs.begin()); // Erase the first element (oldest input)
//   }

//   double averageInput = 0.0;
//   for (double input : leftInputs)
//   {
//     averageInput += input;
//   }
//   averageInput /= leftInputs.size(); // Get the average of the last 10 inputs

//   if (abs(averageInput - left.setpoint) < 5.0)
//   { // Threshold of 1.0 for "close enough"

//     std::ostringstream oss;
//     oss << "Time taken to reach: " << up << " " << ((double)currentTime - (double)startTime) / 1000 << " s";

//     Serial.println(oss.str().c_str());
//     startTime = currentTime; // Reset the start time for the next iteration
//     up = !up;                // Toggle the setpoint (255 -> 0, 0 -> 255)
//     if (up == true)
//     {
//       index = (index + 1) % PID.size();
//       prevUp = false;
//     }
//     left.setpoint = up ? 255 : 0;
//     right.setpoint = up ? 255 : 0;
//   }
// }
