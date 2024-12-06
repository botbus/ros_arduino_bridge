#include "include/commands.h"
#include "include/diff_controller.h"
#include "include/encoder_driver.h"

std::atomic<long> left_enc_pos{0};
std::atomic<long> right_enc_pos{0};
static const int ENC_STATES[4][4] = {{0, -1, 1, 2},
                                     {1, 0, 2, -1},
                                     {-1, 2, 0, 1},
                                     {2, 1, -1, 0}};

const TickType_t ENCDelay = 1 / portTICK_PERIOD_MS;

bool run_left_ISR{false};
bool run_right_ISR{false};

const int LEFT_ENC_PIN_A{4}; // pin 2
const int LEFT_ENC_PIN_B{5}; // pin 3
bool resetEncoderPos{false};
// below can be changed, but should be PORTC pins
const int RIGHT_ENC_PIN_A{3}; // pin A4
const int RIGHT_ENC_PIN_B{2}; // pin A5
void initMotorPins()
{
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);

  // attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), PIN_ISR_LEFT, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), PIN_ISR_RIGHT, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), PIN_ISR_LEFT, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), PIN_ISR_RIGHT, CHANGE);
}
// void PIN_ISR_LEFT()
// {
//   run_left_ISR = true;
// }
// void PIN_ISR_RIGHT()
// {
//   run_right_ISR = true;
// }

// void RUN_PIN_ISR_LEFT(void *pvParameters)
// {
//   (void)pvParameters;
//   xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
//   Serial.println("started LEFT ENC TASK");
//   xSemaphoreGive(xSemaphore);
//   while (1)
//   {

//     if (run_left_ISR)
//     {
//       run_left_ISR = false;
//       noInterrupts(); // Disable interrupts
//       if (xSemaphoreTake(xSemaphoreENC, (TickType_t)portMAX_DELAY) == pdTRUE)
//       {
//         long gpio_states = sio_hw->gpio_in;
//         static uint8_t enc_last = 0;
//         enc_last <<= 2;
//         enc_last |= ((gpio_states >> LEFT_ENC_PIN_A << 1) | (gpio_states >> LEFT_ENC_PIN_B));
//         // left_enc_pos += ENC_STATES[(enc_last & 0x0f)];

//         interrupts();
//         xSemaphoreGive(xSemaphoreENC);
//       }
//     }
//     vTaskDelay(ENCDelay);
//   }
// }
// void RUN_PIN_ISR_RIGHT(void *pvParameters)
// {
//   (void)pvParameters;
//   xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
//   Serial.println("started RIGHT ENC TASK");
//   xSemaphoreGive(xSemaphore);
//   while (1)
//   {
//     if (run_right_ISR)
//     {

//       if (xSemaphoreTake(xSemaphoreENC, (TickType_t)portMAX_DELAY) == pdTRUE)
//       {
//         noInterrupts();
//         long gpio_states = sio_hw->gpio_in;
//         static uint8_t enc_last_right = 0;
//         enc_last_right <<= 2;
//         enc_last_right |= ((gpio_states >> RIGHT_ENC_PIN_B) << 1) | (gpio_states >> RIGHT_ENC_PIN_B);
//         // right_enc_pos += ENC_STATES[(enc_last_right & 0x0f)];

//         interrupts();
//         xSemaphoreGive(xSemaphoreENC);
//       }
//     }
//     vTaskDelay(ENCDelay);
//   }
// }

// void right_ENC(void *pvParameters)
// {
//   (void)pvParameters;
//   xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
//   Serial.println("started RIGHT ENC TASK");
//   xSemaphoreGive(xSemaphore);
//   while (1)
//   {
//     // static int prevVal_left = (digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B);
//     static int newVal_right{0};
//     static unsigned int clockState_right = 0;
//     static unsigned int counterClockState_right = 0;

//     int valA_right = digitalRead(RIGHT_ENC_PIN_A);
//     int valB_right = digitalRead(RIGHT_ENC_PIN_B);

//     static int prevVal_right = (valA_right << 1) + valB_right;
//     newVal_right = (valA_right << 1) + valB_right;

//     int info_right = ENC_STATES[prevVal_right][newVal_right];

//     if (info_right == 1)
//     {
//       clockState_right |= (1 << newVal_right); // set the bit to 1
//     }
//     else if (info_right == -1)
//     {
//       counterClockState_right |= (1 << newVal_right);
//     }

//     if (prevVal_right != newVal_right && newVal_right == 3)
//     {
//       // changed to the non moving state, lets figure out what direction we went!

//       // for each clockwise and counterclockwise, the encoder state goes through 4 distinct states
//       // make sure it's gone through at least 3 of those (and assume if one is missing it's because I didn't read fast enough)
//       if (clockState_right == 0b1011 || clockState_right == 0b1101 || clockState_right == 0b1110 || clockState_right == 0b1111)
//       {
//         right_enc_pos += 1;
//         // xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
//         // Serial.println("RIGHT ++");
//         // xSemaphoreGive(xSemaphore);
//       }
//       if (counterClockState_right == 0b1011 || counterClockState_right == 0b1101 || counterClockState_right == 0b1110 || counterClockState_right == 0b1111)
//       {
//         right_enc_pos -= 1;
//         // xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
//         // Serial.println("RIGHT --");
//         // xSemaphoreGive(xSemaphore);
//       }

//       clockState_right = 0;
//       counterClockState_right = 0;
//     }
//     prevVal_right = newVal_right;
//     // vTaskDelay(ENCDelay);
//   }
// }

void left_ENC(void *pvParameters)
{
  (void)pvParameters;
  xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
  Serial.println("started LEFT ENC TASK");
  xSemaphoreGive(xSemaphore);
  int newVal_right{0};
  unsigned int clockState_right{0};
  unsigned int counterClockState_right{0};
  int newVal_left{0};
  unsigned int clockState_left = 0;
  unsigned int counterClockState_left = 0;
  while (1)
  {
    // static int prevVal_left = (digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B);
    if (resetEncoderPos)
    {
      left_enc_pos.store(0);
      right_enc_pos.store(0);
    }
    long gpio_states = sio_hw->gpio_in;
    int valA_right = gpio_states >> RIGHT_ENC_PIN_A;
    int valB_right = gpio_states >> RIGHT_ENC_PIN_B;
    int valA_left = gpio_states >> LEFT_ENC_PIN_A;
    int valB_left = gpio_states >> LEFT_ENC_PIN_B;

    static int prevVal_right = (valA_right << 1) + valB_right;
    newVal_right = (valA_right << 1) + valB_right;

    static int prevVal_left = (valA_left << 1) + valB_left;
    newVal_left = (valA_left << 1) + valB_left;

    int info_left = ENC_STATES[prevVal_left][newVal_left];
    int info_right = ENC_STATES[prevVal_right][newVal_right];

    if (info_right == 1)
    {
      clockState_right |= (1 << newVal_right); // set the bit to 1
    }
    else if (info_right == -1)
    {
      counterClockState_right |= (1 << newVal_right);
    }

    if (info_left == 1)
    {
      clockState_left |= (1 << newVal_left); // set the bit to 1
    }
    else if (info_left == -1)
    {
      counterClockState_left |= (1 << newVal_left);
    }

    if (prevVal_right != newVal_right && newVal_right == 3)
    {
      // changed to the non moving state, lets figure out what direction we went!

      // for each clockwise and counterclockwise, the encoder state goes through 4 distinct states
      // make sure it's gone through at least 3 of those (and assume if one is missing it's because I didn't read fast enough)
      if (clockState_right == 0b1011 || clockState_right == 0b1101 || clockState_right == 0b1110 || clockState_right == 0b1111)
      {
        right_enc_pos.fetch_add(1);
        // xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
        // Serial.println("RIGHT ++");
        // xSemaphoreGive(xSemaphore);
      }
      if (counterClockState_right == 0b1011 || counterClockState_right == 0b1101 || counterClockState_right == 0b1110 || counterClockState_right == 0b1111)
      {
        right_enc_pos.fetch_sub(1);
        // xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
        // Serial.println("RIGHT --");
        // xSemaphoreGive(xSemaphore);
      }

      clockState_right = 0;
      counterClockState_right = 0;
    }

    if (prevVal_left != newVal_left && newVal_left == 3)
    {
      // changed to the non moving state, lets figure out what direction we went!

      // for each clockwise and counterclockwise, the encoder state goes through 4 distinct states
      // make sure it's gone through at least 3 of those (and assume if one is missing it's because I didn't read fast enough)
      if (clockState_left == 0b1011 || clockState_left == 0b1101 || clockState_left == 0b1110 || clockState_left == 0b1111)
      {
        left_enc_pos.fetch_add(1);
        // xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
        // Serial.println("LEFT ++");
        // xSemaphoreGive(xSemaphore);
      }
      if (counterClockState_left == 0b1011 || counterClockState_left == 0b1101 || counterClockState_left == 0b1110 || counterClockState_left == 0b1111)
      {
        left_enc_pos.fetch_sub(1);
        // xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
        // Serial.println("LEFT --");
        // xSemaphoreGive(xSemaphore);
      }

      clockState_left = 0;
      counterClockState_left = 0;
    }
    prevVal_right = newVal_right;
    prevVal_left = newVal_left;
    vTaskDelay(ENCDelay);
  }
}
long readEncoder(int i)
{
  if (i == LEFT)
    return left_enc_pos.load();
  else
    return right_enc_pos.load();
}

void resetEncoders()
{
  resetEncoderPos = true;
}
