#include "include/commands.h"
#include "include/encoder_driver.h"

std::atomic<long> left_enc_pos{0};
std::atomic<long> right_enc_pos{0};
static const int ENC_STATES[4][4] = {{0, -1, 1, 2},
                                     {1, 0, 2, -1},
                                     {-1, 2, 0, 1},
                                     {2, 1, -1, 0}};

const TickType_t ENCDelay = 1 / portTICK_PERIOD_MS;
bool resetEncoderPos{false};
const int LEFT_ENC_PIN_A{4};  // pin 2
const int LEFT_ENC_PIN_B{5};  // pin 3
const int RIGHT_ENC_PIN_A{3}; // pin A4
const int RIGHT_ENC_PIN_B{2}; // pin A5
// void initMotorPins()
// {
//   pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
//   pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
//   pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
//   pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
// }

void wheelENC(void *pvParameters)
{
  (void)pvParameters;

  int newVal_right{0};
  unsigned int clockState_right{0};
  unsigned int counterClockState_right{0};
  int newVal_left{0};
  unsigned int clockState_left = 0;
  unsigned int counterClockState_left = 0;
  gpio_set_dir(LEFT_ENC_PIN_A, false);
  gpio_pull_up(LEFT_ENC_PIN_A);
  gpio_set_dir(LEFT_ENC_PIN_B, false);
  gpio_pull_up(LEFT_ENC_PIN_B);

  gpio_set_dir(RIGHT_ENC_PIN_A, false);
  gpio_pull_up(RIGHT_ENC_PIN_A);
  gpio_set_dir(RIGHT_ENC_PIN_B, false);
  gpio_pull_up(RIGHT_ENC_PIN_B);

  while (1)
  {
    // static int prevVal_left = (digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B);
    if (resetEncoderPos)
    {
      left_enc_pos.store(0);
      right_enc_pos.store(0);
      resetEncoderPos = false;
    }
    long gpio_states = sio_hw->gpio_in;
    int valA_right = (gpio_states >> RIGHT_ENC_PIN_A) & 1;
    int valB_right = (gpio_states >> RIGHT_ENC_PIN_B) & 1;
    int valA_left = (gpio_states >> LEFT_ENC_PIN_A) & 1;
    int valB_left = (gpio_states >> LEFT_ENC_PIN_B) & 1;
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
      if (clockState_right == 0b1011 || clockState_right == 0b1101 || clockState_right == 0b1110 || clockState_right == 0b1111)
      {
        right_enc_pos.fetch_add(1);
      }
      if (counterClockState_right == 0b1011 || counterClockState_right == 0b1101 || counterClockState_right == 0b1110 || counterClockState_right == 0b1111)
      {
        right_enc_pos.fetch_sub(1);
      }

      clockState_right = 0;
      counterClockState_right = 0;
    }

    if (prevVal_left != newVal_left && newVal_left == 3)
    {
      if (clockState_left == 0b1011 || clockState_left == 0b1101 || clockState_left == 0b1110 || clockState_left == 0b1111)
      {
        left_enc_pos.fetch_add(1);
      }
      if (counterClockState_left == 0b1011 || counterClockState_left == 0b1101 || counterClockState_left == 0b1110 || counterClockState_left == 0b1111)
      {
        left_enc_pos.fetch_sub(1);
      }

      clockState_left = 0;
      counterClockState_left = 0;
    }
    prevVal_right = newVal_right;
    prevVal_left = newVal_left;

    // vTaskDelay(ENCDelay);
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
