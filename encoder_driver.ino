#include "include/commands.h"
#include "include/diff_controller.h"
#include "include/encoder_driver.h"

volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
static const int ENC_STATES[4][4] = {{0, -1, 1, 2},
                                     {1, 0, 2, -1},
                                     {-1, 2, 0, 1},
                                     {2, 1, -1, 0}};

const TickType_t ENCDelay = 1 / portTICK_PERIOD_MS;

bool run_left_ISR{false};
bool run_right_ISR{false};

const int LEFT_ENC_PIN_A{4}; // pin 2
const int LEFT_ENC_PIN_B{5}; // pin 3

// below can be changed, but should be PORTC pins
const int RIGHT_ENC_PIN_A{3}; // pin A4
const int RIGHT_ENC_PIN_B{2}; // pin A5
void initMotorPins()
{
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), PIN_ISR_LEFT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), PIN_ISR_RIGHT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), PIN_ISR_LEFT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), PIN_ISR_RIGHT, CHANGE);
}
void PIN_ISR_LEFT()
{
  run_left_ISR = true;
}
void PIN_ISR_RIGHT()
{
  run_right_ISR = true;
}

void RUN_PIN_ISR_LEFT(void *pvParameters)
{
  (void)pvParameters;
  xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
  Serial.println("started LEFT ENC TASK");
  xSemaphoreGive(xSemaphore);
  while (1)
  {

    if (run_left_ISR)
    {
      run_left_ISR = false;
      noInterrupts(); // Disable interrupts
      if (xSemaphoreTake(xSemaphoreENC, (TickType_t)portMAX_DELAY) == pdTRUE)
      {
        long gpio_states = sio_hw->gpio_in;
        static uint8_t enc_last = 0;
        enc_last <<= 2;
        enc_last |= ((gpio_states >> LEFT_ENC_PIN_A << 1) | (gpio_states >> LEFT_ENC_PIN_B));
        // left_enc_pos += ENC_STATES[(enc_last & 0x0f)];

        interrupts();
        xSemaphoreGive(xSemaphoreENC);
      }
    }
    vTaskDelay(ENCDelay);
  }
}
void RUN_PIN_ISR_RIGHT(void *pvParameters)
{
  (void)pvParameters;
  xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
  Serial.println("started RIGHT ENC TASK");
  xSemaphoreGive(xSemaphore);
  while (1)
  {
    if (run_right_ISR)
    {

      if (xSemaphoreTake(xSemaphoreENC, (TickType_t)portMAX_DELAY) == pdTRUE)
      {
        noInterrupts();
        long gpio_states = sio_hw->gpio_in;
        static uint8_t enc_last_right = 0;
        enc_last_right <<= 2;
        enc_last_right |= ((gpio_states >> RIGHT_ENC_PIN_B) << 1) | (gpio_states >> RIGHT_ENC_PIN_B);
        // right_enc_pos += ENC_STATES[(enc_last_right & 0x0f)];

        interrupts();
        xSemaphoreGive(xSemaphoreENC);
      }
    }
    vTaskDelay(ENCDelay);
  }
}

void right_ENC()
{
  // static int prevVal = (digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B);
  static int prevVal = (digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B);
  int val = (digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B);

  Serial.print(prevVal);
  Serial.print("   :   ");
  Serial.print(val);
  Serial.print("   :   ");
  Serial.println(ENC_STATES[prevVal][val]);
}
long readEncoder(int i)
{
  if (i == LEFT)
    return left_enc_pos;
  else
    return right_enc_pos;
}

void resetEncoder(int i)
{
  if (i == LEFT)
  {
    left_enc_pos = 0L;
    return;
  }
  else
  {
    right_enc_pos = 0L;
    return;
  }
}

void resetEncoders()
{
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}
