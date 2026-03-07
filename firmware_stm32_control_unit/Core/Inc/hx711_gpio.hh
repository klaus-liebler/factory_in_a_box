#include "stdint.h"
#include "stdbool.h"
#include "main.h"
#include "stm32g4xx_hal.h"

#define CHANNEL_A 0
#define CHANNEL_B 1
#define interrupts() __enable_irq()
#define noInterrupts() __disable_irq()

class HX711 {
private:
  GPIO_TypeDef  *clk_gpio;
  uint16_t      clk_pin;
  GPIO_TypeDef  *dat_gpio;
  uint16_t      dat_pin;
  long       	Aoffset=0;
  float         Ascale=0.0;
  uint8_t		Again=0;
  long       	Boffset=0;
  float         Bscale=0.0;
  uint8_t		Bgain=0;

  uint8_t shiftIn(uint8_t bitOrder) {
      uint8_t value = 0;
      uint8_t i;

      for(i = 0; i < 8; ++i) {
          HAL_GPIO_WritePin(clk_gpio, clk_pin, GPIO_PIN_SET);
          if(bitOrder == 0)
              value |= HAL_GPIO_ReadPin(dat_gpio, dat_pin) << i;
          else
              value |= HAL_GPIO_ReadPin(dat_gpio, dat_pin) << (7 - i);
          HAL_GPIO_WritePin(clk_gpio, clk_pin, GPIO_PIN_RESET);
      }
      return value;
  }

public:
  // Constructor
  HX711(GPIO_TypeDef *clk_gpio, uint16_t clk_pin, GPIO_TypeDef *dat_gpio, uint16_t dat_pin):
    clk_gpio(clk_gpio), clk_pin(clk_pin), dat_gpio(dat_gpio), dat_pin(dat_pin) {


    GPIO_InitTypeDef  gpio = {0};
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = clk_pin;
    HAL_GPIO_Init(clk_gpio, &gpio);
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = dat_pin;
    HAL_GPIO_Init(dat_gpio, &gpio);
  }

  void set_scale(float Ascale_param, float Bscale_param) {
    // Set the scale. To calibrate the cell, run the program with a scale of 1, call the tare function and then the get_units function. 
    // Divide the obtained weight by the real weight. The result is the parameter to pass to scale
    Ascale = Ascale_param;
    Bscale = Bscale_param;
  }

  void set_gain(uint8_t Again_param, uint8_t Bgain_param) {
    // Define A channel's gain
    switch (Again_param) {
        case 128:		// channel A, gain factor 128
          Again = 1;
          break;
        case 64:		// channel A, gain factor 64
          Again = 3;
          break;
    }
    Bgain = 2;
  }

  void set_offset(long offset, uint8_t channel) {
    if(channel == CHANNEL_A) Aoffset = offset;
    else Boffset = offset;
  }

  bool is_ready() {
    if(HAL_GPIO_ReadPin(dat_gpio, dat_pin) == GPIO_PIN_RESET) {
      return 1;
    }
    return 0;
  }

  void wait_ready() {
    // Wait for the chip to become ready.
    while (!is_ready()) {
      HAL_Delay(0);
    }
  }

  long read(uint8_t channel) {
    wait_ready();
    unsigned long value = 0;
    uint8_t data[3] = { 0 };
    uint8_t filler = 0x00;

    noInterrupts();

    data[2] = shiftIn(1);
    data[1] = shiftIn(1);
    data[0] = shiftIn(1);

    uint8_t gain = 0;
    if(channel == 0) gain = Again;
    else gain = Bgain;

    for (unsigned int i = 0; i < gain; i++) {
      HAL_GPIO_WritePin(clk_gpio, clk_pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(clk_gpio, clk_pin, GPIO_PIN_RESET);
    }

    interrupts();

    // Replicate the most significant bit to pad out a 32-bit signed integer
    if (data[2] & 0x80) {
      filler = 0xFF;
    } else {
      filler = 0x00;
    }

    // Construct a 32-bit signed integer
    value = ( (unsigned long)(filler) << 24
        | (unsigned long)(data[2]) << 16
        | (unsigned long)(data[1]) << 8
        | (unsigned long)(data[0]) );

    return (long)(value);
  }

  long read_average(int8_t times, uint8_t channel) {
    long sum = 0;
    for (int8_t i = 0; i < times; i++) {
      sum += read(channel);
      HAL_Delay(0);
    }
    return sum / times;
  }

  double get_value(int8_t times, uint8_t channel) {
    long offset = 0;
    if(channel == CHANNEL_A) offset = Aoffset;
    else offset = Boffset;
    return read_average(times, channel) - offset;
  }

  void tare(uint8_t times, uint8_t channel) {
    read(channel); // Change channel
    double sum = read_average(times, channel);
    set_offset(sum, channel);
  }

  void tare_all(uint8_t times) {
    tare(times, CHANNEL_A);
    tare(times, CHANNEL_B);
  }

  float get_weight(int8_t times, uint8_t channel) {
    // Read load cell
    read(channel);
    float scale = 0;
    if(channel == CHANNEL_A) scale = Ascale;
    else scale = Bscale;
    return get_value(times, channel) / scale;
  }
};