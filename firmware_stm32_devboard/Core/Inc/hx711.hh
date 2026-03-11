
#include "gpio.hh"
#include "log.h"
#include "main.h"

#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string_view>
#include <utility>
#include <vector>

enum class Channel_And_Gain{
  CH_A_GAIN_128 = 0x40,//for 25 pulses on the clock pin
  CH_B_GAIN_32 = 0x50, //for 26 pulses on the clock pin
  CH_A_GAIN_64 = 0x54, //for 27 pulses on the clock pin
};

 struct CalibrationPoint {
    int32_t reading;
    double weight;
  };

template<size_t AVERAGE_OVER_READINGS = 8>
class HX711 {
  struct u32i32_converter{
    union
    {
      uint32_t uint32_value;
      int32_t int32_value;
    };
  };

  enum class ReadoutState {
    WAIT_FOR_DATA_READY,
    WAIT_FOR_DMA_COMPLETION,
    ERROR,
  };

 

private:
  SPI_HandleTypeDef* hspi;
  gpio::Pin data_pin;
  Channel_And_Gain channel_and_gain=Channel_And_Gain::CH_A_GAIN_128;
  std::array<int32_t, AVERAGE_OVER_READINGS> readings_buffer{INT32_MIN};
  size_t current_reading_index{0};
  

  //see https://community.st.com/t5/stm32-mcus-products/spi-mosi-idle-state-quirk-sometimes-high-sometimes-low/td-p/298590
  uint8_t TxData[8] ={0x55, 0x55, 0x55, 0x55, 0x55, 0x55,(uint8_t)Channel_And_Gain::CH_A_GAIN_128,0x00};  
  uint8_t RxData[8]= {0};
  
  ReadoutState readout_state = ReadoutState::ERROR; // Start in error state until setup is called to ensure loop() doesn't run before setup()

  void convertArrayToUint32AndUpdateLastReading() {
    // Read every second bit starting from bit index 2, collecting 24 bits total
    // Bits to extract: 2, 4, 6, ..., 48 (24 bits total)
    
    uint32_t raw_value = 
    ((RxData[0] & 0b01000000)>>6)<<23 |
    ((RxData[0] & 0b00010000)>>4)<<22 |
    ((RxData[0] & 0b00000100)>>2)<<21 |
    ((RxData[0] & 0b00000001)>>0)<<20 |
    ((RxData[1] & 0b01000000)>>6)<<19 |
    ((RxData[1] & 0b00010000)>>4)<<18 |
    ((RxData[1] & 0b00000100)>>2)<<17 |
    ((RxData[1] & 0b00000001)>>0)<<16 |
    ((RxData[2] & 0b01000000)>>6)<<15 |
    ((RxData[2] & 0b00010000)>>4)<<14 |
    ((RxData[2] & 0b00000100)>>2)<<13 |
    ((RxData[2] & 0b00000001)>>0)<<12 |
    ((RxData[3] & 0b01000000)>>6)<<11 |
    ((RxData[3] & 0b00010000)>>4)<<10 |
    ((RxData[3] & 0b00000100)>>2)<<9  |
    ((RxData[3] & 0b00000001)>>0)<<8  |
    ((RxData[4] & 0b01000000)>>6)<<7  |
    ((RxData[4] & 0b00010000)>>4)<<6  |
    ((RxData[4] & 0b00000100)>>2)<<5  |
    ((RxData[4] & 0b00000001)>>0)<<4  |
    ((RxData[5] & 0b01000000)>>6)<<3  |
    ((RxData[5] & 0b00010000)>>4)<<2  |
    ((RxData[5] & 0b00000100)>>2)<<1  |
    ((RxData[5] & 0b00000001)>>0)<<0;
  

    log_debug("RxData: 0x%02X %02X %02X %02X %02X %02X %02X %02X Raw read complete, raw_value=0x%08lX", 
             RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7], raw_value);
		u32i32_converter u32i32;
    u32i32.uint32_value = raw_value << 8;
    int32_t result = u32i32.int32_value/256;
    readings_buffer[current_reading_index] = result;
    current_reading_index = (current_reading_index + 1) % AVERAGE_OVER_READINGS;
    log_debug("Updated readings buffer with new value: %ld, current buffer index for next reading: %lu", (long)result, (unsigned long)current_reading_index);
  
  }


public:
  // Constructor
  HX711(SPI_HandleTypeDef* hspi, gpio::Pin data_pin, Channel_And_Gain channel_and_gain=Channel_And_Gain::CH_A_GAIN_128) :
    hspi(hspi),
    data_pin(data_pin),
    channel_and_gain(channel_and_gain)
  {
    TxData[6] = (uint8_t)channel_and_gain;
  }

void Setup(void) {
  log_info("Starting SPI HX711 setup");
  if (hspi == nullptr) {
    log_error("SPI handle is null");
    readout_state = ReadoutState::ERROR;
    return;
  }
  
  //Fire an initial dummy read to set the MOSI line to the correct idle state (LOW) before the first real readout cycle starts.
  uint8_t dummy_tx[4] = {0};
  HAL_SPI_Transmit(hspi, dummy_tx, 4, HAL_MAX_DELAY);
  this->readout_state = ReadoutState::WAIT_FOR_DATA_READY; // Start in WAIT_FOR_DATA_READY state to begin the measurement cycle
  log_info("SPI HX711 setup complete including initial dummy read, state machine initialized");
}

void loop(void) {
  switch (readout_state) {
    case ReadoutState::WAIT_FOR_DATA_READY:
      {
        bool pin_state = gpio::Gpio::Get(data_pin);
        if(pin_state)  // Pin is HIGH, data not ready
        {
          log_debug("WAIT_FOR_DATA_READY: pin is HIGH (not ready), waiting... ");
          return;
        }
      }
      log_debug("WAIT_FOR_DATA_READY --> DATA IS READY, starting DMA readout --> WAIT_FOR_DMA_COMPLETION");
      
      if (HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t*) (&TxData), (uint8_t*) (&RxData), sizeof(TxData)) != HAL_OK){
        log_warn("Failed to start DMA readout: state=%lu err=0x%08lX", (unsigned long)hspi->State, (unsigned long)hspi->ErrorCode);
        readout_state = ReadoutState::ERROR;
        
      } else {
        log_debug("Started DMA readout successfully");
        readout_state = ReadoutState::WAIT_FOR_DMA_COMPLETION;
      }
      break;
    case ReadoutState::WAIT_FOR_DMA_COMPLETION:
      if (hspi->ErrorCode != 0) {
        // DMA error occurred
        log_warn("WAIT_FOR_DMA_COMPLETION error: state=%lu err=0x%08lX", (unsigned long)hspi->State, (unsigned long)hspi->ErrorCode);
        readout_state = ReadoutState::ERROR;
      }else if (hspi->State != HAL_SPI_STATE_READY && hspi->State != HAL_SPI_STATE_RESET) {
          log_debug("WAIT_FOR_DMA_COMPLETION, still waiting, debug info: state=%lu err=0x%08lX",
                 (unsigned long)hspi->State, (unsigned long)hspi->ErrorCode);
          return; // Still waiting for DMA to complete
      } else if (hspi->State == HAL_SPI_STATE_READY) {
        log_debug("DMA readout completed successfully");
        convertArrayToUint32AndUpdateLastReading();
        readout_state = ReadoutState::WAIT_FOR_DATA_READY;
      }else{
        log_debug("WAIT_FOR_DMA_COMPLETION, unexpected state, debug info: state=%lu err=0x%08lX",
                 (unsigned long)hspi->State, (unsigned long)hspi->ErrorCode);
        readout_state = ReadoutState::ERROR;
      }
      break;
    case ReadoutState::ERROR:
      log_error("HX711 readout error state reached. Please check previous logs for details.");
      return;
  }
}

/*
void Calibration(uint8_t step_0_or_1, Channel_And_Gain mode, double weight){
  //const std::vector<Channel_And_Gain> query_order_before_calibration = query_order;

  step_0_or_1%=2; // Ensure step is either 0 or 1
  
  switch (mode) {
    case Channel_And_Gain::CH_A_GAIN_128:
    case Channel_And_Gain::CH_A_GAIN_64:
      while(channelA__last_reading == INT32_MIN) {
        log_debug("Waiting for valid reading for Channel A to be available...");
        loop();
      }
      calibration_points_chA[step_0_or_1].reading = channelA__last_reading;
      calibration_points_chA[step_0_or_1].weight = weight;
      break;
    case Channel_And_Gain::CH_B_GAIN_32:
      while(channelB_last_reading == INT32_MIN) {
        log_debug("Waiting for valid reading for Channel B to be available...");
        loop();
      }
      calibration_points_chB[step_0_or_1].reading = channelB_last_reading;
      calibration_points_chB[step_0_or_1].weight = weight;
      break;
  }

  applyQueryOrderAndResync(query_order_before_calibration);
}
*/


bool getLastRawValue(int32_t* out_value) {
  if(readings_buffer[(current_reading_index + AVERAGE_OVER_READINGS - 1) % AVERAGE_OVER_READINGS] == INT32_MIN) {
    return false;
  }
  // Linear calibration with two points 
  *out_value = readings_buffer[(current_reading_index + AVERAGE_OVER_READINGS - 1) % AVERAGE_OVER_READINGS];
  return true;
}


bool getAverageCalibratedWeight(double* out_value, std::array<CalibrationPoint, 2> calibration_points) {
  int32_t average_raw_value;
  if (!getAverageRawValue(&average_raw_value)) {
    return false;
  }
  
  int32_t reading_1 = calibration_points[0].reading;
  double weight_1 = calibration_points[0].weight;
  int32_t reading_2 = calibration_points[1].reading;
  double weight_2 = calibration_points[1].weight;
  
  if (reading_1 == reading_2) {
    return false;
  }
  *out_value = weight_1 + (average_raw_value - reading_1) * (weight_2 - weight_1) / (reading_2 - reading_1);
  return true;
}

bool getAverageRawValue(int32_t* out_value) {
  int64_t sum = 0;
  size_t count = 0;
  for (size_t i = 0; i < AVERAGE_OVER_READINGS; ++i) {
    int32_t reading = readings_buffer[i];
    if (reading != INT32_MIN) {
      sum += reading;
      count++;
    }
  }
  if (count == 0) {
    return false;
  }
  *out_value = static_cast<int32_t>(sum / count);
  return true;
}


};
