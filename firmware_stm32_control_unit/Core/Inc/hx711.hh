
#include "gpio.hh"
#include "log.h"
#include "main.h"

#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string_view>

enum class Channel_And_Gain{
  CH_A_GAIN_128 = 0xA0,//for 25 pulses on the clock pin
  CH_B_GAIN_32 = 0xA8, //for 26 pulses on the clock pin
  CH_A_GAIN_64 = 0xAA, //for 27 pulses on the clock pin
};


enum class ReadoutState {
  WAIT_BETWEEN_MEASUREMENTS,
  WAIT_FOR_DATA_READY,
  WAIT_FOR_DMA_COMPLETION,
  ERROR,
};

struct CalibrationPoint {
  int32_t reading;
  double weight;
};



class HX711 {
private:
  SPI_HandleTypeDef* hspi;
  gpio::Pin data_pin;
  Channel_And_Gain mode;
  Channel_And_Gain mode_used_in_last_measurement[2] = {Channel_And_Gain::CH_A_GAIN_128, Channel_And_Gain::CH_A_GAIN_128};
  int32_t channelA__last_reading{INT32_MIN};
  int32_t channelB_last_reading{INT32_MIN};
  uint8_t TxData[8] ={0x2A, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,0x8,0x0};  
  uint8_t RxData[8]= {0};
  CalibrationPoint calibration_points_chA[2] = {
    {0, 0.0}, 
    {1, 1.0}  
  };

  CalibrationPoint calibration_points_chB[2] = {
    {0, 0.0}, 
    {1, 1.0}  
  };


  volatile ReadoutState readout_state = ReadoutState::WAIT_BETWEEN_MEASUREMENTS;
  uint32_t spi3_dma_test_start_count{0};
  uint32_t spi3_dma_test_cplt_count{0};
  uint32_t spi3_dma_test_error_count{0};
  uint32_t spi3_dma_test_last_error{0};
  uint32_t last_state_change{0};

  void convertArrayToUint32AndUpdateLastReading(uint8_t* arr, Channel_And_Gain current_mode) {
    // Read every second bit starting from bit index 1, collecting 24 bits total
    // Bits to extract: 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47
    uint32_t raw_value = 0;
    
    int output_bit = 0;
    for (int bit_index = 1; bit_index < 48; bit_index += 2) {
      int byte_index = bit_index / 8;
      int bit_in_byte = bit_index % 8;
      
      uint8_t bit = (arr[byte_index] >> bit_in_byte) & 1;
      raw_value |= ((uint32_t)bit) << output_bit;
      output_bit++;
    }
    
    // Handle two's complement: if bit 23 (sign bit) is set, sign-extend to 32-bit
    if (raw_value & 0x800000) {
      raw_value |= 0xFF000000;
    }

    
    int32_t result = (int32_t)raw_value;
    switch(current_mode) {
      case Channel_And_Gain::CH_A_GAIN_128:
        log_info("Channel A, Gain 128 selected. Raw value: %d", result);
        channelA__last_reading = result;
        break;
      case Channel_And_Gain::CH_B_GAIN_32:
        log_info("Channel B, Gain 32 selected. Raw value: %d", result);
        channelB_last_reading = result >> 2; // Aligning B gain 32 reading to be comparable with A gain 128 by shifting right 2 bits (multiplying by 32/128 = 1/4)
        break;
      case Channel_And_Gain::CH_A_GAIN_64:
        log_info("Channel A, Gain 64 selected. Raw value: %d", result);
        channelA__last_reading = result >> 1; // Aligning A gain 64 reading to be comparable with A gain 128 by shifting right 1 bit (multiplying by 64/128 = 1/2)
        break;
    }
  }


public:
  // Constructor
  HX711(SPI_HandleTypeDef* hspi, gpio::Pin data_pin, Channel_And_Gain initial_mode):
    hspi(hspi), data_pin(data_pin), mode(initial_mode) {
    mode_used_in_last_measurement[0] = initial_mode;
    mode_used_in_last_measurement[1] = initial_mode;
  }

void setup(void) {
  log_info("Starting SPI HX711 setup");
  if (hspi == nullptr) {
    log_error("SPI handle is null");
    readout_state = ReadoutState::ERROR;
    return;
  }
  
  // Don't start initial DMA transfer in setup - let loop() handle everything
  readout_state = ReadoutState::WAIT_BETWEEN_MEASUREMENTS;
  last_state_change = HAL_GetTick();
  log_info("HX711 setup complete, state machine ready");
}
int64_t last_weight_measurement_time = 0;

void Calibration(uint8_t step_0_or_1, Channel_And_Gain mode, double weight){
  step_0_or_1%=2; // Ensure step is either 0 or 1
  changeMode(mode);
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
}

void changeMode(Channel_And_Gain new_mode) {
  mode = new_mode;
  switch (new_mode) {
    case Channel_And_Gain::CH_A_GAIN_128:
    case Channel_And_Gain::CH_A_GAIN_64:
      channelA__last_reading = INT32_MIN; // Reset last reading for channel A gain 128
      break;
    case Channel_And_Gain::CH_B_GAIN_32:
      channelB_last_reading = INT32_MIN; // Reset last reading for channel B gain 32
      break;
  }
}

bool getLastReadingA(double* out_value) {
  if(channelA__last_reading == INT32_MIN) {
    return false;
  }
  // Linear calibration with two points
  int32_t reading_1 = calibration_points_chA[0].reading;
  double weight_1 = calibration_points_chA[0].weight;
  int32_t reading_2 = calibration_points_chA[1].reading;
  double weight_2 = calibration_points_chA[1].weight;
  
  if (reading_1 == reading_2) {
    return false;
  }
  *out_value = weight_1 + (static_cast<double>(channelA__last_reading) - reading_1) * (weight_2 - weight_1) / (reading_2 - reading_1);
  return true;
}

bool getLastReadingB(double* out_value) {
  if(channelB_last_reading == INT32_MIN) {
    return false;
  }
  // Linear calibration with two points
  int32_t reading_1 = calibration_points_chB[0].reading;
  double weight_1 = calibration_points_chB[0].weight;
  int32_t reading_2 = calibration_points_chB[1].reading;
  double weight_2 = calibration_points_chB[1].weight;
  
  if (reading_1 == reading_2) {
    return false;
  }
  
  *out_value = weight_1 + (static_cast<double>(channelB_last_reading) - reading_1) * (weight_2 - weight_1) / (reading_2 - reading_1);
  return true;
}

void loop(void) {
  uint32_t now = HAL_GetTick();
  switch (readout_state) {
    case ReadoutState::WAIT_BETWEEN_MEASUREMENTS:
      if(now - last_state_change < 2000U) {  // Wait 2 seconds after power-on or between measurements
        log_debug("WAIT_BETWEEN_MEASUREMENTS -->WAITING... %lu ms", now - last_state_change);
        return;
      }
      log_info("WAIT_BETWEEN_MEASUREMENTS --> WAIT_FOR_DATA_READY");
      readout_state = ReadoutState::WAIT_FOR_DATA_READY;
      break;
    case ReadoutState::WAIT_FOR_DATA_READY:
      {
        bool pin_state = gpio::Gpio::Get(data_pin);
        if(pin_state != false)  // Pin is HIGH, data not ready
        {
          log_debug("WAIT_FOR_DATA_READY: pin is HIGH (not ready), waiting... %lu ms", now - last_state_change);
          return;
        }
      }
      log_info("WAIT_FOR_DATA_READY --> DATA pin is LOW, starting DMA readout --> WAIT_FOR_DMA_COMPLETION");
      TxData[6] = static_cast<uint8_t>(mode);
      mode_used_in_last_measurement[1] = mode_used_in_last_measurement[0];
      mode_used_in_last_measurement[0] = mode;
      if (HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t*) (&TxData), (uint8_t*) (&RxData), sizeof(TxData)) != HAL_OK){
        log_warn("Failed to start DMA readout: state=%lu err=0x%08lX", (unsigned long)hspi->State, (unsigned long)hspi->ErrorCode);
        readout_state = ReadoutState::ERROR;
        
      } else {
        log_debug("Started DMA readout successfully");
        spi3_dma_test_start_count++;
        readout_state = ReadoutState::WAIT_FOR_DMA_COMPLETION;
      }
      break;
    case ReadoutState::WAIT_FOR_DMA_COMPLETION:
      if (hspi->ErrorCode != 0) {
        // DMA error occurred
        log_warn("WAIT_FOR_DMA_COMPLETION error: state=%lu err=0x%08lX", (unsigned long)hspi->State, (unsigned long)hspi->ErrorCode);
        spi3_dma_test_error_count++;
        spi3_dma_test_last_error = hspi->ErrorCode;
        readout_state = ReadoutState::ERROR;
      }else if (hspi->State != HAL_SPI_STATE_READY && hspi->State != HAL_SPI_STATE_RESET) {
          log_debug("WAIT_FOR_DMA_COMPLETION, still waiting, debug info: state=%lu err=0x%08lX, DMA start count=%lu, DMA complete count=%lu, DMA error count=%lu, last DMA error=0x%08lX",
                 (unsigned long)hspi->State, (unsigned long)hspi->ErrorCode, spi3_dma_test_start_count, spi3_dma_test_cplt_count, spi3_dma_test_error_count, spi3_dma_test_last_error);
          return; // Still waiting for DMA to complete
      } else if (hspi->State == HAL_SPI_STATE_READY) {
        log_info("DMA readout completed successfully");
        spi3_dma_test_cplt_count++;
        log_info("WAIT_FOR_DMA_COMPLETION --> CALCULATE_WEIGHT_AND_LOG ");
        convertArrayToUint32AndUpdateLastReading(RxData, mode_used_in_last_measurement[1]);
        readout_state = ReadoutState::WAIT_BETWEEN_MEASUREMENTS;
        last_state_change = HAL_GetTick();
      }else{
        log_debug("WAIT_FOR_DMA_COMPLETION, unexpected state, debug info: state=%lu err=0x%08lX, DMA start count=%lu, DMA complete count=%lu, DMA error count=%lu, last DMA error=0x%08lX",
                 (unsigned long)hspi->State, (unsigned long)hspi->ErrorCode, spi3_dma_test_start_count, spi3_dma_test_cplt_count, spi3_dma_test_error_count, spi3_dma_test_last_error);
        readout_state = ReadoutState::ERROR;
      }
      break;
    case ReadoutState::ERROR:
      log_error("HX711 readout error state reached. Please check previous logs for details.");
      return;
  }
  last_state_change = now;
  
}
};
