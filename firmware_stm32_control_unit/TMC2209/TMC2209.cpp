#include "TMC2209.hpp"
#include "gpio.hh"
#include "log.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/types.h>

namespace tmc2209 {


uint8_t TMC2209::crc8(uint8_t *datagram, size_t datagram_size_with_crc)
{
  uint8_t crc = 0;
  uint8_t byte;
  for (uint8_t i=0; i<(datagram_size_with_crc - 1); ++i)
  {
    byte = (datagram[i]);
    for (uint8_t j=0; j<8; ++j)
    {
      if ((crc >> 7) ^ (byte & 0x01))
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc = crc << 1;
      }
      byte = byte >> 1;
    }
  }
  return crc;
}

TMC2209::TMC2209(UART_HandleTypeDef *huart, uint8_t address, gpio::Pin enPin)
    : huart_(huart), addr_(address & 0x0F), en_(enPin) {}




bool TMC2209::writeRegister(Reg reg, uint32_t value, uint32_t timeoutMs) {
  // Write frame: 0x05, addr, (reg|0x80), data[3], data[2], data[1], data[0],
  // crc
  uint8_t frame[8];
  frame[0] = 0x05;
  frame[1] = static_cast<uint8_t>(addr_); // node address (bits 0-1)
  frame[2] = static_cast<uint8_t>(static_cast<uint8_t>(reg) | 0x80);
  frame[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
  frame[4] = static_cast<uint8_t>((value >> 16) & 0xFF);
  frame[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
  frame[6] = static_cast<uint8_t>(value & 0xFF);
  frame[7] = crc8(frame, 8);
  log_info("TMC2209: Write Reg 0x%02X = 0x%08X\r\n", (unsigned int)reg,
         (unsigned int)value);

  HAL_StatusTypeDef res=HAL_UART_Transmit(huart_, const_cast<uint8_t *>(frame), sizeof(frame), timeoutMs);
  if(res!=HAL_OK){
    log_warn("TMC2209: Write Reg 0x%02X - TX failed with %d", (unsigned int)reg, (int)res);
    return false;
  }
  log_debug("TMC2209: Write Reg 0x%02X - TX OK", (unsigned int)reg);
  return true;
}

bool TMC2209::readRegister(Reg reg, uint32_t &value, uint32_t timeoutMs) {
  // Read request: 0x05, addr, reg, crc
  uint8_t req[4];
  req[0] = 0x05;
  req[1] = static_cast<uint8_t>(addr_); // read (bit 7 = 0)
  req[2] = static_cast<uint8_t>(reg);
  req[3] = crc8(req, 4);
  
  // Response: 0x05, addr, reg, data[3], data[2], data[1], data[0], crc
  // Note: Single-wire UART echoes TX bytes, so we receive: 4 echo bytes + 8 response bytes
  uint8_t resp[12];
  
  // Start RX first to be ready when response arrives, then TX request
  HAL_StatusTypeDef res = HAL_UART_Receive_IT(huart_, resp, sizeof(resp));
  if (res != HAL_OK) {
    log_info("TMC2209: Read Reg 0x%02X - RX IT failed with %d", (unsigned int)reg, (int)res);
    return false;
  }
  
  res = HAL_UART_Transmit_IT(huart_, const_cast<uint8_t *>(req), sizeof(req));
  if (res != HAL_OK) {
    log_info("TMC2209: Read Reg 0x%02X - TX IT failed with %d", (unsigned int)reg, (int)res);
    return false;
  }
  
  // Wait for both to complete with timeout
  uint32_t tickStart = HAL_GetTick();
  while ((huart_->gState != HAL_UART_STATE_READY) || 
         (huart_->RxState != HAL_UART_STATE_READY)) {
    if ((HAL_GetTick() - tickStart) > timeoutMs) {
      HAL_UART_AbortReceive_IT(huart_);
      log_info("TMC2209: Read Reg 0x%02X - TX/RX timeout", (unsigned int)reg);
      return false;
    }
  }
  
  // Skip first 4 bytes (echo of TX), actual response starts at resp[4]
  uint8_t *pResp = &resp[4];
  
  // Basic validation
  if (pResp[0] != 0x05){
    log_warn("TMC2209: Read Reg 0x%02X - Invalid start byte 0x%02X", (unsigned int)reg, (unsigned int)pResp[0]);
    return false;
  }
  if (pResp[1] != 0xFF){
    log_warn("TMC2209: Read Reg 0x%02X - Address mismatch ", (unsigned int)reg);
    return false;
  }
  if (pResp[2] != static_cast<uint8_t>(reg)){
    log_warn("TMC2209: Read Reg 0x%02X - Register mismatch 0x%02X", (unsigned int)reg, (unsigned int)pResp[2]);
    return false;
  }
  uint8_t rx_crc=pResp[7];   
  crc8(pResp, 8);
  if (rx_crc != pResp[7]){
    log_warn("TMC2209: Read Reg 0x%02X - CRC mismatch", (unsigned int)reg);
    return false;
  }

  value = (static_cast<uint32_t>(pResp[3]) << 24) |
          (static_cast<uint32_t>(pResp[4]) << 16) |
          (static_cast<uint32_t>(pResp[5]) << 8) |
          (static_cast<uint32_t>(pResp[6]));
  printf("TMC2209: Read Reg 0x%02X = 0x%08X\r\n", (unsigned int)reg,
         (unsigned int)value);
  return true;
}

void TMC2209::enable() {
  if (en_ != gpio::Pin::NO_PIN) {
    // Ensure EN pin is configured as output and set to low (enabled)
    // TMC2209 EN is typically low-active (EN low = enabled)
    gpio::Gpio::ConfigureGPIOOutput(en_, false);
  }
}

void TMC2209::disable() {
  if (en_ != gpio::Pin::NO_PIN) {
    // Set EN high to disable
    gpio::Gpio::Set(en_, true);
  }
}

bool TMC2209::setIHoldIRun(uint8_t ihold, uint8_t irun, uint8_t iholddelay) {
  ihold &= 0x1F;
  irun &= 0x1F;
  iholddelay &= 0x0F;
  uint32_t v = (static_cast<uint32_t>(iholddelay) << 16) |
               (static_cast<uint32_t>(irun) << 8) |
               (static_cast<uint32_t>(ihold));
  return writeRegister(Reg::IHOLD_IRUN, v);
}

} // namespace tmc2209
