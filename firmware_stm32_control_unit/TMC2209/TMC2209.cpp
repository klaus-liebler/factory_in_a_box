#include "TMC2209.hpp"
#include "gpio.hh"
#include "log.h"
#include "tmc2209_reg.hpp"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/types.h>

#define CHECK_REG_WRITE(reg, value, error_msg)                                 \
  do {                                                                         \
    if (!writeRegister(reg, value)) {                                          \
      log_error("TMC2209: " error_msg);                                        \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define CHECK_REG_READ(reg, value, error_msg)                                  \
  do {                                                                         \
    if (!readRegister(reg, value)) {                                           \
      log_error("TMC2209: " error_msg);                                        \
      return false;                                                            \
    }                                                                          \
  } while (0)

namespace tmc2209 {

uint8_t TMC2209::crc8(uint8_t *datagram, size_t datagram_size_with_crc) {
  uint8_t crc = 0;
  uint8_t byte;
  for (uint8_t i = 0; i < (datagram_size_with_crc - 1); ++i) {
    byte = (datagram[i]);
    for (uint8_t j = 0; j < 8; ++j) {
      if ((crc >> 7) ^ (byte & 0x01)) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc = crc << 1;
      }
      byte = byte >> 1;
    }
  }
  return crc;
}

TMC2209::TMC2209(UART_HandleTypeDef *huart, uint8_t address, gpio::Pin enPin)
    : huart_(huart), addr_(address & 0x0F), en_(enPin) {}

bool TMC2209::fetchImportantRegistersForLocalMirroring() {
  // Verify communication by reading the IOIN register
  REG_FIELD::IOIN ioin;
  CHECK_REG_READ(RegIdx::IOIN, ioin.U32,
                 "Failed to read IOIN register during init");
  if (ioin.REG.version != 0x21) {
    log_error("TMC2209: Unexpected chip version 0x%02X",
              (unsigned int)ioin.REG.version);
    return false;
  }

  uint32_t gconfValue = 0;
  CHECK_REG_READ(RegIdx::GCONF, gconfValue,
                 "Failed to read GCONF register for local mirroring");
  gconf.U32 = gconfValue;
  return true;
}

bool TMC2209::shaftTurnReversed(bool reversed) {

  gconf.REG.shaft = reversed ? 1 : 0;
  CHECK_REG_WRITE(RegIdx::GCONF, gconf.U32, "Failed to write GCONF register");
  return true;
}

bool TMC2209::initForNormalSpeedAndUartBasedOperation(
    bool usePotimeterForCurrentScaling) {
  // Optionally, perform any initialization here
  enable();
  fetchImportantRegistersForLocalMirroring();

  gconf.REG.i_scale_analog = usePotimeterForCurrentScaling ? 0b0 : 0b1;
  gconf.REG.internal_rsense =
      0; // Current sense resistors. 0: Exteral; 1: internal
  gconf.REG.enable_spread_cycle =
      0; // Speed mode. 0: StealthChop; 1: SpreadCycle
  gconf.REG.shaft = 0;
  gconf.REG.pdn_disable = 1; // Disable PDN_UART input function
  gconf.REG.mstep_reg_select =
      1;                        // Microstepping. 0: MS1, MS2 pins; 1: register
  gconf.REG.multistep_filt = 1; // Step pulse filter. 0: disable; 1: enable

  CHECK_REG_WRITE(RegIdx::GCONF, gconf.U32,
                  "Failed to write GCONF register during init");
  return true;
}

bool TMC2209::performStealthChopAutoTuningForQuietOperation() {

  // 2. Stromparameter für 17HS4401S-22B (1.7A/Phase)
  log_info("IHOLD_IRUN: IHOLD=8 (0.5A), IRUN=31 (max 2.8A), IHOLDDELAY=6");
  REG_FIELD::IHOLD_IRUN ihold_irun_reg = {
      .REG = {.ihold = 8, .irun = 31, .iholddelay = 6}};
  CHECK_REG_WRITE(RegIdx::IHOLD_IRUN, ihold_irun_reg.U32,
                  "Failed to write IHOLD_IRUN register for StealthChop");

  // 3. TPWMTHRS: Übergang zwischen StealthChop und SpreadCycle.  (ca. 200-500
  // Hz) Formel: TPWMTHRS = f_CLK / (f_thrs * 256) Für f_thrs = 500Hz: 12Mhz /
  // (500 * 256) ≈ 94
  CHECK_REG_WRITE(RegIdx::TPWMTHRS, 94,
                  "Failed to write TPWMTHRS register for StealthChop");

  // 4. Initiale PWM Konfiguration
  log_info(
      "PWMCONF: pwm_offset=36, pwm_grad=2, pwm_freq=35kHz, "
      "pwm_autoscale=1, pwm_autograd=1, freewheel=0 (normal), "
      "pwm_reg=8, pwm_lim=12");
  REG_FIELD::PWMCONF pwmconf = {
      .REG = {.pwm_offset = 36,   // Default Wert
              .pwm_grad = 2,      // Sanfter Start
              .pwm_freq = 0b01,   // 35kHz
              .pwm_autoscale = 1, // Enable automatic tuning
              .pwm_autograd = 1,  // Enable automatic gradient adjustment
              .freewheel = 0b00,  // Normal operation
              .reserved = 0,
              .pwm_reg = 8,    // 4 increments (default with OTP2.1=0)
              .pwm_lim = 12}}; // Default Wert
  CHECK_REG_WRITE(RegIdx::PWMCONF, pwmconf.U32,
                  "Failed to write PWMCONF register for StealthChop");

  // 3. Motor mit konstanter Geschwindigkeit bewegen für Tuning
  // Datasheet empfiehlt:  "A typical range is 60-300 RPM"
  log_info("Start Automatic Tuning with 400 Steps per Second.");
  generateSteps(400); // 400 steps/s = 2 RPS = 120 RPM for 200 steps/rev motor
  for (int i = 0; i < 10; i++) {
    REG_FIELD::PWM_AUTO pwm_auto;
    CHECK_REG_READ(RegIdx::PWMCONF, pwmconf.U32,
                   "Failed to read PWMCONF register for tuning results");
    CHECK_REG_READ(RegIdx::PWM_AUTO, pwm_auto.U32,
                   "Failed to read PWM_AUTO register for tuning results");
    log_info("  Tuning Step %d: PWM_GRAD=%d, PWM_OFFSET=%d, "
             "PWM_OFFSET_AUTO=0x%03X, PWM_GRADIENT_AUTO=0x%03X",
             i + 1, pwmconf.REG.pwm_grad, pwmconf.REG.pwm_offset,
             (unsigned int)pwm_auto.REG.pwm_ofs_auto,
             (unsigned int)pwm_auto.REG.pwm_grad_auto);
    HAL_Delay(1000);
  }
  // Motor anhalten
  generateSteps(0); // Stop motor
  log_info("Automatic Tuning completed.");
  return true;
}

void TMC2209::printPrettyFullSystemState() {
    log_info("================================================");
    log_info("TMC2209: Full System State:");
    log_info("================================================");

    // GCONF Register
    log_info("GCONF (0x00): 0x%08X", (unsigned int)gconf.U32);
    log_info("  i_scale_analog: %d", gconf.REG.i_scale_analog);
    log_info("  internal_rsense: %d", gconf.REG.internal_rsense);
    log_info("  shaft: %d", gconf.REG.shaft);
    log_info("  index_otpw: %d", gconf.REG.index_otpw);
    log_info("  index_step: %d", gconf.REG.index_step);
    log_info("  pdn_disable: %d", gconf.REG.pdn_disable);
    log_info("  mstep_reg_select: %d", gconf.REG.mstep_reg_select);
    log_info("  multistep_filt: %d", gconf.REG.multistep_filt);

    // GSTAT Register
    uint32_t gstat = 0;
    if (readRegister(RegIdx::GSTAT, gstat)) {
      log_info("GSTAT (0x01): 0x%08X", (unsigned int)gstat);
      log_info("  reset: %d", (gstat >> 0) & 0x01);
      log_info("  drv_err: %d", (gstat >> 1) & 0x01);
      log_info("  uv_cp: %d", (gstat >> 2) & 0x01);
    }

    // IFCOUNT Register
    uint32_t ifcount = 0;
    if (readRegister(RegIdx::IFCNT, ifcount)) {
      log_info("IFCOUNT (0x02): 0x%08X (UART Interface Requests: %u)",
               (unsigned int)ifcount, (unsigned int)(ifcount & 0xFF));
    }

    // IHOLD_IRUN Register
    uint32_t iholdIrun = 0;
    if (readRegister(RegIdx::IHOLD_IRUN, iholdIrun)) {
      log_info("IHOLD_IRUN (0x10): 0x%08X", (unsigned int)iholdIrun);
      log_info("  ihold: %u", (unsigned int)(iholdIrun & 0x1F));
      log_info("  irun: %u", (unsigned int)((iholdIrun >> 8) & 0x1F));
      log_info("  iholddelay: %u", (unsigned int)((iholdIrun >> 16) & 0x0F));
    }

    // CHOPCONF Register
    uint32_t chopconf = 0;
    if (readRegister(RegIdx::CHOPCONF, chopconf)) {
      log_info("CHOPCONF (0x6C): 0x%08X", (unsigned int)chopconf);
      log_info("  toff: %u", (unsigned int)(chopconf & 0x0F));
      log_info("  hstrt: %u", (unsigned int)((chopconf >> 4) & 0x07));
      log_info("  hend: %u", (unsigned int)((chopconf >> 7) & 0x0F));
      log_info("  chm: %u", (unsigned int)((chopconf >> 14) & 0x01));
      log_info("  mres: %u", (unsigned int)((chopconf >> 24) & 0x0F));
      log_info("  intpol: %u", (unsigned int)((chopconf >> 28) & 0x01));
      log_info("  dedge: %u", (unsigned int)((chopconf >> 29) & 0x01));
      log_info("  diss2g: %u", (unsigned int)((chopconf >> 30) & 0x01));
    }

    // COOLCONF Register
    uint32_t coolconf = 0;
    if (readRegister(RegIdx::COOLCONF, coolconf)) {
      log_info("COOLCONF (0x6D): 0x%08X", (unsigned int)coolconf);
      log_info("  semin: %u", (unsigned int)(coolconf & 0x0F));
      log_info("  seup: %u", (unsigned int)((coolconf >> 5) & 0x03));
      log_info("  semax: %u", (unsigned int)((coolconf >> 8) & 0x0F));
      log_info("  sedn: %u", (unsigned int)((coolconf >> 13) & 0x03));
      log_info("  seimin: %u", (unsigned int)((coolconf >> 15) & 0x01));
    }



    // DRV_STATUS Register
    uint32_t drvStatus = 0;
    if (readRegister(RegIdx::DRV_STATUS, drvStatus)) {
      log_info("DRV_STATUS (0x6F): 0x%08X", (unsigned int)drvStatus);
      log_info("  otpw: %d", (drvStatus >> 0) & 0x01);
      log_info("  ot: %d", (drvStatus >> 1) & 0x01);
      log_info("  s2ga: %d", (drvStatus >> 2) & 0x01);
      log_info("  s2gb: %d", (drvStatus >> 3) & 0x01);
      log_info("  ola: %d", (drvStatus >> 4) & 0x01);
      log_info("  olb: %d", (drvStatus >> 5) & 0x01);
      log_info("  t120: %d", (drvStatus >> 6) & 0x01);
      log_info("  t143: %d", (drvStatus >> 7) & 0x01);
      log_info("  t150: %d", (drvStatus >> 8) & 0x01);
      log_info("  t157: %d", (drvStatus >> 9) & 0x01);
      log_info("  cs_actual: %u", (unsigned int)((drvStatus >> 16) & 0x1F));
      log_info("  stealth: %d", (drvStatus >> 30) & 0x01);
      log_info("  standstill: %d", (drvStatus >> 31) & 0x01);
    }

    log_info("================================================");
  
}

bool TMC2209::writeRegister(RegIdx reg, uint32_t value, uint32_t timeoutMs) {
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

  HAL_StatusTypeDef res = HAL_UART_Transmit(
      huart_, const_cast<uint8_t *>(frame), sizeof(frame), timeoutMs);
  if (res != HAL_OK) {
    log_warn("TMC2209: Write Reg 0x%02X - TX failed with %d", (unsigned int)reg,
             (int)res);
    return false;
  }
  log_debug("TMC2209: Write Reg 0x%02X - TX OK", (unsigned int)reg);
  return true;
}

bool TMC2209::readRegister(RegIdx reg, uint32_t &value, uint32_t timeoutMs, bool checkStartByte) {
  // Clear any pending RX data to avoid stale bytes from previous transactions
  HAL_UART_AbortReceive(huart_);
#ifdef __HAL_UART_FLUSH_DRREGISTER
  __HAL_UART_FLUSH_DRREGISTER(huart_);
#endif
  __HAL_UART_CLEAR_OREFLAG(huart_);

  // Read request: 0x05, addr, reg, crc
  uint8_t req[4];
  req[0] = 0x05;
  req[1] = static_cast<uint8_t>(addr_); // read (bit 7 = 0)
  req[2] = static_cast<uint8_t>(reg);
  req[3] = crc8(req, 4);

  // Response: 0x05, addr, reg, data[3], data[2], data[1], data[0], crc
  // Note: Single-wire UART echoes TX bytes, so we receive: 4 echo bytes + 8
  // response bytes
  uint8_t resp[12];
  memset(resp, 0, sizeof(resp));

  // Start RX first to be ready when response arrives, then TX request
  HAL_StatusTypeDef res = HAL_UART_Receive_IT(huart_, resp, sizeof(resp));
  if (res != HAL_OK) {
    log_warn("TMC2209: Read Reg 0x%02X - RX IT failed with %d",
             (unsigned int)reg, (int)res);
    return false;
  }

  res = HAL_UART_Transmit_IT(huart_, const_cast<uint8_t *>(req), sizeof(req));
  if (res != HAL_OK) {
    log_warn("TMC2209: Read Reg 0x%02X - TX IT failed with %d",
             (unsigned int)reg, (int)res);
    return false;
  }

  // Wait for both to complete with timeout
  uint32_t tickStart = HAL_GetTick();
  while ((huart_->gState != HAL_UART_STATE_READY) ||
         (huart_->RxState != HAL_UART_STATE_READY)) {
    if ((HAL_GetTick() - tickStart) > timeoutMs) {
      HAL_UART_AbortReceive_IT(huart_);
      log_warn("TMC2209: Read Reg 0x%02X - TX/RX timeout", (unsigned int)reg);
      return false;
    }
  }

  // Skip first 4 bytes (echo of TX), actual response starts at resp[4]
  uint8_t *pResp = &resp[4];

  // Basic validation
  if (checkStartByte && pResp[0] != 0x05) {
    log_warn("TMC2209: Read Reg 0x%02X - Invalid start byte (expected 0x05). Received: 0x%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned int)reg, (unsigned int)pResp[0], (unsigned int)pResp[1],
             (unsigned int)pResp[2], (unsigned int)pResp[3], (unsigned int)pResp[4],
             (unsigned int)pResp[5], (unsigned int)pResp[6], (unsigned int)pResp[7]);
    return false;
  }

  // TMC2209 replies use 0xFF as address in UART single-wire mode; accept it unconditionally
  if (pResp[1] != 0xFF) {
    log_warn("TMC2209: Read Reg 0x%02X - Address mismatch (expected broadcast 0xFF). Received: 0x%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned int)reg, (unsigned int)pResp[0], (unsigned int)pResp[1],
             (unsigned int)pResp[2], (unsigned int)pResp[3], (unsigned int)pResp[4],
             (unsigned int)pResp[5], (unsigned int)pResp[6], (unsigned int)pResp[7]);
    return false;
  }
  if (pResp[2] != static_cast<uint8_t>(reg)) {
    log_warn("TMC2209: Read Reg 0x%02X - Register mismatch. Received: 0x%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned int)reg, (unsigned int)pResp[0], (unsigned int)pResp[1],
             (unsigned int)pResp[2], (unsigned int)pResp[3], (unsigned int)pResp[4],
             (unsigned int)pResp[5], (unsigned int)pResp[6], (unsigned int)pResp[7]);
    return false;
  }
  const uint8_t rx_crc = pResp[7];
  const uint8_t expected_crc = crc8(pResp, 8);
  if (rx_crc != expected_crc) {
    log_warn("TMC2209: Read Reg 0x%02X - CRC mismatch (expected 0x%02X). Received: 0x%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned int)reg, (unsigned int)expected_crc, (unsigned int)pResp[0], (unsigned int)pResp[1],
             (unsigned int)pResp[2], (unsigned int)pResp[3], (unsigned int)pResp[4], (unsigned int)pResp[5],
             (unsigned int)pResp[6], (unsigned int)pResp[7]);
    return false;
  }

  value = (static_cast<uint32_t>(pResp[3]) << 24) |
          (static_cast<uint32_t>(pResp[4]) << 16) |
          (static_cast<uint32_t>(pResp[5]) << 8) |
          (static_cast<uint32_t>(pResp[6]));
  log_debug("TMC2209: Read Reg 0x%02X = 0x%08X", (unsigned int)reg,
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
  return writeRegister(RegIdx::IHOLD_IRUN, v);
}

constexpr float t = (1 << 24) / 12000000.0f;

union U2I32 {
  uint32_t u32;
  int32_t i32;
};

bool TMC2209::generateSteps(int fullStepsPerSecond) {
  // Generate motor steps via register-based pulse generation
  // without requiring external STEP pulse input
  U2I32 vactualUnion;
  vactualUnion.i32 =
      fullStepsPerSecond * t *
      256; // VACTUAL = velocity in steps/s * 256 * (2^24 / f_CLK)

  log_info("Writing VACTUAL register with value %d", vactualUnion.i32);
  if (!writeRegister(RegIdx::VACTUAL, vactualUnion.u32)) {
    log_error("TMC2209: Failed to write VACTUAL register");
    return false;
  }
  return true;
}

} // namespace tmc2209
