#include "TMC2209.hpp"
#include "common.hh"
#include "gpio.hh"
#include "log.h"
#include "tmc2209_reg.hpp"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/types.h>


namespace tmc2209 {

  
  bool TMC2209::checkedWriteRegister(RegIdx reg, uint32_t value, const char* error_msg){                                   
    if (!writeRegister(reg, value)) {       
      log_error(error_msg);                                        
      return false;                                                            
    }
    return true;                                                                          
  } 

  bool TMC2209::checkedReadRegister(RegIdx reg, uint32_t& value, const char* error_msg){                                   
    if (!readRegister(reg, value)) {                                          
      log_error(error_msg);                                        
      return false;                                                           
    }       
    return true;                                                                   
  }

  bool TMC2209::clearGStat(const char* error_msg) {
    constexpr uint32_t clear_all_gstat_flags = 0x00000007;
    return checkedWriteRegister(RegIdx::GSTAT, clear_all_gstat_flags, error_msg);
  }



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
  checkedReadRegister(RegIdx::IOIN, ioin.U32,
                 "Failed to read IOIN register during init");
  if (ioin.REG.version != 0x21) {
    log_error("TMC2209: Unexpected chip version 0x%02X",
              (unsigned int)ioin.REG.version);
    return false;
  }

  uint32_t val = 0;
  checkedReadRegister(RegIdx::GCONF, val,
                 "Failed to read GCONF register for local mirroring");
  gconf.U32 = val;
  checkedReadRegister(RegIdx::CHOPCONF, val,
                 "Failed to read CHOPCONF register for local mirroring");
  chopconf.U32 = val;
  return true;
}

bool TMC2209::ShaftTurnReversed(bool reversed) {

  gconf.REG.shaft = reversed ? 1 : 0;
  checkedWriteRegister(RegIdx::GCONF, gconf.U32, "Failed to write GCONF register");
  return true;
}
/**
 * Initialize the TMC2209 for normal speed (=StealthChop) and UART-based operation
 * @param usePotimeterForCurrentScaling Whether to use the potentiometer for current scaling
 * @param disable_read Whether to disable reading of registers
 * @param resolution Microstep resolution
 * @return true if initialization was successful, false otherwise
 */
bool TMC2209::InitForNormalSpeedAndUartBasedOperation(
    bool usePotimeterForCurrentScaling, bool disable_read, MicroStepResolution resolution) {
  // Optionally, perform any initialization here
  enable();
  HAL_Delay(10);
  if(!disable_read) {
    if (!fetchImportantRegistersForLocalMirroring()) {
      return false;
    }
  }

  if (!clearGStat("Failed to clear GSTAT before init")) {
    return false;
  }

  gconf.REG.i_scale_analog = usePotimeterForCurrentScaling ? 0b1 : 0b0;
  gconf.REG.internal_rsense =
      0; // Current sense resistors. 0: Exteral; 1: internal
    gconf.REG.enable_spread_cycle =
      0; // 0: StealthChop; 1: SpreadCycle
  gconf.REG.shaft = 0;
  gconf.REG.pdn_disable = 1; // Disable PDN_UART input function
  gconf.REG.mstep_reg_select =
      1;                        // Microstepping. 0: MS1, MS2 pins; 1: register
  gconf.REG.multistep_filt = 1; // Step pulse filter. 0: disable; 1: enable
  gconf.REG.test_mode = 0;        // 0: normal operation, 1: Enable analog test output on pin ENN
  if (!checkedWriteRegister(RegIdx::GCONF, gconf.U32,
                  "Failed to write GCONF register during init")) {
    return false;
  }

  if (!disable_read) {
    uint32_t gconf_readback = 0;
    if (!checkedReadRegister(RegIdx::GCONF, gconf_readback,
                    "Failed to verify GCONF register during init")) {
      return false;
    }
    constexpr uint32_t gconf_mask = 0x000003FF;
    if ((gconf_readback & gconf_mask) != (gconf.U32 & gconf_mask)) {
      log_error("TMC2209: GCONF verification failed. Wrote 0x%08X, read back 0x%08X",
                (unsigned int)gconf.U32, (unsigned int)gconf_readback);
      return false;
    }
  }
if(resolution != MicroStepResolution::RES256){
  chopconf.REG.mres = static_cast<uint32_t>(resolution);
  if (!checkedWriteRegister(RegIdx::CHOPCONF, chopconf.U32,
                  "Failed to write CHOPCONF register during init")) {
    return false;
  }

}


  if (!clearGStat("Failed to clear GSTAT after init")) {
    return false;
  }

  return true;
}

bool TMC2209::PerformStealthChopAutoTuningForQuietOperation() {
  log_info("Starting StealthChop auto-tuning for quiet operation...");
  // 2. Stromparameter für no-load/low-load Betrieb:
  // weniger Strom reduziert Vibrationen und akustisches Rauschen deutlich.
  log_info("  IHOLD_IRUN: IHOLD=4, IRUN=12 (no-load quiet mode), IHOLDDELAY=6");
  REG_FIELD::IHOLD_IRUN ihold_irun_reg = {
      .REG = {.ihold = 4, .irun = 12, .iholddelay = 6}};
  checkedWriteRegister(RegIdx::IHOLD_IRUN, ihold_irun_reg.U32,
                  "Failed to write IHOLD_IRUN register for StealthChop");

  // TPWMTHRS: Übergang zwischen StealthChop und SpreadCycle.
  // 0 = deaktiviert → StealthChop bei ALLEN Geschwindigkeiten erzwungen (leisester Betrieb).
  // Beispielwert für Grenze bei ~500 Hz: f_CLK / (f_thrs * 256) = 12MHz / (500 * 256) ≈ 94
  checkedWriteRegister(RegIdx::TPWMTHRS, 0,
                  "Failed to write TPWMTHRS register for StealthChop");

  // 4. Initiale PWM Konfiguration
  log_info(
      "  Initial PWMCONF: pwm_offset=36, pwm_grad=2, pwm_freq=35kHz, pwm_autoscale=1, pwm_autograd=1, freewheel=0 (normal), pwm_reg=8, pwm_lim=12");
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
  checkedWriteRegister(RegIdx::PWMCONF, pwmconf.U32,
                  "Failed to write PWMCONF register for StealthChop");

  // 3. Motor mit konstanter Geschwindigkeit bewegen für Tuning
  // Datasheet empfiehlt:  "A typical range is 60-300 RPM"
  log_info("  Turn Motor with 400 Steps per Second.");
  GenerateSteps(400); // 400 steps/s = 2 RPS = 120 RPM for 200 steps/rev motor
  REG_FIELD::PWM_AUTO pwm_auto;
  for (int i = 0; i < 10; i++) {
    checkedReadRegister(RegIdx::PWMCONF, pwmconf.U32,
                   "Failed to read PWMCONF register for tuning results");
    checkedReadRegister(RegIdx::PWM_AUTO, pwm_auto.U32,
                   "Failed to read PWM_AUTO register for tuning results");
    log_info("  Tuning Step %d: PWM_GRAD=%d, PWM_OFFSET=%d, "
             "PWM_OFFSET_AUTO=0x%03X, PWM_GRADIENT_AUTO=0x%03X",
             i + 1, pwmconf.REG.pwm_grad, pwmconf.REG.pwm_offset,
             (unsigned int)pwm_auto.REG.pwm_ofs_auto,
             (unsigned int)pwm_auto.REG.pwm_grad_auto);
    HAL_Delay(1000);
  }
  // Motor anhalten
  GenerateSteps(0); // Stop motor

  // Write back converged auto-tuned values into PWMCONF so future power-ups
  // start from the calibrated operating point instead of the initial defaults.
  pwmconf.REG.pwm_offset = pwm_auto.REG.pwm_ofs_auto;
  pwmconf.REG.pwm_grad   = pwm_auto.REG.pwm_grad_auto;
  checkedWriteRegister(RegIdx::PWMCONF, pwmconf.U32,
                  "Failed to write back auto-tuned values to PWMCONF");
  log_info("  Written back tuned values to PWMCONF: pwm_offset=%d, pwm_grad=%d",
           (unsigned int)pwmconf.REG.pwm_offset,
           (unsigned int)pwmconf.REG.pwm_grad);

  log_info("Automatic Tuning completed.");
  return true;
}

bool TMC2209::EnableCoolStep(uint32_t tcoolthrs, uint8_t sgthrs,
                             uint8_t semin, uint8_t seup,
                             uint8_t semax, uint8_t sedn,
                             bool seimin) {
  if (semin > 0x0F || seup > 0x03 || semax > 0x0F || sedn > 0x03) {
    log_error("EnableCoolStep: Invalid COOLCONF field value");
    return false;
  }

  // CoolStep uses StallGuard and therefore needs SpreadCycle enabled.
  gconf.REG.enable_spread_cycle = 1;
  if (!checkedWriteRegister(RegIdx::GCONF, gconf.U32,
                            "Failed to enable SpreadCycle for CoolStep")) {
    return false;
  }

  if (!checkedWriteRegister(RegIdx::TCOOLTHRS, tcoolthrs,
                            "Failed to write TCOOLTHRS for CoolStep")) {
    return false;
  }

  if (!checkedWriteRegister(RegIdx::SGTHRS, sgthrs,
                            "Failed to write SGTHRS for CoolStep")) {
    return false;
  }

  REG_FIELD::COOLCONF coolconf = {
      .REG = {.semin = semin,
              .reserved1 = 0,
              .seup = seup,
              .reserved2 = 0,
              .semax = semax,
              .reserved3 = 0,
              .sedn = sedn,
              .seimin = seimin ? 1U : 0U,
              .reserved4 = 0}};

  if (!checkedWriteRegister(RegIdx::COOLCONF, coolconf.U32,
                            "Failed to write COOLCONF for CoolStep")) {
    return false;
  }

  last_tcoolthrs_ = tcoolthrs;
  last_sgthrs_ = sgthrs;
  last_coolconf_ = coolconf.U32;
  coolstep_configured_ = true;

  if (semin == 0) {
    log_error("EnableCoolStep: semin is 0, CoolStep stays disabled");
  }

  log_info("CoolStep configured: TCOOLTHRS=%lu SGTHRS=%u COOLCONF=0x%08lX",
           (unsigned long)tcoolthrs, (unsigned int)sgthrs,
           (unsigned long)coolconf.U32);
  return true;
}

bool TMC2209::LogCoolStepRuntimeStatus() {
  if (!coolstep_configured_) {
    return false;
  }

  uint32_t sg_result = 0;
  REG_FIELD::DRV_STATUS drv_status = {};
  uint32_t tstep = 0;

  if (!checkedReadRegister(RegIdx::SG_RESULT, sg_result,
                           "Failed to read SG_RESULT for CoolStep diagnostics")) {
    return false;
  }
  if (!checkedReadRegister(RegIdx::DRV_STATUS, drv_status.U32,
                           "Failed to read DRV_STATUS for CoolStep diagnostics")) {
    return false;
  }
  if (!checkedReadRegister(RegIdx::TSTEP, tstep,
                           "Failed to read TSTEP for CoolStep diagnostics")) {
    return false;
  }

  log_info(
      "CoolStep runtime: SG_RESULT=%lu CS_ACTUAL=%u TSTEP=%lu SGTHRS=%u "
      "TCOOLTHRS=%lu COOLCONF=0x%08lX",
      (unsigned long)(sg_result & 0x3FF),
      (unsigned int)drv_status.REG.current_scaling,
      (unsigned long)tstep,
      (unsigned int)last_sgthrs_,
      (unsigned long)last_tcoolthrs_,
      (unsigned long)last_coolconf_);
  return true;
}

void TMC2209::PrintPrettyFullSystemState() {
    log_info("================================================");
    log_info("TMC2209: Full System State:");
    log_info("================================================");
    // GCONF Register
    if (readRegister(RegIdx::GCONF, this->gconf.U32)) {
      log_info("GCONF (0x00): 0x%08X", (unsigned int)gconf.U32);
      log_info("  i_scale_analog: %d", gconf.REG.i_scale_analog);
      log_info("  internal_rsense: %d", gconf.REG.internal_rsense);
      log_info("  shaft: %d", gconf.REG.shaft);
      log_info("  index_otpw: %d", gconf.REG.index_otpw);
      log_info("  index_step: %d", gconf.REG.index_step);
      log_info("  pdn_disable: %d", gconf.REG.pdn_disable);
      log_info("  mstep_reg_select: %d", gconf.REG.mstep_reg_select);
      log_info("  multistep_filt: %d", gconf.REG.multistep_filt);
    }
    

    // GSTAT Register
    REG_FIELD::GSTAT gstat;
    if (readRegister(RegIdx::GSTAT, gstat.U32)) {
      log_info("GSTAT (0x01): 0x%08X", (unsigned int)gstat.U32);
      log_info("  reset: %d", gstat.REG.reset);
      log_info("  drv_err: %d", gstat.REG.drv_err);
      log_info("  uv_cp: %d", gstat.REG.uv_cp);
    }

    // IFCOUNT Register
    uint32_t ifcount = 0;
    if (readRegister(RegIdx::IFCNT, ifcount)) {
      log_info("IFCOUNT (0x02): 0x%08X (UART Interface Requests: %u)",
               (unsigned int)ifcount, (unsigned int)(ifcount & 0xFF));
    }

    // CHOPCONF Register
    REG_FIELD::CHOPCONF chopconf;
    if (readRegister(RegIdx::CHOPCONF, chopconf.U32)) {
      log_info("CHOPCONF (0x6C): 0x%08X", (unsigned int)chopconf.U32);
      log_info("  toff: %u", (unsigned int)(chopconf.REG.toff));
      log_info("  hstrt: %u", (unsigned int)(chopconf.REG.hstrt));
      log_info("  hend: %u", (unsigned int)(chopconf.REG.hend));
      log_info("  tbl: %u", (unsigned int)(chopconf.REG.tbl));
      log_info("  vsense: %u", (unsigned int)(chopconf.REG.vsense));
      log_info("  mres: %u", (unsigned int)(chopconf.REG.mres));
      log_info("  interpolation: %u", (unsigned int)(chopconf.REG.interpolation));
      log_info("  double_edge: %u", (unsigned int)(chopconf.REG.double_edge));
      log_info("  diss2g: %u", (unsigned int)(chopconf.REG.diss2g));
      log_info("  diss2vs: %u", (unsigned int)(chopconf.REG.diss2vs));
    }

    // COOLCONF readback is not reliable on TMC2209 in this UART mode.
    if (coolstep_configured_) {
      REG_FIELD::COOLCONF coolconf = {.U32 = last_coolconf_};
      log_info("COOLCONF (0x42): write-only in practice, cached value 0x%08X",
               (unsigned int)coolconf.U32);
      log_info("  semin: %u", (unsigned int)(coolconf.REG.semin));
      log_info("  seup: %u", (unsigned int)(coolconf.REG.seup));
      log_info("  semax: %u", (unsigned int)(coolconf.REG.semax));
      log_info("  sedn: %u", (unsigned int)(coolconf.REG.sedn));
      log_info("  seimin: %u", (unsigned int)(coolconf.REG.seimin));
      log_info("  sgthrs (cached): %u", (unsigned int)last_sgthrs_);
      log_info("  tcoolthrs (cached): %u", (unsigned int)last_tcoolthrs_);
    }



    // DRV_STATUS Register
    REG_FIELD::DRV_STATUS drvStatus;
    if (readRegister(RegIdx::DRV_STATUS, drvStatus.U32)) {
      log_info("DRV_STATUS (0x6F): 0x%08X", (unsigned int)drvStatus.U32);
      log_info("  otpw: %d", drvStatus.REG.over_temperature_prewarning);
      log_info("  ot: %d", (drvStatus.REG.over_temperature));
      log_info("  short_to_ground_a: %d", (drvStatus.REG.short_to_ground_a));
      log_info("  short_to_ground_b: %d", (drvStatus.REG.short_to_ground_b));
      log_info("  open_load_a: %d", (drvStatus.REG.open_load_a));
      log_info("  open_load_b: %d", (drvStatus.REG.open_load_b));
      log_info("  over_temperature_120c: %d", (drvStatus.REG.over_temperature_120c));
      log_info("  over_temperature_143c: %d", (drvStatus.REG.over_temperature_143c));
      log_info("  over_temperature_150c: %d", (drvStatus.REG.over_temperature_150c));
      log_info("  over_temperature_157c: %d", (drvStatus.REG.over_temperature_157c));
      log_info("  current_scaling: %u", (unsigned int)(drvStatus.REG.current_scaling));
      log_info("  stealth_chop_mode: %d", (drvStatus.REG.stealth_chop_mode));
      log_info("  standstill: %d", (drvStatus.REG.standstill));
    }
    REG_FIELD::IOIN ioin;
    if(readRegister(RegIdx::IOIN, ioin.U32)){
      log_info("IOIN (0x06): 0x%08X", (unsigned int)ioin.U32);
      log_info("  version (should be 0x21): 0x%02X", ioin.REG.version);
      log_info("  enn (0 means enabled): %u", (unsigned int)ioin.REG.enn);
      log_info("  ms1: %u", (unsigned int)ioin.REG.ms1);
      log_info("  ms2: %u", (unsigned int)ioin.REG.ms2);
      log_info("  diag (1 means error condition): %d", ioin.REG.diag);
      log_info("  pdn_uart: %d", ioin.REG.pdn_uart);
      log_info("  step: %u", (unsigned int)ioin.REG.step);
      log_info("  spread_en: %u", (unsigned int)ioin.REG.spread_en);
      log_info("  dir: %u", (unsigned int)ioin.REG.dir);

    }

    //TSTEP and TPWMTHRS
    uint32_t tstep = 0;
    if (readRegister(RegIdx::TSTEP, tstep)) {
      log_info("TSTEP (0x12): %u (Actual measured time between two 1/256 microsteps)",
               (unsigned int)tstep);
    }
    // TPWMTHRS (0x13) is write-only; reading always returns 0, skipping readback.

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
  const uint32_t value = (static_cast<uint32_t>(iholddelay) << 16) |
                         (static_cast<uint32_t>(irun) << 8) |
                         (static_cast<uint32_t>(ihold));
  return writeRegister(RegIdx::IHOLD_IRUN, value);
}

constexpr float t = (1 << 24) / 12000000.0f;

union U2I32 {
  uint32_t u32;
  int32_t i32;
};

bool TMC2209::GenerateSteps(int fullStepsPerSecond) {
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

  // Read back key runtime registers to verify that velocity mode is actually active.
  uint32_t vactual_readback = 0;
  if (readRegister(RegIdx::VACTUAL, vactual_readback, 200, false)) {
    log_info("TMC2209: VACTUAL readback = 0x%08X (%ld)",
             (unsigned int)vactual_readback,
             (long)static_cast<int32_t>(vactual_readback));
  }

  REG_FIELD::DRV_STATUS drv_status{};
  if (readRegister(RegIdx::DRV_STATUS, drv_status.U32, 200, false)) {
    log_info("TMC2209: DRV_STATUS after VACTUAL = 0x%08X (standstill=%u, cs=%u)",
             (unsigned int)drv_status.U32,
             (unsigned int)drv_status.REG.standstill,
             (unsigned int)drv_status.REG.current_scaling);
  }

  uint32_t tstep = 0;
  if (readRegister(RegIdx::TSTEP, tstep, 200, false)) {
    log_info("TMC2209: TSTEP after VACTUAL = %u", (unsigned int)tstep);
  }

  uint32_t mscnt = 0;
  if (readRegister(RegIdx::MSCNT, mscnt, 200, false)) {
    log_info("TMC2209: MSCNT after VACTUAL = %u", (unsigned int)(mscnt & 0x3FF));

    HAL_Delay(50);
    uint32_t mscnt_later = 0;
    if (readRegister(RegIdx::MSCNT, mscnt_later, 200, false)) {
      const uint16_t m0 = static_cast<uint16_t>(mscnt & 0x3FF);
      const uint16_t m1 = static_cast<uint16_t>(mscnt_later & 0x3FF);
      const uint16_t delta = static_cast<uint16_t>((m1 + 1024U - m0) % 1024U);
      log_info("TMC2209: MSCNT after 50ms = %u (delta=%u)",
               (unsigned int)m1,
               (unsigned int)delta);
    }
  }
  return true;
}

} // namespace tmc2209
