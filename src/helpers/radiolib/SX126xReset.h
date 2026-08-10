#pragma once

#include <RadioLib.h>

// Full receiver reset for all SX126x-family chips (SX1262, SX1268, LLCC68, STM32WLx).
// Warm sleep powers down analog, Calibrate() refreshes ADC/PLL/image calibration,
// then re-applies RX settings that calibration may reset.
// Used by the 30s AGC timer (resetAGC) where the radio may be in RX with sticky IRQ flags.
//
// TCXO boards (SX126X_DIO3_TCXO_VOLTAGE defined, e.g. Heltec V3 at 1.8 V):
//   1. setTCXO() is re-applied BEFORE Calibrate(), mirroring RadioLib's modSetup() order:
//        setTCXO() → Calibrate().
//      After warm sleep the DIO3 TCXO register may not survive the wakeup transition;
//      calling setTCXO() first ensures the TCXO is configured before calibration runs.
//   2. RC64K + RC13M calibration bits (0x03) are SKIPPED (mask = 0x7C).
//      RC13M is not the active XOSC on a TCXO board; calibrating it while DIO3 is
//      driving the TCXO supply can interfere with oscillator selection and cause
//      progressive RX deafness with every timer-fired reset.
//      RadioLib's begin() uses the same TCXO-safe subset when TCXO is enabled.
inline void sx126xResetAGC(SX126x* radio) {
  radio->sleep(true);
  radio->standby(RADIOLIB_SX126X_STANDBY_RC, true);

#ifdef SX126X_DIO3_TCXO_VOLTAGE
  // Re-apply TCXO control before calibrating — mirrors RadioLib's modSetup() sequence:
  // setTCXO() → Calibrate().  setTCXO() also calls standby() internally so it acts as
  // the busy-wait for the wakeup from sleep above.
  radio->setTCXO(SX126X_DIO3_TCXO_VOLTAGE);

  // Skip RC64K (0x01) + RC13M (0x02) on TCXO boards — same 0x7C mask as RadioLib begin().
  uint8_t calData = RADIOLIB_SX126X_CALIBRATE_ALL &
                    ~(RADIOLIB_SX126X_CALIBRATE_RC64K_ON | RADIOLIB_SX126X_CALIBRATE_RC13M_ON);
#else
  uint8_t calData = RADIOLIB_SX126X_CALIBRATE_ALL;
#endif
  radio->mod->SPIwriteStream(RADIOLIB_SX126X_CMD_CALIBRATE, &calData, 1, true, false);
  radio->mod->hal->delay(5);
  uint32_t start = millis();
  while (radio->mod->hal->digitalRead(radio->mod->getGpio())) {
    if (millis() - start > 50) break;
    radio->mod->hal->yield();
  }

  // Calibrate() defaults image calibration to 902–928 MHz band.
  // Re-calibrate for the actual operating frequency.
  radio->calibrateImage(radio->freqMHz);

#ifdef SX126X_DIO2_AS_RF_SWITCH
  radio->setDio2AsRfSwitch(SX126X_DIO2_AS_RF_SWITCH);
#endif
#ifdef SX126X_RX_BOOSTED_GAIN
  radio->setRxBoostedGainMode(SX126X_RX_BOOSTED_GAIN);
#endif
#ifdef SX126X_REGISTER_PATCH
  uint8_t r_data = 0;
  radio->readRegister(0x8B5, &r_data, 1);
  r_data |= 0x01;
  radio->writeRegister(0x8B5, &r_data, 1);
#endif
}

// Post-TX calibration WITHOUT the sleep step.
// After finishTransmit() the radio is in STANDBY_RC with no sticky IRQ flags
// (radio was transmitting, not receiving) so sleeping to clear IRQs is unnecessary.
// Calling sleep(true) here breaks TCXO boards (e.g. Heltec V3, DIO3 TCXO at 1.8 V):
// the sleep→wake cycle leaves startReceive() unable to properly re-enable RX, causing
// the radio to go deaf until the next firmware reboot.  Direct calibration from
// STANDBY_RC avoids this.
//
// Uses the same TCXO-safe mask as sx126xResetAGC(): skip RC64K + RC13M on TCXO boards.
inline void sx126xPostTxCalibrate(SX126x* radio) {
#ifdef SX126X_DIO3_TCXO_VOLTAGE
  // setTCXO() before Calibrate() mirrors the RadioLib init sequence on TCXO boards.
  radio->setTCXO(SX126X_DIO3_TCXO_VOLTAGE);
  uint8_t calData = RADIOLIB_SX126X_CALIBRATE_ALL &
                    ~(RADIOLIB_SX126X_CALIBRATE_RC64K_ON | RADIOLIB_SX126X_CALIBRATE_RC13M_ON);
#else
  uint8_t calData = RADIOLIB_SX126X_CALIBRATE_ALL;
#endif
  radio->mod->SPIwriteStream(RADIOLIB_SX126X_CMD_CALIBRATE, &calData, 1, true, false);
  radio->mod->hal->delay(5);
  uint32_t start = millis();
  while (radio->mod->hal->digitalRead(radio->mod->getGpio())) {
    if (millis() - start > 50) break;
    radio->mod->hal->yield();
  }
  radio->calibrateImage(radio->freqMHz);
#ifdef SX126X_DIO2_AS_RF_SWITCH
  radio->setDio2AsRfSwitch(SX126X_DIO2_AS_RF_SWITCH);
#endif
#ifdef SX126X_RX_BOOSTED_GAIN
  radio->setRxBoostedGainMode(SX126X_RX_BOOSTED_GAIN);
#endif
#ifdef SX126X_REGISTER_PATCH
  uint8_t r_data2 = 0;
  radio->readRegister(0x8B5, &r_data2, 1);
  r_data2 |= 0x01;
  radio->writeRegister(0x8B5, &r_data2, 1);
#endif
}
