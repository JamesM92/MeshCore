#pragma once

#include <RadioLib.h>

class CustomSX1262 : public SX1262 {
  // Timeout-based stuck-IRQ detection:
  //   _preambleMillis  — max ms for a detected preamble to produce a valid header;
  //                      cleared after this to avoid the 30s AGC-timer lockup.
  //   _maxPayloadMillis — max ms for a valid header to produce RX_DONE;
  //                       cleared after this if the packet never completes.
  // Defaults tuned for SF7 @ 125 kHz (preamble ~66 ms, max payload ~3.9 s).
  // setParams() updates these via setPreambleMillis()/setMaxPayloadMillis().
  uint32_t _preambleMillis = 66;
  uint32_t _maxPayloadMillis = 3934;
  uint32_t _activityAt = 0;
  bool _headerSeen = false;

  public:
    CustomSX1262(Module *mod) : SX1262(mod) { }

  #ifdef RP2040_PLATFORM
    bool std_init(SPIClassRP2040* spi = NULL)
  #else
    bool std_init(SPIClass* spi = NULL)
  #endif
    {
  #ifdef SX126X_DIO3_TCXO_VOLTAGE
      float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
  #else
      float tcxo = 1.6f;
  #endif

  #ifdef LORA_CR
      uint8_t cr = LORA_CR;
  #else
      uint8_t cr = 5;
  #endif

  #if defined(P_LORA_SCLK)
    #ifdef NRF52_PLATFORM
      if (spi) { spi->setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI); spi->begin(); }
    #elif defined(RP2040_PLATFORM)
      if (spi) {
        spi->setMISO(P_LORA_MISO);
        //spi->setCS(P_LORA_NSS); // Setting CS results in freeze
        spi->setSCK(P_LORA_SCLK);
        spi->setMOSI(P_LORA_MOSI);
        spi->begin();
      }
    #else
      if (spi) spi->begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
    #endif
  #endif
      int status = begin(LORA_FREQ, LORA_BW, LORA_SF, cr, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo);
      // if radio init fails with -707/-706, try again with tcxo voltage set to 0.0f
      if (status == RADIOLIB_ERR_SPI_CMD_FAILED || status == RADIOLIB_ERR_SPI_CMD_INVALID) {
        tcxo = 0.0f;
        status = begin(LORA_FREQ, LORA_BW, LORA_SF, cr, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo);
      }
      if (status != RADIOLIB_ERR_NONE) {
        Serial.print("ERROR: radio init failed: ");
        Serial.println(status);
        return false;  // fail
      }

  #ifdef SX126X_DIO3_TCXO_VOLTAGE
      // RadioLib's begin() calls modSetup() → config() → calibrate(0x7F), which includes
      // RC13M calibration.  On TCXO boards (DIO3 drives the TCXO supply via a DAC),
      // RC13M calibration interferes with the DIO3 voltage: the RC oscillator and the
      // TCXO contend for the same reference path, leaving the TCXO in a degraded state
      // with elevated phase noise and progressive frequency drift.
      //
      // Symptom: after the very first begin(), one or two packets may still decode (the
      // SNR headroom at -31 dBm is enormous), but after each subsequent firmware reboot
      // (which re-runs begin() → calibrate(0x7F)), header decoding fails permanently —
      // recv stays at 0 and errors stays at 0 (HEADER_ERR never fires DIO1 so we never
      // count it).  The noise floor still converges to -111 dBm (RSSI sampling is just
      // an ADC read, not frequency-dependent) creating the misleading impression that the
      // radio is healthy.
      //
      // Fix: immediately re-run calibration with the TCXO-safe mask (0x7C = skip RC64K +
      // RC13M) right after begin().  setTCXO() is called first to re-assert the DIO3
      // voltage (begin() may have left DIO3 in an uncertain state after the dirty
      // calibration), mirroring the setTCXO() → calibrate() order used by modSetup().
      setTCXO(SX126X_DIO3_TCXO_VOLTAGE);
      {
        uint8_t calData = RADIOLIB_SX126X_CALIBRATE_ALL &
                          ~(RADIOLIB_SX126X_CALIBRATE_RC64K_ON | RADIOLIB_SX126X_CALIBRATE_RC13M_ON);
        mod->SPIwriteStream(RADIOLIB_SX126X_CMD_CALIBRATE, &calData, 1, true, false);
        mod->hal->delay(5);
        uint32_t start = millis();
        while (mod->hal->digitalRead(mod->getGpio())) {
          if (millis() - start > 50) break;
          mod->hal->yield();
        }
      }
      calibrateImage(LORA_FREQ);  // LORA_FREQ is already in MHz
  #endif

      setCRC(1);

  #ifdef SX126X_CURRENT_LIMIT
      setCurrentLimit(SX126X_CURRENT_LIMIT);
  #endif
  #ifdef SX126X_DIO2_AS_RF_SWITCH
      setDio2AsRfSwitch(SX126X_DIO2_AS_RF_SWITCH);
  #endif
  #ifdef SX126X_RX_BOOSTED_GAIN
      setRxBoostedGainMode(SX126X_RX_BOOSTED_GAIN);
  #endif
  #if defined(SX126X_RXEN) || defined(SX126X_TXEN)
    #ifndef SX126X_RXEN
      #define SX126X_RXEN RADIOLIB_NC
    #endif
    #ifndef SX126X_TXEN
      #define SX126X_TXEN RADIOLIB_NC
    #endif
      setRfSwitchPins(SX126X_RXEN, SX126X_TXEN);
  #endif

  #ifdef SX126X_REGISTER_PATCH
    uint8_t r_data = 0;
    readRegister(0x8B5, &r_data, 1);
    r_data |= 0x01;
    writeRegister(0x8B5, &r_data, 1);
  #endif

      return true;  // success
    }

    // Timeout-based isReceiving() — auto-clears stuck HEADER_VALID / HEADER_ERR
    // IRQ flags via timeout so the 30s AGC timer can run even when interference
    // permanently trips the HEADER_VALID flag (the SX126x sticky-IRQ lockup scenario).
    //
    // PREAMBLE_DETECTED is NOT tracked here — it is not enabled in the default
    // irqFlags (RADIOLIB_IRQ_RX_DEFAULT_FLAGS) so it never appears in getIrqFlags().
    // Tracking it would require overriding startReceive(), and doing so on boards with
    // strong nearby transmitters (Heltec V4 at -34 dBm) causes isReceiving() to return
    // true almost continuously (preamble fires every few seconds), blocking RSSI
    // sampling and starving packet reception.  HEADER_VALID alone is sufficient:
    // the header arrives 66 ms after the preamble at SF7/BW125, so a HEADER_VALID
    // timeout of a few seconds catches any genuinely stuck state.
    //
    // _preambleMillis / _maxPayloadMillis: set by setParams() via the wrapper so the
    // timeouts scale correctly with SF/BW/CR.
    bool isReceiving() {
      uint32_t irq = getIrqFlags();
      bool header   = irq & RADIOLIB_SX126X_IRQ_HEADER_VALID;  // bit 4
      bool hdrErr   = irq & RADIOLIB_SX126X_IRQ_HEADER_ERR;    // bit 5
      uint32_t now  = millis();

      if (hdrErr) {
        // Header error — clear all stuck activity flags immediately.
        clearIrqFlags(RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED | RADIOLIB_SX126X_IRQ_HEADER_VALID |
                      RADIOLIB_SX126X_IRQ_HEADER_ERR | RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID);
        _activityAt = 0;
        _headerSeen = false;
        return false;
      }
      if (!header && _headerSeen) {
        // Header flag disappeared without RX_DONE — reset state.
        _activityAt = 0;
        _headerSeen = false;
        return false;
      }
      if (header) {
        if (!_headerSeen) { _headerSeen = true; _activityAt = now; }
        if (now - _activityAt > _maxPayloadMillis) {
          // Header valid for too long without RX_DONE — packet is gone, clear flags.
          clearIrqFlags(RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED | RADIOLIB_SX126X_IRQ_HEADER_VALID |
                        RADIOLIB_SX126X_IRQ_HEADER_ERR | RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID);
          _activityAt = 0;
          _headerSeen = false;
          return false;
        }
        return true;
      }
      _activityAt = 0;
      _headerSeen = false;
      return false;
    }

    // Update timeouts when radio params change (called from setParams via the wrapper).
    void setPreambleMillis(uint32_t ms) { _preambleMillis = ms; }
    void setMaxPayloadMillis(uint32_t ms) { _maxPayloadMillis = ms; }

    bool getRxBoostedGainMode() {
      uint8_t rxGain = 0;
      readRegister(RADIOLIB_SX126X_REG_RX_GAIN, &rxGain, 1);
      return (rxGain == RADIOLIB_SX126X_RX_GAIN_BOOSTED);
    }
};