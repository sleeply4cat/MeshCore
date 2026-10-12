#pragma once

#include <RadioLib.h>
#include "SX126xSpectrumPlugin.h"

// Frequency sweeps on an SX1262, run by the radio itself. A plugin on the radio's core (SX126xSpectrumPlugin.h)
// retunes the receiver after every RSSI update and keeps the reading that belongs to each point, so a whole
// sweep costs the host one command and one buffer read.
//
// The RSSI register updates every ~1.6/RBW ms (RBW in kHz) and a point takes about one update: 250 points take
// ~56ms at an RBW of 7.3kHz, ~21ms at 19.5kHz, ~5ms at 78kHz.
// While begin() is in effect the radio is in GFSK RX and can't do LoRa. A reset of the chip removes the plugin.
class SX126xSpectrumScanner {
  enum {
    CMD_SWEEP = 0x9B,           // the plugin's command
    REG_RESULT = 0x0400,        // the data buffer, where a sweep leaves one level per point
    REG_VERSION = 0x0320,       // the ROM version string, 16 chars
    REG_VERSION_TAG = 0x032E,   // its last char, which the plugin sets once loaded
    // in GFSK RX, noise that passes for a packet is written to the data buffer too: a fixed packet length of
    // one byte, at the very end of the buffer, keeps it away from the results
    RX_BASE = 0xFF,
  };

  SX1262* _radio;
  uint8_t _params[11];   // the sweep command's parameters, see setSweep()
  uint8_t _points;
  bool _active;

  // RSSI updates to skip after a retune: the RSSI shows a new frequency only ~3.4 updates later at a narrow
  // RBW, ~4.2 at 150kHz and wider
  static uint8_t latencyFor(float rbw_khz) {
    return rbw_khz < 17 ? 2 : (rbw_khz < 150 ? 3 : 4);
  }

public:
  enum { MAX_POINTS = 250 };   // the plugin counts points + skipped updates in a byte

  // the narrowest GFSK RX bandwidth at least 'khz' wide (kHz; 467 at most)
  static float pickRxBw(float khz) {
    static const float bws[] = { 4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0, 46.9, 58.6,
                                 78.2, 93.8, 117.3, 156.2, 187.2, 234.3, 312.0, 373.6, 467.0 };
    for (size_t i = 0; i < sizeof(bws) / sizeof(bws[0]); i++) {
      if (bws[i] >= khz) return bws[i];
    }
    return 467.0;
  }

  explicit SX126xSpectrumScanner(SX1262* radio) : _radio(radio), _points(0), _active(false) { }

  bool isActive() const { return _active; }

  // Resets the radio into GFSK RX and loads the plugin: RADIOLIB_ERR_UNSUPPORTED if the chip has another ROM.
  int16_t begin(float freq_mhz, float tcxo_voltage, bool use_ldo) {
    int16_t err = _radio->beginFSK(freq_mhz, 4.8, 5.0, 234.3, 0, 16, tcxo_voltage, use_ldo);
    if (err == RADIOLIB_ERR_SPI_CMD_FAILED || err == RADIOLIB_ERR_SPI_CMD_INVALID) {
      err = _radio->beginFSK(freq_mhz, 4.8, 5.0, 234.3, 0, 16, 0.0f, use_ldo);   // no TCXO, as CustomSX1262 retries
    }
    if (err != RADIOLIB_ERR_NONE) return err;

    char ver[16];
    err = _radio->readRegister(REG_VERSION, (uint8_t*) ver, sizeof(ver));
    if (err != RADIOLIB_ERR_NONE) return err;
    if (memcmp(ver, "SX1261 V2D 2D02", 15) != 0) return RADIOLIB_ERR_UNSUPPORTED;

    err = _radio->uploadPatch(SX126X_SPECTRUM_PLUGIN, sizeof(SX126X_SPECTRUM_PLUGIN));
    if (err != RADIOLIB_ERR_NONE) return err;
    if (_radio->mod->SPIreadRegister(REG_VERSION_TAG) != SX126X_SPECTRUM_PLUGIN_TAG) return RADIOLIB_ERR_CHIP_NOT_FOUND;

    err = _radio->fixedPacketLengthMode(1);
    if (err == RADIOLIB_ERR_NONE) err = _radio->setBufferBaseAddress(0, RX_BASE);
    if (err != RADIOLIB_ERR_NONE) return err;

  #ifdef SX126X_DIO2_AS_RF_SWITCH
    _radio->setDio2AsRfSwitch(SX126X_DIO2_AS_RF_SWITCH);
  #endif
    _radio->setRxBoostedGainMode(true);
    _active = true;
    return RADIOLIB_ERR_NONE;
  }

  // Sets up sweeps of 'points' (1..MAX_POINTS) points, from start_mhz in steps of step_mhz, measured through
  // the narrowest RX bandwidth of at least rbw_khz.
  int16_t setSweep(float start_mhz, float step_mhz, uint8_t points, float rbw_khz) {
    if (points == 0 || points > MAX_POINTS) return RADIOLIB_ERR_INVALID_NUM_SAMPLES;
    float rbw = pickRxBw(rbw_khz);
    _radio->standby();
    int16_t err = _radio->setRxBandwidth(rbw);
    if (err != RADIOLIB_ERR_NONE) return err;
    err = _radio->setFrequency(start_mhz + step_mhz * points / 2);   // calibrates the image rejection for the band
    if (err != RADIOLIB_ERR_NONE) return err;
    // standby() set an external RF switch (RXEN/TXEN pins) to idle, and setRx() doesn't set it back (unlike
    // startReceive())
    _radio->mod->setRfSwitchState(Module::MODE_RX);
    err = _radio->setRx(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    if (err != RADIOLIB_ERR_NONE) return err;

    // The plugin polls the RSSI register for each update: ~8 polls per update period, a poll taking ~0.6us
    // plus 'delay' x 0.125us; after 'wait' polls (~3 periods) without a change it takes the reading as it is
    // (a steady signal reads the same).
    float update_us = 1600.0f / rbw;
    float delay = (update_us / 8 - 0.6f) / 0.125f;
    uint8_t d = delay < 0 ? 0 : (delay > 255 ? 255 : (uint8_t)delay);
    float wait = 3 * update_us / (0.6f + 0.125f * d);
    uint8_t w = wait < 4 ? 4 : (wait > 255 ? 255 : (uint8_t)wait);

    // frequencies in the radio's units: 32MHz / 2^25 per unit, ie. 2^20 per MHz
    uint32_t f = (uint32_t)(start_mhz * 1048576.0);
    uint32_t s = (uint32_t)(step_mhz * 1048576.0 + 0.5);
    uint8_t p[] = {
      (uint8_t)(f >> 24), (uint8_t)(f >> 16), (uint8_t)(f >> 8), (uint8_t)f,   // first point
      (uint8_t)(s >> 16), (uint8_t)(s >> 8), (uint8_t)s,                       // step
      points, latencyFor(rbw), d, w,
    };
    memcpy(_params, p, sizeof(_params));
    _points = points;
    return RADIOLIB_ERR_NONE;
  }

  // Runs one sweep (blocking for its duration): 'level' gets a level per point, -2 * dBm (as GetRssiInst).
  int16_t sweep(uint8_t* level) {
    if (!_active || _points == 0) return RADIOLIB_ERR_WRONG_MODEM;
    // the plugin is gone if the chip was reset (eg. a brownout): its command would then run whatever is in RAM
    if (_radio->mod->SPIreadRegister(REG_VERSION_TAG) != SX126X_SPECTRUM_PLUGIN_TAG) return RADIOLIB_ERR_CHIP_NOT_FOUND;
    int16_t err = _radio->mod->SPIwriteStream(CMD_SWEEP, _params, sizeof(_params));   // returns after the sweep
    if (err == RADIOLIB_ERR_NONE) err = _radio->mod->SPIcheckStream();   // the plugin's status: refused parameters
    if (err == RADIOLIB_ERR_NONE) err = _radio->readRegister(REG_RESULT, level, _points);
    return err;
  }

  void end() {
    if (_active) _radio->standby();
    _active = false;
  }
};
