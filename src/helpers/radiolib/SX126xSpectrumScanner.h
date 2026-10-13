#pragma once

#include <RadioLib.h>
#include "SX126xSpectrumPlugin.h"

// SDR-like frequency sweep for SX126x. The whole sweep runs inside the radio, in our own PRAM plugin
// (SX126xSpectrumPlugin.h): the radio's core retunes itself right after every RSSI
// update and files the (latency + 1)-th update after a retune under that point (the receiver chain shows a
// frequency change in the RSSI only ~3.4 updates later). The host sends one command and reads one buffer.
// The plugin's addresses were found on ROM "SX1261 V2D 2D02" (in SX1262s too): begin() refuses other ROMs.
//
// The RSSI register updates every ~1.6/RBW ms (RBW in kHz), and a point costs ~1 update: 250 points take
// ~56ms at RBW 7.3kHz, ~21ms at 19.5kHz, ~5ms at 78kHz.
// The plugin only lives in the radio's RAM: a reset or a sleep removes it. Many boards don't wire the radio's
// NRESET (P_LORA_RESET = RADIOLIB_NC: Heltec V3, Wireless Paper, ...), so RadioLib's begin() can't reset it there;
// begin() and end() reset it with a sleep instead.
// While active, the radio is in GFSK RX and unusable for LoRa traffic.
class SX126xSpectrumScanner {
  enum {
    CMD_SWEEP = 0x9B,
    REG_RESULT = 0x0400,        // RADIO_BUF
    REG_VERSION = 0x0320,       // ROM version string, 16 bytes
    REG_VERSION_TAG = 0x032E,   // last char of the version string, set by the plugin's init
    // In GFSK RX a packet (or noise passing for one) is written to the data buffer, which also holds the
    // results: fixed length 1, at the very end of the buffer, keeps it out of the way
    RX_BASE = 0xFF,
  };

  SX1262* _radio;
  uint8_t _params[11];
  uint16_t _num_points;
  bool _active;
  SPISettings _saved_spi;

  // like a pulse on NRESET, wired or not: a cold sleep keeps nothing, and the chip boots again when woken (the
  // plugin is gone and '2' is back at the end of the version string)
  void resetChip() {
    _radio->standby();
    _radio->sleep(false);
    _radio->standby(RADIOLIB_SX126X_STANDBY_RC, true);   // wakes it, then waits for the boot (BUSY)
  }

public:
  // the narrowest GFSK RX bandwidth at least 'khz' wide (kHz)
  static float pickRxBw(float khz) {
    static const float bws[] = { 4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0, 46.9, 58.6,
                                 78.2, 93.8, 117.3, 156.2, 187.2, 234.3, 312.0, 373.6, 467.0 };
    for (int i = 0; i < sizeof(bws)/sizeof(bws[0]); i++) {
      if (bws[i] >= khz) return bws[i];
    }
    return 467.0;
  }

  enum { MAX_POINTS = 250 };   // + latency <= 255 updates per command

  // radio: NULL where the board has another radio (the scanner then only says so, see hasRadio())
  SX126xSpectrumScanner(SX1262* radio) : _radio(radio), _num_points(0), _active(false) { }

  bool hasRadio() const { return _radio != NULL; }
  bool isActive() const { return _active; }

  // resets the radio into GFSK and loads the plugin
  int16_t begin(float freq_mhz, float tcxo, bool use_ldo) {
    if (_radio == NULL) return RADIOLIB_ERR_UNSUPPORTED;
    resetChip();   // a plugin of an earlier run may still be there, eg. after a restart of the MCU alone
    int16_t err = _radio->beginFSK(freq_mhz, 4.8, 5.0, 234.3, 0, 16, tcxo, use_ldo);
    if (err == RADIOLIB_ERR_SPI_CMD_FAILED || err == RADIOLIB_ERR_SPI_CMD_INVALID) {
      err = _radio->beginFSK(freq_mhz, 4.8, 5.0, 234.3, 0, 16, 0.0f, use_ldo);   // same TCXO fallback as std_init()
    }
    if (err != RADIOLIB_ERR_NONE) return err;

    char ver[16];   // after the reset, so the plugin's tag isn't there yet
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

    // RadioLib's default is 2MHz. The SX126x allows 16MHz (as does the nRF52840's SPIM3, the radio's bus on
    // the Mesh Pocket), but reading the results takes ~0.3ms of a 4-56ms sweep at 8MHz already
    ArduinoHal* hal = (ArduinoHal*) _radio->mod->hal;
    _saved_spi = hal->spiSettings;
    hal->spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
    _active = true;
    return RADIOLIB_ERR_NONE;
  }

  // Updates skipped after a retune: the point gets the next one, (latency + 1) updates after its retune.
  // The RSSI shows a retune ~3.4 updates later at narrow RBW, ~4.2 at 150kHz and up (measured on a steady
  // carrier); these values gave the best match with a slow per-point reference.
  static uint8_t latencyFor(float rbw_khz) {
    return rbw_khz < 17 ? 2 : (rbw_khz < 150 ? 3 : 4);
  }

  // Configure a sweep of num_points (<= MAX_POINTS) from start_mhz in step_mhz increments, with an RX
  // bandwidth of at least rbw_khz.
  int16_t setSweep(float start_mhz, float step_mhz, uint16_t num_points, float rbw_khz) {
    if (num_points == 0 || num_points > MAX_POINTS) return RADIOLIB_ERR_INVALID_FREQUENCY;
    float rbw = pickRxBw(rbw_khz);
    _radio->standby();
    int16_t err = _radio->setRxBandwidth(rbw);
    if (err != RADIOLIB_ERR_NONE) return err;
    err = _radio->setFrequency(start_mhz + step_mhz * num_points / 2);   // includes image calibration
    if (err != RADIOLIB_ERR_NONE) return err;
    // standby() put an external RF switch (RXEN/TXEN pins, eg. E22 modules) to idle, and the low-level setRx()
    // doesn't set it back, unlike startReceive(): without this the sweep would run with the LNA path off
    _radio->mod->setRfSwitchState(Module::MODE_RX);
    err = _radio->setRx(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    if (err != RADIOLIB_ERR_NONE) return err;

    // poll the RSSI register ~8 times per update (a poll costs ~0.6us, a delay loop 0.125us: 2 cycles
    // of the radio's 16MHz core), and give up waiting for an update after ~3 periods (a steady signal)
    float update_us = 1600.0f / rbw;
    float delay = (update_us / 8 - 0.6f) / 0.125f;
    uint8_t d = delay < 0 ? 0 : (delay > 255 ? 255 : (uint8_t)delay);
    float wait = 3 * update_us / (0.6f + 0.125f * d);
    uint8_t w = wait < 4 ? 4 : (wait > 255 ? 255 : (uint8_t)wait);

    uint32_t f = (uint32_t)(start_mhz * 1048576.0);   // REG_FREQ units: MHz * 2^25 / 32
    uint32_t s = (uint32_t)(step_mhz * 1048576.0 + 0.5);
    uint8_t p[] = { (uint8_t)(f >> 24), (uint8_t)(f >> 16), (uint8_t)(f >> 8), (uint8_t)f,
                    (uint8_t)(s >> 16), (uint8_t)(s >> 8), (uint8_t)s,
                    (uint8_t)num_points, latencyFor(rbw), d, w };
    memcpy(_params, p, sizeof(_params));
    _num_points = num_points;
    return RADIOLIB_ERR_NONE;
  }

  uint16_t numPoints() const { return _num_points; }

  // Runs one sweep (blocks for its duration). 'level' gets -2 * dBm per point, as GetRssiInst; it's only
  // valid if this returns RADIOLIB_ERR_NONE.
  int16_t sweep(uint8_t* level) {
    if (!_active || _num_points == 0) return RADIOLIB_ERR_INVALID_FREQUENCY;
    // a reset or brownout would leave the opcode pointing into whatever is in PRAM: check the plugin is there
    if (_radio->mod->SPIreadRegister(REG_VERSION_TAG) != SX126X_SPECTRUM_PLUGIN_TAG) return RADIOLIB_ERR_CHIP_NOT_FOUND;
    int16_t err = _radio->mod->SPIwriteStream(CMD_SWEEP, _params, sizeof(_params));   // BUSY timeout
    if (err == RADIOLIB_ERR_NONE) err = _radio->mod->SPIcheckStream();   // the plugin's status (bad params)
    if (err == RADIOLIB_ERR_NONE) err = _radio->readRegister(REG_RESULT, level, _num_points);
    return err;
  }

  // also after a begin() that failed half-way
  void end() {
    if (_radio == NULL) return;
    resetChip();   // for the LoRa setup that comes next: no plugin left behind
    if (_active) ((ArduinoHal*) _radio->mod->hal)->spiSettings = _saved_spi;
    _active = false;
  }
};
