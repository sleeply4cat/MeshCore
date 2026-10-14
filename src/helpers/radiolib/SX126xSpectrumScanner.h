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
//
// Gain: fixed at the AGC's top step (7C), with the AGC held. With the AGC free, the gain steps down from ~-85 dBm in
// the channel and back ~5 RSSI updates after the signal is gone, and every step holds the RSSI register for ~6 more
// updates: a sweep showed a strong narrow signal with a shelf of ~10 points after it. The RSSI stays calibrated at
// a fixed step. Also found on hardware (sxplug docs/measurements.md), and handled here:
// - a strong signal near the channel (before the channel filter) makes the AGC try to step down at once; with a
//   fixed step that never happens and the RSSI stops for good (from ~-34 dBm at the top step), and a held AGC
//   drops to its lowest step and stays there. 0x08B5 bit 0 prevents both (MeshCore sets it on boards with a
//   front-end LNA, SX126X_REGISTER_PATCH); sweep() still restarts the receiver if the RSSI looks stuck;
// - after a SetRfFrequency the image rejection is 15..30 dB instead of ~43 until a CalibrateImage (a strong signal
//   shows again 2 x IF away), which RadioLib only does on a move of several MHz: setSweep() calibrates every time.
class SX126xSpectrumScanner {
  enum {
    CMD_SWEEP = 0x9B,
    REG_RESULT = 0x0400,        // RADIO_BUF
    REG_VERSION = 0x0320,       // ROM version string, 16 bytes
    REG_VERSION_TAG = 0x032E,   // last char of the version string, set by the plugin's init
    REG_MANUAL_GAIN = 0x08B6,   // bit 5: a fixed gain step, in bits 3..0 (1..13, lowest first)
    REG_AGC_HOLD = 0x08A7,      // bit 0: the AGC holds still
    REG_AGC_STATE = 0x08BF,     // bit 1: the RSSI held for a gain change (with a fixed step: maybe stuck)
    REG_AGC_PATCH = 0x08B5,     // bit 0: no stuck RSSI on strong signals (see the top)
    GAIN_STEP = 13,             // 7C, the top
    // a step written sooner after SetRx doesn't take, and the RSSI stays dead: from STDBY_XOSC (the TCXO kept on)
    // the receiver needs ~0.5 ms at an RBW of 19.5 kHz, 1-2 ms at 4.8-7.3 kHz
    GAIN_AFTER_RX_MS = 5,
    // In GFSK RX a packet (or noise passing for one) is written to the data buffer, which also holds the
    // results: fixed length 1, at the very end of the buffer, keeps it out of the way
    RX_BASE = 0xFF,
  };

  SX1262* _radio;
  uint8_t _params[11];
  uint16_t _num_points;
  uint8_t _agc_reg;   // 0x08B6 after the reset: the AGC
  bool _active;
  bool _stuck;        // 0x08BF bit 1 at the latest check
  SPISettings _saved_spi;

  // like a pulse on NRESET, wired or not: a cold sleep keeps nothing, and the chip boots again when woken (the
  // plugin is gone and '2' is back at the end of the version string)
  void resetChip() {
    _radio->standby();
    _radio->sleep(false);
    _radio->standby(RADIOLIB_SX126X_STANDBY_RC, true);   // wakes it, then waits for the boot (BUSY)
  }

  static float pickRxBw(float khz) {
    static const float bws[] = { 4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0, 46.9, 58.6,
                                 78.2, 93.8, 117.3, 156.2, 187.2, 234.3, 312.0, 373.6, 467.0 };
    for (int i = 0; i < sizeof(bws)/sizeof(bws[0]); i++) {
      if (bws[i] >= khz) return bws[i];
    }
    return 467.0;
  }

public:
  enum { MAX_POINTS = 250 };   // + latency <= 255 updates per command

  // radio: NULL where the board has another radio (the scanner then only says so, see hasRadio())
  SX126xSpectrumScanner(SX1262* radio) : _radio(radio), _num_points(0), _agc_reg(0x0A), _active(false), _stuck(false) { }

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
    uint8_t r = 0;
    err = _radio->readRegister(REG_AGC_PATCH, &r, 1);
    r |= 0x01;
    if (err == RADIOLIB_ERR_NONE) err = _radio->writeRegister(REG_AGC_PATCH, &r, 1);
    if (err == RADIOLIB_ERR_NONE) err = _radio->readRegister(REG_MANUAL_GAIN, &_agc_reg, 1);
    if (err != RADIOLIB_ERR_NONE) return err;
    _agc_reg &= ~0x20;
    _stuck = false;

    // RadioLib's default is 2MHz. The SX126x allows 16MHz (as does the nRF52840's SPIM3, the radio's bus on
    // the Mesh Pocket), but reading the results takes ~0.3ms of a 4-56ms sweep at 8MHz already
    ArduinoHal* hal = (ArduinoHal*) _radio->mod->hal;
    _saved_spi = hal->spiSettings;
    hal->spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
    _active = true;
    return RADIOLIB_ERR_NONE;
  }

  // Updates skipped after a retune: the point gets the next one, (latency + 1) updates after its retune.
  // Readings trail the frequency by ~3 updates up to 78kHz (seen in sweeps up and down over a steady carrier),
  // ~4 at 150kHz and wider.
  static uint8_t latencyFor(float rbw_khz) {
    return rbw_khz < 150 ? 3 : 4;
  }

  // Configure a sweep of num_points (<= MAX_POINTS) from start_mhz in step_mhz increments, with an RX
  // bandwidth of at least rbw_khz.
  int16_t setSweep(float start_mhz, float step_mhz, uint16_t num_points, float rbw_khz) {
    if (num_points == 0 || num_points > MAX_POINTS) return RADIOLIB_ERR_INVALID_FREQUENCY;
    float rbw = pickRxBw(rbw_khz);
    _radio->standby();
    int16_t err = _radio->setRxBandwidth(rbw);
    if (err != RADIOLIB_ERR_NONE) return err;
    float centre = start_mhz + step_mhz * num_points / 2;
    err = _radio->calibrateImage(centre);   // see the top (~6.6 ms)
    if (err == RADIOLIB_ERR_NONE) err = _radio->setFrequency(centre);
    if (err == RADIOLIB_ERR_NONE) err = startRx();
    if (err != RADIOLIB_ERR_NONE) return err;

    // poll the RSSI register ~8 times per update (a poll costs ~0.6us, a delay loop 0.125us: 2 cycles of the
    // radio's 16MHz core). 3-5% of updates keep the value: when no change comes within ~1.25 periods, the reading
    // is taken as it is
    float update_us = 1600.0f / rbw;
    float delay = (update_us / 8 - 0.6f) / 0.125f;
    uint8_t d = delay < 0 ? 0 : (delay > 255 ? 255 : (uint8_t)delay);
    float wait = 1.25f * update_us / (0.6f + 0.125f * d);
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
    int16_t err = rssiStuck() ? startRx() : RADIOLIB_ERR_NONE;
    if (err == RADIOLIB_ERR_NONE) err = _radio->mod->SPIwriteStream(CMD_SWEEP, _params, sizeof(_params));   // BUSY timeout
    if (err == RADIOLIB_ERR_NONE) err = _radio->mod->SPIcheckStream();   // the plugin's status (bad params)
    if (err == RADIOLIB_ERR_NONE) err = _radio->readRegister(REG_RESULT, level, _num_points);
    return err;
  }

private:
  // Starts the receiver with the AGC (started with a fixed step, its RSSI stays dead), then fixes the step and holds
  // the AGC (with the step alone, it still holds the RSSI for ~3 points past a strong signal).
  int16_t startRx() {
    _stuck = false;
    int16_t err = _radio->standby(RADIOLIB_SX126X_STANDBY_XOSC);
    // standby() put an external RF switch (RXEN/TXEN pins, eg. E22 modules) to idle, and the low-level setRx()
    // doesn't set it back, unlike startReceive(): without this the sweep would run with the LNA path off
    _radio->mod->setRfSwitchState(Module::MODE_RX);
    uint8_t v = 0;
    if (err == RADIOLIB_ERR_NONE) err = _radio->writeRegister(REG_AGC_HOLD, &v, 1);
    if (err == RADIOLIB_ERR_NONE) err = _radio->writeRegister(REG_MANUAL_GAIN, &_agc_reg, 1);
    if (err == RADIOLIB_ERR_NONE) err = _radio->setRx(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    if (err != RADIOLIB_ERR_NONE) return err;
    delay(GAIN_AFTER_RX_MS);
    v = 0x20 | GAIN_STEP;
    err = _radio->writeRegister(REG_MANUAL_GAIN, &v, 1);
    v = 0x01;
    if (err == RADIOLIB_ERR_NONE) err = _radio->writeRegister(REG_AGC_HOLD, &v, 1);
    return err;
  }

  // 0x08BF bit 1 at this check and the one before (it also comes up for a few updates whenever the AGC would step)
  bool rssiStuck() {
    uint8_t r = 0;
    bool now = _radio->readRegister(REG_AGC_STATE, &r, 1) == RADIOLIB_ERR_NONE && (r & 0x02);
    bool stuck = now && _stuck;
    _stuck = now && !stuck;
    return stuck;
  }

public:
  // also after a begin() that failed half-way
  void end() {
    if (_radio == NULL) return;
    resetChip();   // for the LoRa setup that comes next: no plugin, fixed gain or held AGC left behind
    // The reset also took the image calibration (back to the 902-928 MHz band), and RadioLib's begin() redoes it
    // only on a move of 20 MHz from the frequency it last set (our last sweep's centre): this makes it redo it
    _radio->freqMHz = 0;
    if (_active) ((ArduinoHal*) _radio->mod->hal)->spiSettings = _saved_spi;
    _active = false;
  }
};
