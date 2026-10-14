#pragma once

#include <RadioLib.h>
#include "SX126xSpectrumPlugin.h"

// Frequency sweeps on an SX1262, run by the radio itself. A plugin on the radio's core (SX126xSpectrumPlugin.h)
// sweeps on in the background, in rows up and down the span: it retunes the receiver after every RSSI update,
// files the reading that belongs to each point, and keeps the strongest level per point until the host takes
// it, at the end of a row (readFrame()).
//
// The RSSI register updates every ~1.6/RBW ms (RBW in kHz) and a point takes about one update: a row of 250
// points takes ~56ms at an RBW of 7.3kHz, ~21ms at 19.5kHz, ~5ms at 78kHz.
//
// While begin() is in effect the radio is in GFSK RX and can't do LoRa. end() resets the chip with a cold sleep
// (many boards don't wire NRESET), which removes the plugin and every setting made here.
//
// Gain: the radio's AGC (default), or a fixed step (setGain()). The AGC steps the gain down from ~-85dBm in the
// channel and back up ~5 RSSI updates after the signal is gone, and every step holds the RSSI register for ~6
// more updates: a sweep shows a strong signal with a shelf of ~10 points after it. A fixed step has none of
// that, and the RSSI stays calibrated at every step; a lower step takes stronger signals at the cost of noise.
// Register 0x08B5 bit 0 (also set by CustomSX1262 for boards with SX126X_REGISTER_PATCH) keeps a strong signal
// near the channel from stopping the RSSI: without it a fixed step's RSSI freezes for good from ~-34dBm at the
// top steps (-5dBm at the lowest) until the receiver restarts, and a held AGC drops to its lowest step and stays.
// The registers of this paragraph aren't in the datasheet: they were found on hardware.
class SX126xSpectrumScanner {
  enum {
    CMD_HOLD = 0x9C,            // the plugin's command: sweeps in the background, the strongest level per point kept
    REG_HOLD_DONE = 0x0332,     // 1 once the held sweeps have stopped
    REG_HOLD_LOCK = 0x0333,     // the buffer's handover, see readFrame()
    REG_SCAN_RUN = 0x089B,      // 0 stops the held sweeps (at the end of a row)
    FRAME_WAIT_MS = 500,        // for the end of a row: ~110ms at most (250 points at an RBW of 4.8kHz)
    REG_RESULT = 0x0400,        // the data buffer, where the plugin keeps one level per point
    REG_MANUAL_GAIN = 0x08B6,   // bit 5: manual gain, bits 3..0: the step (see setGain())
    REG_AGC_HOLD = 0x08A7,      // bit 0: the AGC holds still (see startRx())
    REG_AGC_PATCH = 0x08B5,     // bit 0: no frozen RSSI on strong signals (see the top)
    MANUAL_GAIN_AFTER_RX_MS = 5,   // see startRx()
    REG_VERSION = 0x0320,       // the ROM version string, 16 chars
    // in GFSK RX, noise that passes for a packet is written to the data buffer too: a fixed packet length of
    // one byte, at the very end of the buffer, keeps it away from the results
    RX_BASE = 0xFF,
  };

  SX1262* _radio;
  uint8_t _gain = 0, _agc_reg = 0x0A;   // see setGain(); 0x08B6 as begin() found it
  uint8_t _params[13];   // CMD_HOLD's parameters, see setSweep()
  uint8_t _points;
  bool _active, _holding;
  uint32_t _asked_at;   // the latest frame asked for (readFrame())

  // RSSI updates to skip after a retune: readings trail the frequency by ~3 updates up to 78kHz (seen in sweeps
  // up and down over a steady carrier), ~4 at 150kHz and wider
  static uint8_t latencyFor(float rbw_khz) {
    return rbw_khz < 150 ? 3 : 4;
  }

public:
  enum { MAX_POINTS = 250 };   // the plugin counts points + skipped updates in a byte
  enum { GAIN_STEPS = 13 };    // setGain(): 1..13 = the AGC's steps from the lowest to the highest

  // The GFSK RX bandwidths and their RSSI update periods (measured).
  struct RxBw { float khz; float update_us; };
  static const RxBw* rxBw(float khz) {
    static const RxBw bws[] = {
      { 4.8, 333 }, { 5.8, 276 }, { 7.3, 219 }, { 9.7, 165 }, { 11.7, 137 }, { 14.6, 110 }, { 19.5, 82 },
      { 23.4, 68 }, { 29.3, 55 }, { 39.0, 41 }, { 46.9, 34 }, { 58.6, 27 }, { 78.2, 20 }, { 93.8, 17 },
      { 117.3, 13.6 }, { 156.2, 10.2 }, { 187.2, 8.5 }, { 234.3, 6.8 }, { 312.0, 5.1 }, { 373.6, 4.3 },
      { 467.0, 3.4 },
    };
    const int n = sizeof(bws) / sizeof(bws[0]);
    for (int i = 0; i < n; i++) {
      if (bws[i].khz >= khz - 0.05f) return &bws[i];
    }
    return &bws[n - 1];
  }
  // the narrowest RX bandwidth at least 'khz' wide (kHz; 467 at most)
  static float pickRxBw(float khz) { return rxBw(khz)->khz; }

  // 0: the AGC; 1..GAIN_STEPS: a fixed gain step, lowest first. Takes effect with the next setSweep().
  // The steps (as the AGC goes through them): 12 14 16 18 1A 1C 2A 2C 3C 4C 5C 6C 7C. The RSSI's own floor at an
  // RBW of 19.5kHz is -83 / -88 / -92 / -94 / -100 / -104 / -106 dBm on the lowest seven, -110..-112 from 2C up;
  // the AGC itself sits at 6C (12) when there's nothing strong.
  void setGain(uint8_t gain) { _gain = gain > GAIN_STEPS ? GAIN_STEPS : gain; }

  explicit SX126xSpectrumScanner(SX1262* radio) : _radio(radio), _points(0), _active(false), _holding(false), _asked_at(0) { }

  // Puts the radio into GFSK RX and loads the plugin: RADIOLIB_ERR_UNSUPPORTED if the chip has another ROM.
  // Expects a chip without the plugin, as from a reset or after end().
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
    if (_radio->mod->SPIreadRegister(SX126X_SPECTRUM_PLUGIN_TAG_REG) != SX126X_SPECTRUM_PLUGIN_TAG) return RADIOLIB_ERR_CHIP_NOT_FOUND;

    err = _radio->fixedPacketLengthMode(1);
    if (err == RADIOLIB_ERR_NONE) err = _radio->setBufferBaseAddress(0, RX_BASE);
    if (err != RADIOLIB_ERR_NONE) return err;

  #ifdef SX126X_DIO2_AS_RF_SWITCH
    _radio->setDio2AsRfSwitch(SX126X_DIO2_AS_RF_SWITCH);
  #endif
    uint8_t r = 0;   // see the top
    err = _radio->readRegister(REG_AGC_PATCH, &r, 1);
    r |= 0x01;
    if (err == RADIOLIB_ERR_NONE) err = _radio->writeRegister(REG_AGC_PATCH, &r, 1);
    if (err == RADIOLIB_ERR_NONE) err = _radio->readRegister(REG_MANUAL_GAIN, &_agc_reg, 1);
    if (err != RADIOLIB_ERR_NONE) return err;
    _active = true;
    return RADIOLIB_ERR_NONE;
  }

  // Sets up sweeps of 'points' (1..MAX_POINTS) points, from start_mhz in steps of step_mhz, measured through
  // the narrowest RX bandwidth of at least rbw_khz. Stops held sweeps; startHold() starts them again.
  int16_t setSweep(float start_mhz, float step_mhz, uint8_t points, float rbw_khz) {
    if (points == 0 || points > MAX_POINTS) return RADIOLIB_ERR_INVALID_NUM_SAMPLES;
    const RxBw* bw = rxBw(rbw_khz);
    float rbw = bw->khz;
    int16_t err = stopHold();
    if (err == RADIOLIB_ERR_NONE) err = _radio->standby();
    // as the gain steps were measured; the mesh's setting comes back with its own radio setup
    if (err == RADIOLIB_ERR_NONE) err = _radio->setRxBoostedGainMode(true);
    if (err == RADIOLIB_ERR_NONE) err = _radio->setRxBandwidth(rbw);
    if (err != RADIOLIB_ERR_NONE) return err;
    // After a SetRfFrequency the image rejection is 15..30dB instead of ~43 (a strong signal shows again 2 x IF
    // away, eg. 400kHz up at an RBW of 23.4kHz) until a CalibrateImage, which RadioLib's setFrequency() only does
    // for a move of 20MHz or more. So it's done every time (~6.6ms).
    float centre = start_mhz + step_mhz * points / 2;
    err = _radio->calibrateImage(centre);
    if (err == RADIOLIB_ERR_NONE) err = _radio->setFrequency(centre);
    if (err != RADIOLIB_ERR_NONE) return err;
    err = startRx();
    if (err != RADIOLIB_ERR_NONE) return err;

    // The plugin polls the RSSI register for each update: ~8 polls per update period, a poll taking ~0.6us
    // plus 'delay' x 0.125us. 3-5% of updates keep the value (more on strong signals at a wide RBW): when no
    // change comes within ~1.25 periods, the reading is taken as it is and the next update is due ~1 period on.
    float update_us = bw->update_us;
    float delay = (update_us / 8 - 0.6f) / 0.125f;
    uint8_t d = delay < 0 ? 0 : (delay > 255 ? 255 : (uint8_t)delay);
    float poll_us = 0.6f + 0.125f * d;
    uint8_t w1 = waitPolls(1.25f * update_us / poll_us), w2 = waitPolls(1.1f * update_us / poll_us);

    // frequencies in the radio's units: 32MHz / 2^25 per unit, ie. 2^20 per MHz
    uint32_t f = (uint32_t)(start_mhz * 1048576.0);
    uint32_t s = (uint32_t)(step_mhz * 1048576.0 + 0.5);
    uint8_t p[] = {
      (uint8_t)(f >> 24), (uint8_t)(f >> 16), (uint8_t)(f >> 8), (uint8_t)f,   // first point
      (uint8_t)(s >> 16), (uint8_t)(s >> 8), (uint8_t)s,                       // step
      points, latencyFor(rbw), d, w1, w2,
      0,                                                                       // rows up and down
    };
    memcpy(_params, p, sizeof(_params));
    _points = points;
    return RADIOLIB_ERR_NONE;
  }

  // Starts held sweeps of the settings of setSweep(). Until stopHold(), the radio's core runs them: commands that
  // need it are dropped meanwhile (registers can still be read and written).
  int16_t startHold() {
    if (!_active || _points == 0) return RADIOLIB_ERR_WRONG_MODEM;
    // the plugin is gone if the chip was reset (eg. a brownout): its command would then run whatever is in RAM
    if (_radio->mod->SPIreadRegister(SX126X_SPECTRUM_PLUGIN_TAG_REG) != SX126X_SPECTRUM_PLUGIN_TAG) return RADIOLIB_ERR_CHIP_NOT_FOUND;
    uint8_t ff[MAX_POINTS], unlocked = 0;
    memset(ff, 0xFF, _points);
    int16_t err = _radio->writeRegister(REG_RESULT, ff, _points);
    if (err == RADIOLIB_ERR_NONE) err = _radio->writeRegister(REG_HOLD_LOCK, &unlocked, 1);
    if (err == RADIOLIB_ERR_NONE) err = _radio->mod->SPIwriteStream(CMD_HOLD, _params, sizeof(_params));   // returns at once
    _holding = err == RADIOLIB_ERR_NONE;
    return err;
  }

  // A frame: the strongest level per point, -2 * dBm (as GetRssiInst), of the whole rows (once across the span)
  // swept since the previous one. Doesn't wait: the first call asks the plugin for it, and a later one takes it
  // (*got) once the plugin has finished the row under way.
  // The radio's buffer has one read path for its core and the SPI: a read by the core while the host reads it
  // hands the host the core's byte (and the other way round). So the plugin hands it over: the host asks (lock
  // 1), the plugin sets 2 at the end of a row and waits, the host reads the buffer and sets 3, and the plugin
  // clears it, sets 0 and goes on from point 0. The lock itself is in the radio's RAM, which has no such
  // collision.
  int16_t readFrame(uint8_t* level, bool* got) {
    *got = false;
    if (!_holding) return RADIOLIB_ERR_WRONG_MODEM;
    uint8_t lock;
    int16_t err = _radio->readRegister(REG_HOLD_LOCK, &lock, 1);
    if (err != RADIOLIB_ERR_NONE) return err;
    if (lock == 0) {
      lock = 1;
      _asked_at = millis();
      return _radio->writeRegister(REG_HOLD_LOCK, &lock, 1);
    }
    if (lock != 2) return millis() - _asked_at > FRAME_WAIT_MS ? RADIOLIB_ERR_SPI_CMD_TIMEOUT : RADIOLIB_ERR_NONE;
    err = _radio->readRegister(REG_RESULT, level, _points);
    lock = 3;
    if (err == RADIOLIB_ERR_NONE) err = _radio->writeRegister(REG_HOLD_LOCK, &lock, 1);
    *got = err == RADIOLIB_ERR_NONE;
    return err;
  }

  // Stops held sweeps, at the end of the row under way.
  int16_t stopHold() {
    if (!_holding) return RADIOLIB_ERR_NONE;
    _holding = false;
    uint8_t zero = 0, done = 0;
    int16_t err = _radio->writeRegister(REG_SCAN_RUN, &zero, 1);
    for (uint32_t t = millis(); err == RADIOLIB_ERR_NONE && millis() - t < 500; delay(1)) {
      err = _radio->readRegister(REG_HOLD_DONE, &done, 1);
      if (done == 1) return err;
    }
    return err == RADIOLIB_ERR_NONE ? RADIOLIB_ERR_SPI_CMD_TIMEOUT : err;
  }

  // Leaves the chip as a reset does, for the radio's own begin() to take it from there.
  void end() {
    stopHold();
    // like a pulse on NRESET, wired or not: a cold sleep keeps nothing, and the chip boots again when woken
    _radio->standby();
    _radio->sleep(false);
    _radio->standby(RADIOLIB_SX126X_STANDBY_RC, true);   // wakes it, then waits for the boot (BUSY)
    // The reset also took the image calibration (back to 902-928MHz), and RadioLib's begin() redoes it only on a
    // move of 20MHz from the frequency it last set (our last centre): this makes it redo it.
    _radio->freqMHz = 0;
    _active = false;
  }

private:
  // Starts the receiver, with the AGC: started with a fixed step its RSSI stays dead. Nor does a step set too soon
  // after SetRx take: from STDBY_XOSC (the TCXO kept on) the receiver needs ~0.5ms at an RBW of 19.5kHz, 1-2ms at
  // 4.8-7.3kHz, and delay() may come a tick early.
  int16_t startRx() {
    int16_t err = _radio->standby(RADIOLIB_SX126X_STANDBY_XOSC);
    // standby() sets an external RF switch (RXEN/TXEN pins) to idle, and setRx() doesn't set it back (unlike
    // startReceive())
    _radio->mod->setRfSwitchState(Module::MODE_RX);
    uint8_t hold = 0;
    if (err == RADIOLIB_ERR_NONE) err = _radio->writeRegister(REG_AGC_HOLD, &hold, 1);
    if (err == RADIOLIB_ERR_NONE) err = _radio->writeRegister(REG_MANUAL_GAIN, &_agc_reg, 1);
    if (err == RADIOLIB_ERR_NONE) err = _radio->setRx(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    if (err != RADIOLIB_ERR_NONE || _gain == 0) return err;
    delay(MANUAL_GAIN_AFTER_RX_MS);
    uint8_t g = 0x20 | _gain;   // the step codes are 1..13 (0 mutes the receiver)
    err = _radio->writeRegister(REG_MANUAL_GAIN, &g, 1);
    // With a fixed step alone, the AGC still acts on a strong signal: the RSSI holds for ~3 more points past it, then
    // drops (a shelf at the peak's level). Holding the AGC as well leaves a smooth fall (the RSSI's own averaging).
    hold = 1;
    if (err == RADIOLIB_ERR_NONE) err = _radio->writeRegister(REG_AGC_HOLD, &hold, 1);
    return err;
  }

  static uint8_t waitPolls(float polls) {
    return polls < 4 ? 4 : (polls > 255 ? 255 : (uint8_t)polls);
  }
};
