#pragma once

#include <GxEPD2_BW.h>

#ifndef EINK_FAST_KICK_FRAMES
  #define EINK_FAST_KICK_FRAMES 0     // optional: push a changing pixel the other way first (cleaner, less contrast)
#endif
#ifndef EINK_FAST_FRAMES
  #define EINK_FAST_FRAMES      8     // frames towards the new colour: contrast; each costs ~5.8ms per refresh
#endif
#ifndef EINK_CLEAR_FRAMES
  #define EINK_CLEAR_FRAMES     16    // clear: frames to black, then as many to white (DC-balanced)...
#endif
#ifndef EINK_CLEAR_CYCLES
  #define EINK_CLEAR_CYCLES     3     // ...this many times: 3 x 32 frames ~0.56s
#endif
#ifndef EINK_FAST_FRAME_RATE
  #define EINK_FAST_FRAME_RATE  0x77  // SSD1680 FR code: 0x77 ~ 5.8ms per frame, 0x22 ~ 20ms
#endif

// A GxEPD2 panel class on an SSD1680 / SSD1681 controller (eg. GxEPD2_213_B74, GxEPD2_150_BN, GxEPD2_154_D67)
// with an optional fast partial refresh, for live views (eg. a spectrum). Tuned on a GDEM0213B74 (Mesh Pocket);
// other panels may want other EINK_FAST_FRAMES / EINK_FAST_FRAME_RATE for their contrast. The waveforms keep DC
// balance whatever the settings: the same frames both ways.
//  - a short custom waveform for the pixels that changed (optionally a kick the other way first, then
//    towards the new colour), instead of the panel's built-in (OTP) partial waveform, which takes ~1s.
//    Contrast is lower. 8 frames keep ~10 FPS; a 1-frame kick took visible contrast for little gain.
//    Pixels that don't change are not driven at all (VSS), as in the OTP partial waveform. Driving them on
//    every frame did harm both ways: a one-way push left DC charge (a light band that faded over minutes),
//    and a balanced shake slowly washed out black text and greyed the white background from the edges.
//    DC still balances per pixel over each white->black->white cycle.
//  - a fast clear (all black, then all white), for a clean start without the full refresh's 3.6s.
//  - the panel stays powered between frames (update control 0x04, not 0xC4/0xCC, which re-run the analog
//    power-up every time and cost ~90ms).
//  - full-frame writes in a single SPI transfer: GxEPD2 sends the frame byte by byte, which on nRF52
//    costs ~18us per byte (a DMA setup each), ~220ms per refresh.
//  - the refresh doesn't block: the next frame can be drawn while the panel updates. The 'previous'
//    RAM (0x26) is written from our own copy just before the next frame, rather than after the refresh.
template <class Panel>
class GxEPD2_Fast : public Panel {
  using Panel::_writeCommand;
  using Panel::_writeData;
  using Panel::_waitWhileBusy;
  using Panel::_power_is_on;
  using Panel::power_on_time;
  using Panel::_initial_write;
  using Panel::_initial_refresh;
  using Panel::_init_display_done;
  using Panel::_pSPIx;
  using Panel::_spi_settings;
  using Panel::_dc;
  using Panel::_cs;
  enum { FRAME_BYTES = Panel::WIDTH / 8 * Panel::HEIGHT };
  bool _fast = false, _refreshing = false, _prev_valid = false;
  uint8_t _prev[FRAME_BYTES];   // what the panel shows (or is busy showing)

  // 153-byte SSD1680 LUT: VS bytes for LUT0 = B->B, LUT1 = B->W, LUT2 = W->B, LUT3 = W->W, LUT4 = VCOM, 12 groups
  // each (2 bits per phase A..D: 01 = VSH1 -> black, 10 = VSL -> white); then 7 timing bytes per group
  // (TP A, TP B, SR AB, TP C, TP D, SR CD, RP); frame rates; gate scan.
  enum { VS_BLACK_THEN_WHITE = 0x60, VS_WHITE_THEN_BLACK = 0x90 };   // phase A, phase B
  static uint8_t& vs(uint8_t* lut, int transition, int group) { return lut[12 * transition + group]; }
  static uint8_t* timing(uint8_t* lut, int group) { return &lut[60 + 7 * group]; }

  void loadLut(uint8_t* lut) {
    memset(&lut[144], EINK_FAST_FRAME_RATE, 6);
    _writeCommand(0x32);
    _writeData(lut, 153);
  }

  void loadFastLut() {
    uint8_t lut[153];
    memset(lut, 0, sizeof(lut));       // LUT0 (B->B) and LUT3 (W->W): VSS, no drive
    vs(lut, 1, 0) = VS_BLACK_THEN_WHITE;   // B->W: kick, then white
    vs(lut, 2, 0) = VS_WHITE_THEN_BLACK;   // W->B: kick, then black
    timing(lut, 0)[0] = EINK_FAST_KICK_FRAMES;   // TP A
    timing(lut, 0)[1] = EINK_FAST_FRAMES;        // TP B
    loadLut(lut);
  }

  void loadClearLut() {
    uint8_t lut[153];
    memset(lut, 0, sizeof(lut));
    for (int t = 0; t < 4; t++) vs(lut, t, 0) = VS_BLACK_THEN_WHITE;   // whatever the RAM says
    timing(lut, 0)[0] = EINK_CLEAR_FRAMES;   // TP A
    timing(lut, 0)[1] = EINK_CLEAR_FRAMES;   // TP B
    timing(lut, 0)[6] = EINK_CLEAR_CYCLES - 1;   // RP: repeats
    loadLut(lut);
  }

  void powerOn() {
    if (_power_is_on) return;
    _writeCommand(0x22);
    _writeData(0xc0);  // clock + analog on
    _writeCommand(0x20);
    _waitWhileBusy("_PowerOn", power_on_time);
    _power_is_on = true;
  }

  // Display mode 1, not 2: in mode 2 the controller appears to leave pixels whose RAM bits didn't change
  // undriven, whatever the LUT says (the hold and clear waveforms had no visible effect there).
  void update() {
    _writeCommand(0x22);
    _writeData(0x04);    // display (mode 1), no LUT load, no power cycling
    _writeCommand(0x20);
  }

  void waitIdle() {
    if (_refreshing) {
      _waitWhileBusy("_Update_Fast", 100);
      _refreshing = false;
    }
  }

  bool isFullFrame(int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm) const {
    return x == 0 && y == 0 && w == Panel::WIDTH && h == Panel::HEIGHT && !invert && !mirror_y && !pgm
        && !_initial_write && _init_display_done;   // else let GxEPD2 do its init on the slow path
  }

  void writeFrame(uint8_t ram_cmd, const uint8_t* bitmap) {
    _writeCommand(0x11); _writeData(0x03);   // x, y increment
    _writeCommand(0x44); _writeData(0x00); _writeData((Panel::WIDTH - 1) / 8);
    _writeCommand(0x45); _writeData(0x00); _writeData(0x00); _writeData((Panel::HEIGHT - 1) % 256); _writeData((Panel::HEIGHT - 1) / 256);
    _writeCommand(0x4e); _writeData(0x00);
    _writeCommand(0x4f); _writeData(0x00); _writeData(0x00);
    _writeCommand(ram_cmd);
    _pSPIx->beginTransaction(_spi_settings);
    digitalWrite(_dc, HIGH);
    digitalWrite(_cs, LOW);
  #ifdef ESP32
    _pSPIx->writeBytes(bitmap, FRAME_BYTES);
  #else
    _pSPIx->transfer(bitmap, NULL, (size_t)FRAME_BYTES);
  #endif
    digitalWrite(_cs, HIGH);
    _pSPIx->endTransaction();
  }

public:
  GxEPD2_Fast(int16_t cs, int16_t dc, int16_t rst, int16_t busy) : Panel(cs, dc, rst, busy) { }

  void setFastPartial(bool fast) {
    if (fast == _fast) return;
    waitIdle();
    if (!fast && _prev_valid) writeFrame(0x26, _prev);   // the slow path expects 0x26 = what's on screen
    _prev_valid = false;
    _fast = fast;
  }

  void writeImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false) {
    waitIdle();
    if (!isFullFrame(x, y, w, h, invert, mirror_y, pgm)) {
      Panel::writeImage(bitmap, x, y, w, h, invert, mirror_y, pgm);
      return;
    }
    if (_fast) {
      // on the first fast frame, 0x26 already holds the screen (GxEPD2's writeImageAgain)
      if (_prev_valid) writeFrame(0x26, _prev);
      memcpy(_prev, bitmap, FRAME_BYTES);
      _prev_valid = true;
    }
    writeFrame(0x24, bitmap);
  }

  // after a refresh, GxEPD2 makes the 'previous' RAM equal to the current one
  void writeImageAgain(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false) {
    if (_fast) return;   // done from _prev before the next frame instead, so the refresh needn't be waited for
    if (isFullFrame(x, y, w, h, invert, mirror_y, pgm)) {
      writeFrame(0x26, bitmap);
      writeFrame(0x24, bitmap);
    } else {
      Panel::writeImageAgain(bitmap, x, y, w, h, invert, mirror_y, pgm);
    }
  }

  void refresh(bool partial_update_mode = false) {
    if (partial_update_mode) {
      refresh(0, 0, Panel::WIDTH, Panel::HEIGHT);
    } else {
      waitIdle();
      Panel::refresh(false);
    }
  }

  void refresh(int16_t x, int16_t y, int16_t w, int16_t h) {
    waitIdle();
    if (!_fast || _initial_refresh) {
      Panel::refresh(x, y, w, h);
      return;
    }
    // always the whole panel: GxEPD2_BW::display() has just written the frame in full
    powerOn();
    loadFastLut();       // the OTP partial refresh (0xFC) reloads its own LUT, so set ours every time
    update();
    _refreshing = true;  // waited for before the next write
  }

  // Wipes the panel to white (~0.6s, blocking). Returns false if not possible yet.
  bool clearFast() {
    waitIdle();
    if (!_init_display_done || _initial_refresh) return false;
    powerOn();
    loadClearLut();
    update();
    _waitWhileBusy("_Clear_Fast", 1500);
    memset(_prev, 0xFF, FRAME_BYTES);   // the panel is now white: so are both RAMs
    writeFrame(0x26, _prev);
    writeFrame(0x24, _prev);
    _prev_valid = true;
    return true;
  }

  void powerOff() { waitIdle(); Panel::powerOff(); }
  void hibernate() { waitIdle(); Panel::hibernate(); }
};
