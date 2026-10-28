#pragma once

#include <GxEPD2_BW.h>
#include "SSD1680Lut.h"

#ifndef EINK_FAST_KICK_FRAMES
  #define EINK_FAST_KICK_FRAMES 0     // optional: push a changing pixel the other way first (cleaner, less contrast)
#endif
#ifndef EINK_FAST_FRAMES
  #define EINK_FAST_FRAMES      8     // frames towards the new colour: contrast; each costs ~5.8ms per refresh
#endif
#ifndef EINK_FAST_FRAME_RATE
  #define EINK_FAST_FRAME_RATE  0x77  // SSD1680 FR code: 0x77 ~ 5.8ms per frame, 0x22 ~ 20ms
#endif

// GDEM0213B74 (SSD1680) with an optional fast partial refresh, for live views (eg. a spectrum):
//  - a short custom waveform for the pixels that changed (optionally a kick the other way first, then
//    towards the new colour), instead of the panel's built-in (OTP) partial waveform, which takes ~1s.
//    Contrast is lower. 8 frames keep ~10 FPS; a 1-frame kick took visible contrast for little gain.
//    Pixels that don't change are not driven at all (VSS), as in the OTP partial waveform. Driving them on
//    every frame did harm both ways: a one-way push left DC charge (a light band that faded over minutes),
//    and a balanced shake slowly washed out black text and greyed the white background from the edges.
//    DC still balances per pixel over each white->black->white cycle.
//  - wipes to white that leave no charge of their own in the panel (test/test_ssd1680_lut): a fast picture with a
//    fast refresh to white, which closes the black-white cycles of the fast waveform; the slow path's with the
//    panel's own full refresh, as GxEPD2's clearScreen() does it (a clear waveform of our own greyed the panel).
//  - the panel stays powered between frames (update control 0x04, not 0xC4/0xCC, which re-run the analog
//    power-up every time and cost ~90ms).
//  - full-frame writes in a single SPI transfer: GxEPD2 sends the frame byte by byte, which on nRF52
//    costs ~18us per byte (a DMA setup each), ~220ms per refresh.
//  - the refresh doesn't block: the next frame can be drawn while the panel updates. The 'previous'
//    RAM (0x26) is written from our own copy just before the next frame, rather than after the refresh.
class GxEPD2_213_B74_Fast : public GxEPD2_213_B74 {
  enum { FRAME_BYTES = WIDTH / 8 * HEIGHT };
  bool _fast = false, _refreshing = false, _prev_valid = false;
  uint8_t _prev[FRAME_BYTES];   // what the panel shows (or is busy showing)

  bool _screen_fast = false;    // the black pixels on the panel are the fast waveform's (else the panel's own)

  void loadLut(uint8_t* lut) {
    memset(&lut[144], EINK_FAST_FRAME_RATE, 6);
    _writeCommand(0x32);
    _writeData(lut, SSD1680Lut::LEN);
  }

  void loadFastLut() {
    uint8_t lut[SSD1680Lut::LEN];
    SSD1680Lut::fastLut(lut, EINK_FAST_KICK_FRAMES, EINK_FAST_FRAMES);
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
    return x == 0 && y == 0 && w == WIDTH && h == HEIGHT && !invert && !mirror_y && !pgm
        && !_initial_write && _init_display_done;   // else let GxEPD2 do its init on the slow path
  }

  void writeFrame(uint8_t ram_cmd, const uint8_t* bitmap) {
    _writeCommand(0x11); _writeData(0x03);   // x, y increment
    _writeCommand(0x44); _writeData(0x00); _writeData((WIDTH - 1) / 8);
    _writeCommand(0x45); _writeData(0x00); _writeData(0x00); _writeData((HEIGHT - 1) % 256); _writeData((HEIGHT - 1) / 256);
    _writeCommand(0x4e); _writeData(0x00);
    _writeCommand(0x4f); _writeData(0x00); _writeData(0x00);
    _writeCommand(ram_cmd);
    _pSPIx->beginTransaction(_spi_settings);
    digitalWrite(_dc, HIGH);
    digitalWrite(_cs, LOW);
    _pSPIx->transfer(bitmap, NULL, (size_t)FRAME_BYTES);
    digitalWrite(_cs, HIGH);
    _pSPIx->endTransaction();
  }

public:
  GxEPD2_213_B74_Fast(int16_t cs, int16_t dc, int16_t rst, int16_t busy) : GxEPD2_213_B74(cs, dc, rst, busy) { }

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
      GxEPD2_213_B74::writeImage(bitmap, x, y, w, h, invert, mirror_y, pgm);
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
      GxEPD2_213_B74::writeImageAgain(bitmap, x, y, w, h, invert, mirror_y, pgm);
    }
  }

  void refresh(bool partial_update_mode = false) {
    if (partial_update_mode) {
      refresh(0, 0, WIDTH, HEIGHT);
    } else {
      waitIdle();
      GxEPD2_213_B74::refresh(false);
    }
  }

  void refresh(int16_t x, int16_t y, int16_t w, int16_t h) {
    waitIdle();
    if (!_fast || _initial_refresh) {
      GxEPD2_213_B74::refresh(x, y, w, h);
      return;
    }
    // always the whole panel: GxEPD2_BW::display() has just written the frame in full
    powerOn();
    loadFastLut();       // the OTP partial refresh (0xFC) reloads its own LUT, so set ours every time
    update();
    _refreshing = true;  // waited for before the next write
    _screen_fast = true;
  }

  // Wipes the panel to white (blocking, see the top): a fast picture with a fast refresh to white (~50ms), the slow
  // path's with the panel's full refresh (~3.6s). Returns false if not possible yet.
  bool wipeWhite() {
    waitIdle();
    if (!_init_display_done || _initial_refresh) return false;
    if (_screen_fast) {
      if (_prev_valid) writeFrame(0x26, _prev);
      memset(_prev, 0xFF, FRAME_BYTES);
      writeFrame(0x24, _prev);
      powerOn();
      loadFastLut();
      update();
      _waitWhileBusy("_Wipe_Fast", 500);
      writeFrame(0x26, _prev);
    } else {
      // both RAMs white first: the full refresh reads 0x26 too, and from a picture there it would draw it again
      memset(_prev, 0xFF, FRAME_BYTES);
      writeFrame(0x26, _prev);
      writeFrame(0x24, _prev);
      GxEPD2_213_B74::refresh(false);
    }
    _prev_valid = true;
    _screen_fast = false;
    return true;
  }

  void powerOff() { waitIdle(); GxEPD2_213_B74::powerOff(); }
  void hibernate() { waitIdle(); GxEPD2_213_B74::hibernate(); }
};
