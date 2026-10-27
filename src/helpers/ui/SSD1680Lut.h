#pragma once

#include <stdint.h>
#include <string.h>

// The custom SSD1680 waveforms of GxEPD2_213_B74_Fast, apart from the driver so that a host test can check them
// (charge(): the DC balance a waveform leaves on a pixel).
//
// 153-byte SSD1680 LUT: VS bytes for LUT0 = B->B, LUT1 = B->W, LUT2 = W->B, LUT3 = W->W (old -> new colour, from
// the RAMs 0x26 -> 0x24), LUT4 = VCOM, 12 groups each (2 bits per phase A..D, A in the top bits: 00 = VSS, no drive,
// 01 = VSH1 -> black, 10 = VSL -> white); then 7 timing bytes per group (TP A, TP B, SR AB, TP C, TP D, SR CD, RP:
// frames per phase, repeats of A-B and of C-D, repeats of the group); frame rates; gate and source voltages.
//
// DC balance: a pixel's drive towards black has to be undone by as much towards white over its black-white cycle,
// or the charge left in the panel greys it (and makes it hard to clear). Each waveform here is balanced within the
// cycles it draws itself; the clear has to close the cycles another waveform opened (see clearLut()).
namespace SSD1680Lut {

enum { LEN = 153, B_B = 0, B_W = 1, W_B = 2, W_W = 3 };
enum { VS_NONE = 0x00, VS_WHITE = 0x80, VS_BLACK_THEN_WHITE = 0x60, VS_WHITE_THEN_BLACK = 0x90 };   // phases A, B

inline uint8_t& vs(uint8_t* lut, int transition, int group) { return lut[12 * transition + group]; }
inline uint8_t* timing(uint8_t* lut, int group) { return &lut[60 + 7 * group]; }

// the pixels that change: an optional kick the other way, then towards the new colour; the others aren't driven
// (charge: W->B kick_frames - frames, B->W frames - kick_frames, so a cycle in this waveform balances)
inline void fastLut(uint8_t* lut, int kick_frames, int frames) {
  memset(lut, 0, LEN);
  vs(lut, B_W, 0) = VS_BLACK_THEN_WHITE;
  vs(lut, W_B, 0) = VS_WHITE_THEN_BLACK;
  timing(lut, 0)[0] = kick_frames;   // TP A
  timing(lut, 0)[1] = frames;        // TP B
}

// to white: every pixel black then white, cycles times (balanced), whatever is on the panel; then the pixels that
// were black (B->W) get extra_white frames more towards white, to close the cycle that the waveform which made them
// black left open (fast: its W->B charge, frames - kick_frames)
inline void clearLut(uint8_t* lut, int frames, int cycles, int extra_white) {
  memset(lut, 0, LEN);
  for (int t = 0; t < 4; t++) vs(lut, t, 0) = VS_BLACK_THEN_WHITE;
  timing(lut, 0)[0] = frames;       // TP A
  timing(lut, 0)[1] = frames;       // TP B
  timing(lut, 0)[6] = cycles - 1;   // RP: repeats
  if (extra_white > 0) {
    vs(lut, B_W, 1) = VS_WHITE;
    timing(lut, 1)[0] = extra_white;
  }
}

// the DC a waveform leaves on a pixel of a transition: frames towards black minus frames towards white
inline int charge(const uint8_t* lut, int transition) {
  int q = 0;
  for (int g = 0; g < 12; g++) {
    uint8_t v = lut[12 * transition + g];
    const uint8_t* t = &lut[60 + 7 * g];
    int phase[4];
    for (int p = 0; p < 4; p++) {
      int s = (v >> (6 - 2 * p)) & 3;
      phase[p] = s == 1 || s == 3 ? 1 : (s == 2 ? -1 : 0);   // VSH1, VSH2 -> black; VSL -> white
    }
    int ab = phase[0] * t[0] + phase[1] * t[1], cd = phase[2] * t[3] + phase[3] * t[4];
    q += (t[6] + 1) * ((t[2] + 1) * ab + (t[5] + 1) * cd);
  }
  return q;
}

}
