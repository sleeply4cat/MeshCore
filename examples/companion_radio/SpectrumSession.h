#pragma once

#include <Arduino.h>
#include <helpers/BaseSerialInterface.h>
#include "NodePrefs.h"

// Spectrum stream for a companion client: on request, the radio sweeps a band (SX126xSpectrumScanner) and the
// node streams the levels to the client to draw. The mesh is paused while it runs.
//
// The client starts a session with OP_START and repeats it at least every KEEPALIVE_MS while it watches; an
// OP_START with other settings changes them. The session ends on OP_STOP, when the client's link goes away,
// after KEEPALIVE_MS without an OP_START, or on a radio error, and the radio goes back to the mesh with the
// node's settings. Needs an SX1262 on an nRF52: elsewhere OP_START gets ERR_CODE_UNSUPPORTED_CMD.
//
// Frames start with CMD_SPECTRUM and an op; values are little-endian.
//
// OP_START (1), then:
//   u32 centre     Hz. 0: the node's frequency.
//   u32 span       Hz, at most 32MHz and at least 100Hz per point; the whole span within 150..960MHz.
//   u8  points     16..250, spread evenly: each point is the centre of its 1/points of the span.
//   u32 rbw        Hz, resolution bandwidth, rounded up to the radio's next GFSK bandwidth (4.8..467kHz).
//                  0: 2.5 steps. A sweep takes ~1.6ms per point / RBW in kHz (250 points: ~56ms at 7.3kHz).
//   u16 interval   ms between frames, 10..5000. 0: a frame per sweep, as fast as the link takes them.
//   u8  agg        what a frame holds of the sweeps since the previous one: AGG_MAX (the strongest level, so
//                  short bursts show), AGG_MEAN (mean power) or AGG_LAST (the latest sweep).
//   Reply: RESP_SPECTRUM_INFO, or ERR_CODE_ILLEGAL_ARG for settings out of range.
//
// OP_STOP (2), then optionally u8 flags:
//   STOP_DISCONNECT  drop the Bluetooth link afterwards (a web page can't: the OS may keep the link up).
//   Reply: RESP_SPECTRUM_INFO with STATE_OFF.
//
// RESP_SPECTRUM_INFO, then:
//   u8 state (STATE_*: STARTING while the mesh finishes a packet), u8 config, u8 points, u32 first point (Hz),
//   u32 step (Hz), u32 RBW (Hz), u16 interval (ms), u8 agg, u16 sweeps per second x10 (since the previous
//   reply; 0 until known)
// PUSH_SPECTRUM_FRAME, then:
//   u8 config, u8 frame, u8 first point, a level per point: -2 * dBm (0.5dB steps; 255: no reading yet).
//   A frame of more points than a companion frame holds (172) comes in several pushes of the same frame number,
//   which counts the frames sent (mod 256). 'config' changes with the sweep settings, so that frames of earlier
//   settings still on their way can be told apart.
// PUSH_SPECTRUM_STOPPED, then:
//   u8 reason (STOP_*), i16 RadioLib error (STOP_RADIO)
class SpectrumSession {
public:
  enum { CMD_SPECTRUM = 0xE0, OP_START = 1, OP_STOP = 2 };
  enum { STOP_DISCONNECT = 1 };   // OP_STOP flags
  enum { RESP_SPECTRUM_INFO = 0xE0, PUSH_SPECTRUM_FRAME = 0xE1, PUSH_SPECTRUM_STOPPED = 0xE2 };
  enum { STATE_OFF = 0, STATE_STARTING = 1, STATE_RUNNING = 2 };
  enum { AGG_MAX = 0, AGG_MEAN = 1, AGG_LAST = 2 };
  enum { STOP_CLIENT = 0, STOP_TIMEOUT = 1, STOP_DISCONNECTED = 2, STOP_RADIO = 3 };
  enum { KEEPALIVE_MS = 15000, MAX_POINTS = 250 };

  explicit SpectrumSession(NodePrefs* prefs) : _prefs(prefs) { }

  // a session is on (maybe waiting for the radio), or a link is to be dropped: loop() has work to do
  bool isActive() const { return _s != NULL || _drop_link_at != 0; }
  // the radio is sweeping: the mesh must leave it alone
  bool ownsRadio() const;

  // Handles a CMD_SPECTRUM frame: writes the reply into 'reply' and returns its length, or returns -ERR_CODE_*
  // for an error frame.
  int handleCommand(const uint8_t* frame, int len, uint8_t* reply);

  // From the main loop: starts the sweeps once the radio is free, sends frames, watches the client.
  void loop(BaseSerialInterface* serial);

private:
  NodePrefs* _prefs;
  struct State;
  State* _s = NULL;             // while a session is on
  uint32_t _drop_link_at = 0;   // when the Bluetooth link was asked to be dropped (0: not)

  void dropLinkSoon() { _drop_link_at = millis() | 1; }
  void stop(BaseSerialInterface* serial, uint8_t reason, int16_t err);
  int writeInfo(uint8_t* reply);
};
