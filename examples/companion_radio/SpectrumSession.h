#pragma once

#include <Arduino.h>
#include <helpers/BaseSerialInterface.h>
#include <helpers/radiolib/RadioLibWrappers.h>
#include "NodePrefs.h"

#define CMD_SPECTRUM               67
#define RESP_CODE_SPECTRUM_INFO    32     // a reply to CMD_SPECTRUM
#define PUSH_CODE_SPECTRUM_FRAME   0x92
#define PUSH_CODE_SPECTRUM_STOPPED 0x93

// Spectrum stream for a companion client: on request, the radio sweeps a band (SX126xSpectrumScanner) and the
// node streams the levels to the client to draw. The node borrows its radio from the mesh meanwhile
// (RadioLibWrapper::borrow()): the mesh neither receives nor sends.
//
// The client starts a session with OP_START and repeats it at least every KEEPALIVE_MS while it watches; an
// OP_START with other settings changes them. The session ends on OP_STOP, when no client is connected, after
// KEEPALIVE_MS without an OP_START (a USB link can't tell when its client has gone), or on a radio error, and the
// radio goes back to the mesh with the node's settings, as they are by then.
// Built for nRF52 boards with an SX1262 (SpectrumSession.cpp; -D SPECTRUM_STREAM=0 leaves it out): elsewhere
// CMD_SPECTRUM gets ERR_CODE_UNSUPPORTED_CMD.
//
// Frames start with CMD_SPECTRUM and an op; values are little-endian.
//
// OP_START (1), then:
//   u32 centre     Hz. 0: the node's frequency.
//   u32 span       Hz, at most 32MHz and at least 100Hz per point; the whole span within 150..960MHz.
//   u8  points     16..250, spread evenly: each point is the centre of its 1/points of the span.
//   u16 interval   ms between frames, 10..5000. 0: a frame per sweep (once across the span), as fast as the link
//                  takes them.
//   u8  gain       0: the radio's AGC; 1..13 a fixed gain step, lowest first (SX126xSpectrumScanner::setGain():
//                  no AGC shelf after strong signals; lower steps take stronger signals, with more noise).
//   The radio sweeps up and down the span through an RX bandwidth of ~2.5 points (4.8..467kHz), ~1.6ms per
//   point / RBW in kHz, and a frame holds the strongest level per point over the whole sweeps since the previous
//   one.
//   Reply: RESP_CODE_SPECTRUM_INFO; ERR_CODE_ILLEGAL_ARG for settings out of range, ERR_CODE_UNSUPPORTED_CMD
//   without an SX1262.
//
// OP_STOP (2). Reply: RESP_CODE_SPECTRUM_INFO with STATE_OFF.
//
// RESP_CODE_SPECTRUM_INFO, then:
//   u8 state (STATE_*: STARTING while the mesh finishes a packet), u8 config, u8 points, u32 first point (Hz),
//   u32 step (Hz), u32 RBW (Hz), u16 interval (ms), u8 gain
// PUSH_CODE_SPECTRUM_FRAME, then:
//   u8 config, u8 frame, u8 first point, a level per point: -2 * dBm (0.5dB steps).
//   A frame of more points than a companion frame holds (172) comes in several pushes of the same frame number,
//   which counts the frames sent (mod 256). 'config' changes with the sweep settings, so that frames of earlier
//   settings still on their way can be told apart.
// PUSH_CODE_SPECTRUM_STOPPED, then:
//   u8 reason (STOP_*), i16 RadioLib error (STOP_RADIO)
class SpectrumSession : public RadioBorrower {
public:
  enum { OP_START = 1, OP_STOP = 2 };
  enum { STATE_OFF = 0, STATE_STARTING = 1, STATE_RUNNING = 2 };
  enum { STOP_CLIENT = 0, STOP_TIMEOUT = 1, STOP_RADIO = 2 };
  enum { KEEPALIVE_MS = 15000, MAX_POINTS = 250 };

  explicit SpectrumSession(NodePrefs* prefs) : _prefs(prefs) { }

  // Handles a CMD_SPECTRUM frame: writes the reply into 'reply' and returns its length, or returns -ERR_CODE_*
  // for an error frame.
  int handleCommand(const uint8_t* frame, int len, uint8_t* reply);

  // From the main loop: starts the sweeps once the radio is free, sends frames, watches the client.
  void loop(BaseSerialInterface* serial);

  void returnRadio() override;

private:
  NodePrefs* _prefs;
  struct State;
  State* _s = NULL;   // while a session is on

  void stop(BaseSerialInterface* serial, uint8_t reason, int16_t err);
  int writeInfo(uint8_t* reply);
};
