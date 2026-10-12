#pragma once

#include <Arduino.h>
#include <helpers/BaseSerialInterface.h>
#include "NodePrefs.h"

// LoRa channel scan for a companion client: finds LoRa traffic over a band by channel activity detection (CAD),
// and listens on a candidate channel to tell what it is. The mesh is paused while it runs, and the radio goes back
// to the node's settings afterwards. Needs an SX1262: elsewhere the command gets ERR_CODE_UNSUPPORTED_CMD.
//
// CAD looks for LoRa chirps of one spreading factor and bandwidth; it works below the noise floor, where an RSSI
// sweep sees nothing. A scan goes over the band point by point, for each bandwidth and SF in turn, pass after pass,
// and reports each detection; the client keeps the counts (detections per pass) and picks the candidates. One CAD
// takes about (symbols + 1) symbol times plus a few ms: a pass over 1 MHz in steps of 31.25 kHz at BW 62.5 takes
// ~0.34 s at SF7 and ~1.6 s at SF10 (2-symbol CAD).
//
// Frames start with CMD_LORA_SCAN and an op; values are little-endian. Bandwidths are given by index into
// BANDWIDTHS (7.8 ... 500 kHz).
//
// OP_SCAN (1), then:
//   u32 from, u32 to     Hz, the band (both within 150..960 MHz)
//   u32 step             Hz between points, at least 1 kHz; 0: half the bandwidth (CAD hears up to ~BW/2 off);
//                        1: the bandwidth (twice as fast, the place known to ~BW/2)
//   u16 bandwidths       bit k: BANDWIDTHS[k]
//   u8  sfs              bit k: SF 5 + k
//   u8  symbols          CAD length: 1, 2, 4, 8 or 16 symbols (0: 2)
//   u8  peak             detPeak = SF + this (0: 17 for 1-2 symbols, 13 for more; higher: fewer false alarms,
//                        less sensitive)
//   u8  min              detMin (0: 10)
//   u16 duration         s, 1..3600
//   Reply: RESP_CODE_OK, or ERR_CODE_ILLEGAL_ARG. Then PUSH_LORA_SCAN_HIT per detection, PUSH_LORA_SCAN_PASS after
//   each pass, PUSH_LORA_SCAN_DONE at the end.
// OP_LISTEN (2), then:
//   u32 freq (Hz), u8 bandwidth (index), u8 SF, u8 sync word (0x12: MeshCore), u16 duration (s, 1..3600)
//   Reply: RESP_CODE_OK. Then PUSH_LORA_SCAN_PACKET per packet received, PUSH_LORA_SCAN_DONE at the end.
// OP_STOP (3): ends a scan or a listen early. Reply: RESP_CODE_OK.
// OP_SCAN_HOT (4), while a scan is on: hot points, which the scan goes round for a while after each pass over the
// band, eg. around the best candidates, so as to see them often and catch their packets. Replaces the list:
//   u16 time (ms after each pass; 0: none), u8 flags (1: receive after a detection), u8 sync word, u8 symbols
//   to wait for a header (0: 40), u8 n (0..MAX_HOT), n x (u32 freq, u8 bandwidth, u8 SF)
//   Reply: RESP_CODE_OK, or ERR_CODE_BAD_STATE when no scan is on. Detections on hot points come with the hot
//   flag; packets received after them as PUSH_LORA_SCAN_PACKET.
//
// PUSH_LORA_SCAN_HIT: u32 freq (Hz), u8 bandwidth, u8 SF, u8 hot (1: on a hot point)
// PUSH_LORA_SCAN_PASS: u16 passes over the band so far
// PUSH_LORA_SCAN_PACKET: u32 freq (Hz, as tuned), i32 frequency error (Hz: tuned - sender's), i8 RSSI (dBm),
//   i8 SNR (dB x 4), u8 coding rate (5..8, from the header), u8 flags (1: CRC good, 2: the header announced a CRC),
//   u8 bandwidth, u8 SF, u8 length, the payload (as much as fits)
// PUSH_LORA_SCAN_DONE: u8 op, u8 reason (DONE_*), i16 RadioLib error (DONE_RADIO), u16 passes or packets,
//   u16 preambles detected (listen), u16 header errors (listen)
class LoraScan {
public:
  enum { CMD_LORA_SCAN = 67, OP_SCAN = 1, OP_LISTEN = 2, OP_STOP = 3, OP_SCAN_HOT = 4 };
  enum { MAX_HOT = 24 };
  enum { PUSH_LORA_SCAN_HIT = 0x91, PUSH_LORA_SCAN_PASS = 0x92, PUSH_LORA_SCAN_PACKET = 0x93, PUSH_LORA_SCAN_DONE = 0x94 };
  enum { DONE_COMPLETE = 0, DONE_STOPPED = 1, DONE_DISCONNECTED = 2, DONE_RADIO = 3 };
  static const float BANDWIDTHS[10];

  explicit LoraScan(NodePrefs* prefs) : _prefs(prefs) { }

  // a scan or listen is on (maybe waiting for the radio): loop() has work to do
  bool isActive() const { return _op != 0; }
  // the radio is scanning or listening: the mesh must leave it alone
  bool ownsRadio() const { return _op != 0 && _owns_radio; }

  // Handles a CMD_LORA_SCAN frame: 0 for RESP_CODE_OK, else an ERR_CODE_*.
  int handleCommand(const uint8_t* frame, int len);

  // From the main loop: takes the radio once the mesh is idle, then does one CAD (or checks for a packet).
  void loop(BaseSerialInterface* serial);

private:
  NodePrefs* _prefs;
  uint8_t _op = 0;           // OP_SCAN or OP_LISTEN while on
  bool _owns_radio = false;
  bool _stop_requested = false;
  uint32_t _started, _duration_ms;
  int16_t _err = 0;          // RadioLib's, setting up a listen

  // scan
  uint32_t _from, _to, _step;
  uint16_t _bws;
  uint8_t _sfs, _symbols, _peak, _min;
  uint8_t _bw_i, _sf, _cur_bw = 0xFF, _cur_sf = 0;
  uint32_t _cur_freq = 0;    // as tuned
  uint32_t _freq;
  uint16_t _passes;

  // hot points
  uint32_t _hot_freq[MAX_HOT];
  uint8_t _hot_bw[MAX_HOT], _hot_sf[MAX_HOT], _hot_n = 0, _hot_i;
  uint16_t _hot_ms = 0;
  bool _hot_rx = false, _in_hot = false, _sync_set = false;
  uint8_t _hot_sync, _hot_symbols;
  uint32_t _hot_until;
  uint32_t _band_freq;       // where the band pass goes on after the hot points
  uint8_t _band_bw, _band_sf;

  // listen
  uint8_t _sync;
  uint16_t _packets, _preambles, _header_errors;

  void begin();
  void firstPoint();
  bool nextPoint();
  void scanStep(BaseSerialInterface* serial);
  int16_t tune(uint32_t freq, uint8_t bw, uint8_t sf);
  void receiveAfterCad(BaseSerialInterface* serial);
  void pushPacket(BaseSerialInterface* serial, uint32_t irq);
  void listenStep(BaseSerialInterface* serial);
  void finish(BaseSerialInterface* serial, uint8_t reason, int16_t err);
};
