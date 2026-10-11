#pragma once

#include <bluefruit.h>

// GATT service for the companion's spectrum view (ui-new SpectrumScreen): streams its sweeps to a BLE client,
// eg. the Web Bluetooth page examples/companion_radio/spectrum.html, and takes a few settings from it. It sits
// next to the companion's Nordic UART service, and the client can use it alongside the app (BLE_MAX_CONNECTIONS):
// it needs the same pairing (PIN) as the app.
//
// All values little-endian.
//  info    (read, notify): the view's state, notified on every change
//            u8 version (1), u8 state (STATE_*), u8 span index, u8 points,
//            u32 first point's frequency (Hz), u32 step (Hz), u32 RBW (Hz), u32 centre (Hz),
//            u32 node frequency (Hz), u32 node bandwidth (Hz), i16 radio error, u8 config generation,
//            u8 flags (FLAG_*), u16 frame interval (ms, 0: every sweep), u8 aggregation (AGG_*), u8 reserved
//  data    (notify): frames, one per interval: each point aggregated over the sweeps since the previous frame
//            (AGG_*), in -0.5dBm steps (-2 * dBm, as the radio's RSSI; 255 = no data yet). A frame is split into
//            packets that fit the connection's MTU: u8 config generation, u8 frame sequence, u8 first point, levels.
//            Never waits for the link: a frame that finds the connection's queue full is dropped (a gap in the
//            sequence).
//  control (write): u8 op, args
//            CTRL_START [u8 span], CTRL_STOP, CTRL_SPAN u8 span, CTRL_CENTRE u32 Hz (0: follow the node's channel),
//            CTRL_INTERVAL u16 ms (0: every sweep, else 10..2000), CTRL_AGG u8 (AGG_*)
class SpectrumBLEService : public BLEService {
public:
  enum { STATE_OFF = 0, STATE_STARTING = 1, STATE_RUNNING = 2 };
  enum { FLAG_FOLLOW = 1 };   // the centre follows the node's channel
  enum { CTRL_START = 1, CTRL_STOP = 2, CTRL_SPAN = 3, CTRL_CENTRE = 4, CTRL_INTERVAL = 5, CTRL_AGG = 6 };
  enum { AGG_MAX = 0, AGG_MEAN = 1, AGG_LAST = 2 };   // strongest, mean power, or just the latest sweep
  enum { INFO_LEN = 36, MAX_POINTS = 255 };

  struct Request {
    uint8_t op;
    uint32_t arg;
  };

  SpectrumBLEService();
  err_t begin() override;

  // true if any connection has data notifications on
  bool hasSubscribers();
  // sets the info value, and notifies it if it changed
  void setInfo(const uint8_t info[INFO_LEN]);
  // sends a frame to every subscribed connection, without waiting: false if any of them had no room for it
  bool sendFrame(uint8_t gen, const uint8_t* levels, uint8_t n);
  // next control request (written from the BLE task), false if none
  bool takeRequest(Request& req);

private:
  BLECharacteristic _info, _data, _ctrl;
  uint8_t _info_val[INFO_LEN];
  bool _info_unsent;   // a notification of it found the queue full (of frames)
  uint8_t _seq;

  enum { REQ_QUEUE = 8 };
  Request _reqs[REQ_QUEUE];
  volatile uint8_t _req_head, _req_tail;

  static void onControl(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
};
