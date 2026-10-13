#pragma once

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

// GATT service for the companion's spectrum view: streams its sweeps to a BLE client, eg. the Web Bluetooth page
// examples/companion_radio/spectrum.html, and takes a few settings from it (the same service as on nRF52). It sits
// next to the companion's UART service on the same server, and needs the same pairing (PIN); with
// BLE_MAX_CONNECTIONS > 1 the client connects next to the app.
//
// All values little-endian.
//  info    (read, notify): the view's state, notified on every change (see SpectrumView::Stream)
//  data    (notify): frames, split into packets that fit each connection's MTU: u8 config generation,
//            u8 frame sequence, u8 first point, levels. Never waits for the link: a packet the stack can't
//            take drops the rest of that frame for that connection (a gap in the sequence).
//  control (write): u8 op, args (see SpectrumView::Stream)
class SpectrumBLEService : BLECharacteristicCallbacks {
public:
  enum { INFO_LEN = 36, MAX_POINTS = 255 };

  struct Request {
    uint8_t op;
    uint32_t arg;   // 0xFFFFFFFF: none
  };

  // on the UART's server, after its service is created and before advertising starts
  void begin(BLEServer* server);

  bool hasSubscribers() const { return _data_subs > 0; }
  // sets the info value, and notifies it if it changed
  void setInfo(const uint8_t info[INFO_LEN]);
  // sends a frame to every connection with data notifications on, without waiting: false if any had no room
  bool sendFrame(uint8_t gen, const uint8_t* levels, uint8_t n);
  // next control request (written from the BLE task), false if none
  bool takeRequest(Request& req);

private:
  enum { MAX_CONN = 4, REQ_QUEUE = 8 };
  BLEServer* _server = NULL;
  BLECharacteristic *_info = NULL, *_data = NULL, *_ctrl = NULL;
  BLE2902 *_info_cccd = NULL, *_data_cccd = NULL;
  uint8_t _info_val[INFO_LEN];
  bool _info_unsent = false;
  uint8_t _seq = 0;

  // connections with notifications on, per characteristic (kept from the GATTS events, BLE task)
  volatile uint16_t _info_conn[MAX_CONN], _data_conn[MAX_CONN];
  volatile uint8_t _info_subs = 0, _data_subs = 0;

  Request _reqs[REQ_QUEUE];
  volatile uint8_t _req_head = 0, _req_tail = 0;

  void onWrite(BLECharacteristic* chr, esp_ble_gatts_cb_param_t* param) override;
  static void onGatts(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param, void* ctx);
  static void setSub(volatile uint16_t* conns, volatile uint8_t& n, uint16_t conn, bool on);
  bool notify(BLECharacteristic* chr, uint16_t conn, const uint8_t* data, uint16_t len);
};
