#pragma once

#if defined(NRF52_PLATFORM)
  #include <helpers/nrf52/SerialBLEInterface.h>
#elif defined(ESP32)
  #include <helpers/esp32/SerialBLEInterface.h>
#endif

// GATT service for the spectrum page (SpectrumView): streams its sweeps to a BLE client, eg. the Web Bluetooth page
// examples/companion_radio/spectrum.html, and takes a few settings from it. It sits next to the companion's UART
// service, and needs the same pairing (PIN); with 2 connections (SerialBLEInterface::setMaxConnections()) the client
// connects next to the app. The same service on nRF52 and ESP32.
//
// All values little-endian.
//  info    (read, notify): the view's state, notified on every change (see SpectrumView::Stream)
//  data    (notify): frames, split into packets that fit each connection's MTU: u8 config generation,
//            u8 frame sequence, u8 first point, levels. Never waits for the link: a packet the stack can't
//            take drops the rest of that frame for that connection (a gap in the sequence).
//  control (write): u8 op, args (see SpectrumView::Stream)
#if defined(NRF52_PLATFORM)
class SpectrumBLEService : public BLEService {
#else
class SpectrumBLEService : BLECharacteristicCallbacks {
#endif
public:
  enum { INFO_LEN = 36, MAX_POINTS = 255 };

  struct Request {
    uint8_t op;
    uint32_t arg;   // 0xFFFFFFFF: none
  };

  SpectrumBLEService();
  // after the UART's begin(), before advertising starts
  void begin(SerialBLEInterface& uart);

  bool hasSubscribers();
  // sets the info value, and notifies it if it changed
  void setInfo(const uint8_t info[INFO_LEN]);
  // sends a frame to every connection with data notifications on, without waiting: false if any had no room
  bool sendFrame(uint8_t gen, const uint8_t* levels, uint8_t n);
  // next control request (written from the BLE stack's context), false if none
  bool takeRequest(Request& req);

private:
  enum { REQ_QUEUE = 8 };
  uint8_t _info_val[INFO_LEN];
  bool _info_unsent;   // a notification of it found no room (eg. behind frames)
  uint8_t _seq;
  Request _reqs[REQ_QUEUE];
  volatile uint8_t _req_head, _req_tail;

  void pushRequest(const uint8_t* data, uint16_t len);

#if defined(NRF52_PLATFORM)
  BLECharacteristic _info, _data, _ctrl;

  static void onControl(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
#else
  enum { MAX_CONN = 4 };
  BLEServer* _server;
  BLECharacteristic *_info, *_data, *_ctrl;
  BLE2902 *_info_cccd, *_data_cccd;

  // connections with notifications on, per characteristic (kept from the GATTS events, BLE task)
  volatile uint16_t _info_conn[MAX_CONN], _data_conn[MAX_CONN];
  volatile uint8_t _info_subs, _data_subs;

  void onWrite(BLECharacteristic* chr, esp_ble_gatts_cb_param_t* param) override;
  static void onGatts(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param, void* ctx);
  static void setSub(volatile uint16_t* conns, volatile uint8_t& n, uint16_t conn, bool on);
  bool notify(BLECharacteristic* chr, uint16_t conn, const uint8_t* data, uint16_t len);
#endif
};
