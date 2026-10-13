#pragma once

#include <BLEDevice.h>

// BLEDevice takes a single custom GATTS event handler: this one passes the events on to the companion's BLE
// services that need them (connection ids of writes and disconnects, the GATT interface for notifications to
// one connection), after BLEDevice's own handling.
class BLEGattsHook {
public:
  typedef void (*Listener)(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param, void* ctx);

  static void add(Listener listener, void* ctx);
  static esp_gatt_if_t gattsIf() { return _gatts_if; }   // ESP_GATT_IF_NONE until the server registered

private:
  enum { MAX_LISTENERS = 4 };
  static Listener _listeners[MAX_LISTENERS];
  static void* _ctx[MAX_LISTENERS];
  static uint8_t _count;
  static esp_gatt_if_t _gatts_if;

  static void onEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);
};
