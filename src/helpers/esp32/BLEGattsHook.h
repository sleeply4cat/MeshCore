#pragma once

#include <BLEDevice.h>

// BLEDevice takes a single custom GATTS event handler: this one passes the events on to the companion's BLE
// services that need them (connection ids of writes and disconnects, the GATT interface for notifications to
// one connection), after BLEDevice's own handling. Header-only: not every variant builds helpers/esp32/*.cpp.
class BLEGattsHook {
public:
  typedef void (*Listener)(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param, void* ctx);

  static void add(Listener listener, void* ctx) {
    State& s = state();
    if (s.count == 0) BLEDevice::setCustomGattsHandler(onEvent);
    if (s.count < MAX_LISTENERS) {
      s.listeners[s.count] = listener;
      s.ctx[s.count] = ctx;
      s.count++;
    }
  }
  static esp_gatt_if_t gattsIf() { return state().gatts_if; }   // ESP_GATT_IF_NONE until the server registered

private:
  enum { MAX_LISTENERS = 4 };
  struct State {
    Listener listeners[MAX_LISTENERS];
    void* ctx[MAX_LISTENERS];
    uint8_t count = 0;
    esp_gatt_if_t gatts_if = ESP_GATT_IF_NONE;
  };
  static State& state() {
    static State s;   // one for the program, from whichever file uses it first
    return s;
  }

  static void onEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
    State& s = state();
    if (gatts_if != ESP_GATT_IF_NONE) s.gatts_if = gatts_if;
    for (uint8_t i = 0; i < s.count; i++) s.listeners[i](event, gatts_if, param, s.ctx[i]);
  }
};
