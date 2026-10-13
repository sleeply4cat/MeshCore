#include "BLEGattsHook.h"

BLEGattsHook::Listener BLEGattsHook::_listeners[MAX_LISTENERS];
void* BLEGattsHook::_ctx[MAX_LISTENERS];
uint8_t BLEGattsHook::_count = 0;
esp_gatt_if_t BLEGattsHook::_gatts_if = ESP_GATT_IF_NONE;

void BLEGattsHook::add(Listener listener, void* ctx) {
  if (_count == 0) BLEDevice::setCustomGattsHandler(onEvent);
  if (_count < MAX_LISTENERS) {
    _listeners[_count] = listener;
    _ctx[_count] = ctx;
    _count++;
  }
}

void BLEGattsHook::onEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
  if (gatts_if != ESP_GATT_IF_NONE) _gatts_if = gatts_if;
  for (uint8_t i = 0; i < _count; i++) _listeners[i](event, gatts_if, param, _ctx[i]);
}
