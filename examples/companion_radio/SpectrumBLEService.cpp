#if defined(BLE_PIN_CODE) && (defined(NRF52_PLATFORM) || defined(ESP32))

#include "SpectrumBLEService.h"
#ifdef ESP32
  #include <helpers/esp32/BLEGattsHook.h>
#endif

#define SPECTRUM_UUID_SERVICE  "e5a1f000-7c2e-4d6b-9b8a-3f1c2d4e5a60"
#define SPECTRUM_UUID_INFO     "e5a1f001-7c2e-4d6b-9b8a-3f1c2d4e5a60"
#define SPECTRUM_UUID_DATA     "e5a1f002-7c2e-4d6b-9b8a-3f1c2d4e5a60"
#define SPECTRUM_UUID_CONTROL  "e5a1f003-7c2e-4d6b-9b8a-3f1c2d4e5a60"

#define DATA_HEADER  3

bool SpectrumBLEService::takeRequest(Request& req) {
  if (_req_tail == _req_head) return false;
  req = _reqs[_req_tail];
  _req_tail = (_req_tail + 1) % REQ_QUEUE;
  return true;
}

void SpectrumBLEService::pushRequest(const uint8_t* data, uint16_t len) {
  if (len == 0) return;
  uint8_t next = (_req_head + 1) % REQ_QUEUE;
  if (next == _req_tail) return;   // full: the UI loop isn't keeping up, drop it
  Request& r = _reqs[_req_head];
  r.op = data[0];
  r.arg = 0;
  for (int i = 1; i < len && i <= 4; i++) r.arg |= (uint32_t)data[i] << (8 * (i - 1));
  if (len == 1) r.arg = 0xFFFFFFFF;   // no argument
  _req_head = next;
}

#if defined(NRF52_PLATFORM)

SpectrumBLEService::SpectrumBLEService()
    : BLEService(SPECTRUM_UUID_SERVICE), _info_unsent(false), _seq(0), _req_head(0), _req_tail(0),
      _info(SPECTRUM_UUID_INFO), _data(SPECTRUM_UUID_DATA), _ctrl(SPECTRUM_UUID_CONTROL) {
  memset(_info_val, 0, sizeof(_info_val));
}

void SpectrumBLEService::begin(SerialBLEInterface&) {
  if (BLEService::begin() != ERROR_NONE) return;

  _info.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  _info.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_NO_ACCESS);
  _info.setFixedLen(INFO_LEN);
  if (_info.begin() != ERROR_NONE) return;
  _info.write(_info_val, INFO_LEN);

  _data.setProperties(CHR_PROPS_NOTIFY);
  _data.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_NO_ACCESS);
  _data.setMaxLen(Bluefruit.getMaxMtu(BLE_GAP_ROLE_PERIPH) - 3);
  if (_data.begin() != ERROR_NONE) return;

  _ctrl.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  _ctrl.setPermission(SECMODE_NO_ACCESS, SECMODE_ENC_WITH_MITM);
  _ctrl.setMaxLen(8);
  _ctrl.setWriteCallback(onControl);
  _ctrl.begin();
}

bool SpectrumBLEService::hasSubscribers() {
  for (uint16_t h = 0; h < BLE_MAX_CONNECTION; h++) {
    BLEConnection* conn = Bluefruit.Connection(h);
    if (conn && conn->connected() && _data.notifyEnabled(h)) return true;
  }
  return false;
}

void SpectrumBLEService::setInfo(const uint8_t info[INFO_LEN]) {
  if (memcmp(info, _info_val, INFO_LEN) == 0 && !_info_unsent) return;
  memcpy(_info_val, info, INFO_LEN);
  _info.write(_info_val, INFO_LEN);
  _info_unsent = false;
  for (uint16_t h = 0; h < BLE_MAX_CONNECTION; h++) {
    BLEConnection* conn = Bluefruit.Connection(h);
    if (conn && conn->connected() && _info.notifyEnabled(h) && !_info.notify(h, _info_val, INFO_LEN)) _info_unsent = true;
  }
}

bool SpectrumBLEService::sendFrame(uint8_t gen, const uint8_t* levels, uint8_t n) {
  uint8_t pkt[DATA_HEADER + MAX_POINTS];
  bool all = true;
  _seq++;
  for (uint16_t h = 0; h < BLE_MAX_CONNECTION; h++) {
    BLEConnection* conn = Bluefruit.Connection(h);
    if (conn == NULL || !conn->connected() || !_data.notifyEnabled(h)) continue;
    int chunk = conn->getMtu() - 3 - DATA_HEADER;   // one notification each
    for (int i = 0; i < n; i += chunk) {
      uint16_t len = DATA_HEADER + (n - i < chunk ? n - i : chunk);
      pkt[0] = gen;
      pkt[1] = _seq;
      pkt[2] = i;
      memcpy(&pkt[DATA_HEADER], &levels[i], len - DATA_HEADER);
      // straight to the SoftDevice: BLECharacteristic::notify() waits up to 100ms for room in the queue,
      // which would hold up the sweeps
      ble_gatts_hvx_params_t hvx;
      memset(&hvx, 0, sizeof(hvx));
      hvx.handle = _data.handles().value_handle;
      hvx.type = BLE_GATT_HVX_NOTIFICATION;
      hvx.p_len = &len;
      hvx.p_data = pkt;
      if (sd_ble_gatts_hvx(h, &hvx) != NRF_SUCCESS) {   // queue full (or gone): drop the rest of this frame
        all = false;
        break;
      }
    }
  }
  return all;
}

void SpectrumBLEService::onControl(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_hdl;
  ((SpectrumBLEService&) chr->parentService()).pushRequest(data, len);
}

#else   // ESP32

SpectrumBLEService::SpectrumBLEService()
    : _info_unsent(false), _seq(0), _req_head(0), _req_tail(0), _server(NULL), _info(NULL), _data(NULL), _ctrl(NULL),
      _info_cccd(NULL), _data_cccd(NULL), _info_subs(0), _data_subs(0) {
  memset(_info_val, 0, sizeof(_info_val));
}

void SpectrumBLEService::begin(SerialBLEInterface& uart) {
  BLEServer* server = uart.getServer();
  BLEService* svc = server->createService(BLEUUID(SPECTRUM_UUID_SERVICE), 16);

  _info = svc->createCharacteristic(SPECTRUM_UUID_INFO, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  _info->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM);
  _info_cccd = new BLE2902();
  _info->addDescriptor(_info_cccd);
  _info->setValue(_info_val, INFO_LEN);

  _data = svc->createCharacteristic(SPECTRUM_UUID_DATA, BLECharacteristic::PROPERTY_NOTIFY);
  _data->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM);
  _data_cccd = new BLE2902();
  _data->addDescriptor(_data_cccd);

  _ctrl = svc->createCharacteristic(SPECTRUM_UUID_CONTROL, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  _ctrl->setAccessPermissions(ESP_GATT_PERM_WRITE_ENC_MITM);
  _ctrl->setCallbacks(this);

  svc->start();
  BLEGattsHook::add(onGatts, this);
  _server = server;
}

void SpectrumBLEService::setSub(volatile uint16_t* conns, volatile uint8_t& n, uint16_t conn, bool on) {
  int at = -1;
  for (int i = 0; i < n; i++) if (conns[i] == conn) at = i;
  if (on && at < 0 && n < MAX_CONN) {
    conns[n] = conn;
    n = n + 1;
  } else if (!on && at >= 0) {
    conns[at] = conns[n - 1];
    n = n - 1;
  }
}

// notification writes to the CCCDs, and disconnects: who gets notifications (BLE2902 keeps one value for all)
void SpectrumBLEService::onGatts(esp_gatts_cb_event_t event, esp_gatt_if_t, esp_ble_gatts_cb_param_t* param, void* ctx) {
  SpectrumBLEService* svc = (SpectrumBLEService*) ctx;
  if (event == ESP_GATTS_WRITE_EVT && param->write.len >= 2) {
    bool on = param->write.value[0] & 1;
    if (param->write.handle == svc->_data_cccd->getHandle()) {
      setSub(svc->_data_conn, svc->_data_subs, param->write.conn_id, on);
    } else if (param->write.handle == svc->_info_cccd->getHandle()) {
      setSub(svc->_info_conn, svc->_info_subs, param->write.conn_id, on);
      if (on) svc->_info_unsent = true;   // a fresh subscriber: send the state
    }
  } else if (event == ESP_GATTS_DISCONNECT_EVT) {
    setSub(svc->_data_conn, svc->_data_subs, param->disconnect.conn_id, false);
    setSub(svc->_info_conn, svc->_info_subs, param->disconnect.conn_id, false);
  }
}

bool SpectrumBLEService::notify(BLECharacteristic* chr, uint16_t conn, const uint8_t* data, uint16_t len) {
  // straight to the stack, to this connection only, and without waiting for room
  return esp_ble_gatts_send_indicate(BLEGattsHook::gattsIf(), conn, chr->getHandle(), len, (uint8_t*) data, false) == ESP_OK;
}

bool SpectrumBLEService::hasSubscribers() {
  return _data_subs > 0;
}

void SpectrumBLEService::setInfo(const uint8_t info[INFO_LEN]) {
  if (_server == NULL) return;
  if (memcmp(info, _info_val, INFO_LEN) == 0 && !_info_unsent) return;
  memcpy(_info_val, info, INFO_LEN);
  _info->setValue(_info_val, INFO_LEN);
  _info_unsent = false;
  for (int i = 0; i < _info_subs; i++) {
    if (!notify(_info, _info_conn[i], _info_val, INFO_LEN)) _info_unsent = true;
  }
}

bool SpectrumBLEService::sendFrame(uint8_t gen, const uint8_t* levels, uint8_t n) {
  uint8_t pkt[DATA_HEADER + MAX_POINTS];
  bool all = true;
  _seq++;
  for (int c = 0; c < _data_subs; c++) {
    uint16_t conn = _data_conn[c];
    int mtu = _server->getPeerMTU(conn);
    int chunk = (mtu > 23 ? mtu : 23) - 3 - DATA_HEADER;   // one notification each
    for (int i = 0; i < n; i += chunk) {
      uint16_t len = DATA_HEADER + (n - i < chunk ? n - i : chunk);
      pkt[0] = gen;
      pkt[1] = _seq;
      pkt[2] = i;
      memcpy(&pkt[DATA_HEADER], &levels[i], len - DATA_HEADER);
      if (!notify(_data, conn, pkt, len)) {   // no room (or gone): drop the rest of this frame
        all = false;
        break;
      }
    }
  }
  return all;
}

void SpectrumBLEService::onWrite(BLECharacteristic* chr, esp_ble_gatts_cb_param_t* param) {
  pushRequest(chr->getData(), chr->getLength());
}

#endif
#endif
