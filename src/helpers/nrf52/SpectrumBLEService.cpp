#include "SpectrumBLEService.h"

#define SPECTRUM_UUID_SERVICE  "e5a1f000-7c2e-4d6b-9b8a-3f1c2d4e5a60"
#define SPECTRUM_UUID_INFO     "e5a1f001-7c2e-4d6b-9b8a-3f1c2d4e5a60"
#define SPECTRUM_UUID_DATA     "e5a1f002-7c2e-4d6b-9b8a-3f1c2d4e5a60"
#define SPECTRUM_UUID_CONTROL  "e5a1f003-7c2e-4d6b-9b8a-3f1c2d4e5a60"

#define DATA_HEADER  3

SpectrumBLEService::SpectrumBLEService()
    : BLEService(SPECTRUM_UUID_SERVICE), _info(SPECTRUM_UUID_INFO), _data(SPECTRUM_UUID_DATA), _ctrl(SPECTRUM_UUID_CONTROL),
      _info_unsent(false), _seq(0), _req_head(0), _req_tail(0) {
  memset(_info_val, 0, sizeof(_info_val));
}

err_t SpectrumBLEService::begin() {
  VERIFY_STATUS(BLEService::begin());

  _info.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  _info.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_NO_ACCESS);
  _info.setFixedLen(INFO_LEN);
  VERIFY_STATUS(_info.begin());
  _info.write(_info_val, INFO_LEN);

  _data.setProperties(CHR_PROPS_NOTIFY);
  _data.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_NO_ACCESS);
  _data.setMaxLen(Bluefruit.getMaxMtu(BLE_GAP_ROLE_PERIPH) - 3);
  VERIFY_STATUS(_data.begin());

  _ctrl.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  _ctrl.setPermission(SECMODE_NO_ACCESS, SECMODE_ENC_WITH_MITM);
  _ctrl.setMaxLen(8);
  _ctrl.setWriteCallback(onControl);
  VERIFY_STATUS(_ctrl.begin());

  return ERROR_NONE;
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

bool SpectrumBLEService::takeRequest(Request& req) {
  if (_req_tail == _req_head) return false;
  req = _reqs[_req_tail];
  _req_tail = (_req_tail + 1) % REQ_QUEUE;
  return true;
}

void SpectrumBLEService::onControl(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_hdl;
  SpectrumBLEService& svc = (SpectrumBLEService&) chr->parentService();
  if (len == 0) return;
  uint8_t next = (svc._req_head + 1) % REQ_QUEUE;
  if (next == svc._req_tail) return;   // full: the UI loop isn't keeping up, drop it
  Request& r = svc._reqs[svc._req_head];
  r.op = data[0];
  r.arg = 0;
  for (int i = 1; i < len && i <= 4; i++) r.arg |= (uint32_t)data[i] << (8 * (i - 1));
  if (len == 1) r.arg = 0xFFFFFFFF;    // no argument
  svc._req_head = next;
}
