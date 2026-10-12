#include "LoraScan.h"

const float LoraScan::BANDWIDTHS[10] = { 7.8f, 10.4f, 15.6f, 20.8f, 31.25f, 41.7f, 62.5f, 125.0f, 250.0f, 500.0f };

#if LORA_SCAN

#include <type_traits>
#include <target.h>

extern RADIO_CLASS radio;   // in every variant's target.cpp

#define ERR_CODE_UNSUPPORTED_CMD  1   // as in MyMesh.cpp
#define ERR_CODE_BAD_STATE        4
#define ERR_CODE_ILLEGAL_ARG      6

// The radio as an SX1262, or NULL for other radios: the scan compiles with all of them, and only works there.
template <class R> static SX1262* asSX1262(R& r, std::true_type) { return &r; }
template <class R> static SX1262* asSX1262(R&, std::false_type) { return NULL; }
static SX1262* sx1262() { return asSX1262(radio, std::is_base_of<SX1262, RADIO_CLASS>{}); }

static void put16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void put32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static uint16_t get16(const uint8_t* p) { return p[0] | p[1] << 8; }
static uint32_t get32(const uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

static bool validFreq(uint32_t hz) { return hz >= 150000000 && hz <= 960000000; }

static uint8_t cadSymbols(uint8_t n) {
  switch (n) {
    case 1: return RADIOLIB_SX126X_CAD_ON_1_SYMB;
    case 4: return RADIOLIB_SX126X_CAD_ON_4_SYMB;
    case 8: return RADIOLIB_SX126X_CAD_ON_8_SYMB;
    case 16: return RADIOLIB_SX126X_CAD_ON_16_SYMB;
    default: return RADIOLIB_SX126X_CAD_ON_2_SYMB;
  }
}

int LoraScan::handleCommand(const uint8_t* frame, int len) {
  if (len < 2) return ERR_CODE_ILLEGAL_ARG;
  uint8_t op = frame[1];
  const uint8_t* a = &frame[2];
  if (op == OP_STOP) {
    if (_op) _stop_requested = true;
    return 0;
  }
  if (sx1262() == NULL || op < OP_SCAN || op > OP_SCAN_HOT) return ERR_CODE_UNSUPPORTED_CMD;

  if (op == OP_SCAN_HOT) {   // for the scan on
    if (_op != OP_SCAN) return ERR_CODE_BAD_STATE;
    if (len < 2 + 6) return ERR_CODE_ILLEGAL_ARG;
    uint8_t n = a[5];
    if (n > MAX_HOT || len < 2 + 6 + n * 6) return ERR_CODE_ILLEGAL_ARG;
    for (int k = 0; k < n; k++) {
      const uint8_t* p = &a[6 + k * 6];
      if (!validFreq(get32(p)) || p[4] > 9 || p[5] < 5 || p[5] > 12) return ERR_CODE_ILLEGAL_ARG;
    }
    for (int k = 0; k < n; k++) {
      const uint8_t* p = &a[6 + k * 6];
      _hot_freq[k] = get32(p);
      _hot_bw[k] = p[4];
      _hot_sf[k] = p[5];
    }
    _hot_ms = get16(a);
    _hot_rx = a[2] & 1;
    _hot_sync = a[3];
    _hot_symbols = a[4] ? a[4] : 40;
    _hot_n = n;
    _hot_i = 0;
    _sync_set = false;
    return 0;
  }
  if (_op) return ERR_CODE_BAD_STATE;   // one at a time: OP_STOP first

  if (op == OP_SCAN) {
    if (len < 2 + 20) return ERR_CODE_ILLEGAL_ARG;
    uint32_t from = get32(a), to = get32(a + 4), step = get32(a + 8);
    uint16_t bws = get16(a + 12) & 0x3FF, duration = get16(a + 18);
    uint8_t sfs = a[14], symbols = a[15] ? a[15] : 2;
    if (!validFreq(from) || !validFreq(to) || to < from || (step > 1 && step < 1000)) return ERR_CODE_ILLEGAL_ARG;
    if (bws == 0 || sfs == 0 || duration == 0 || duration > 3600) return ERR_CODE_ILLEGAL_ARG;
    if (symbols != 1 && symbols != 2 && symbols != 4 && symbols != 8 && symbols != 16) return ERR_CODE_ILLEGAL_ARG;
    _from = from; _to = to; _step = step; _bws = bws; _sfs = sfs; _symbols = symbols;
    _peak = a[16] ? a[16] : (symbols <= 2 ? 17 : 13);
    _min = a[17] ? a[17] : 10;
    _duration_ms = duration * 1000UL;
    _hot_n = 0;
  } else {
    if (len < 2 + 9) return ERR_CODE_ILLEGAL_ARG;
    uint32_t freq = get32(a);
    uint16_t duration = get16(a + 7);
    if (!validFreq(freq) || a[4] > 9 || a[5] < 5 || a[5] > 12 || duration == 0 || duration > 3600) return ERR_CODE_ILLEGAL_ARG;
    _freq = freq; _bw_i = a[4]; _sf = a[5]; _sync = a[6];
    _duration_ms = duration * 1000UL;
  }
  _op = op;
  _owns_radio = false;   // taken in loop(), once the mesh has finished with it
  _stop_requested = false;
  return 0;
}

void LoraScan::firstPoint() {
  _bw_i = __builtin_ctz(_bws);
  _sf = 5 + __builtin_ctz(_sfs);
  _freq = _from;
}

// the next point: frequency, then SF, then bandwidth. true at the end of a pass.
bool LoraScan::nextPoint() {
  uint32_t step = _step > 1 ? _step : (uint32_t)(BANDWIDTHS[_bw_i] * (_step == 1 ? 1000 : 500));
  _freq += step;
  if (_freq <= _to) return false;
  _freq = _from;
  for (uint8_t sf = _sf + 1; sf <= 12; sf++) {
    if (_sfs & (1 << (sf - 5))) { _sf = sf; return false; }
  }
  _sf = 5 + __builtin_ctz(_sfs);
  for (uint8_t bw = _bw_i + 1; bw < 10; bw++) {
    if (_bws & (1 << bw)) { _bw_i = bw; return false; }
  }
  _bw_i = __builtin_ctz(_bws);
  return true;
}

void LoraScan::begin() {
  SX1262* r = sx1262();
  r->standby();
  _started = millis();
  _cur_bw = 0xFF;
  _cur_sf = 0;
  _passes = _packets = _preambles = _header_errors = 0;
  _in_hot = false;
  _sync_set = false;
  _err = r->invertIQ(false);
  if (_err == RADIOLIB_ERR_NONE) _err = r->setPreambleLength(8);   // the shortest a sender may use
  if (_op == OP_SCAN) {
    firstPoint();
    return;
  }
  int16_t e = _err;
  if (e == RADIOLIB_ERR_NONE) e = tune(_freq, _bw_i, _sf);
  if (e == RADIOLIB_ERR_NONE) e = r->setSyncWord(_sync);
  if (e == RADIOLIB_ERR_NONE) {
    e = r->SX126x::startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF,
                                RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED),
                                RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
  }
  _err = e;
}

int16_t LoraScan::tune(uint32_t freq, uint8_t bw, uint8_t sf) {
  SX1262* r = sx1262();
  int16_t e = RADIOLIB_ERR_NONE;
  if (bw != _cur_bw) {
    e = r->setBandwidth(BANDWIDTHS[bw]);
    _cur_bw = e == RADIOLIB_ERR_NONE ? bw : 0xFF;
  }
  if (e == RADIOLIB_ERR_NONE && sf != _cur_sf) {
    e = r->setSpreadingFactor(sf);
    _cur_sf = e == RADIOLIB_ERR_NONE ? sf : 0;
  }
  if (e == RADIOLIB_ERR_NONE) e = r->setFrequency(freq / 1e6f);
  _cur_freq = freq;
  return e;
}

void LoraScan::scanStep(BaseSerialInterface* serial) {
  SX1262* r = sx1262();
  if (_in_hot && (_hot_n == 0 || (int32_t)(millis() - _hot_until) >= 0)) {   // back to the band
    _in_hot = false;
    _freq = _band_freq; _bw_i = _band_bw; _sf = _band_sf;
  }
  uint32_t freq = _in_hot ? _hot_freq[_hot_i] : _freq;
  uint8_t bw = _in_hot ? _hot_bw[_hot_i] : _bw_i, sf = _in_hot ? _hot_sf[_hot_i] : _sf;
  int16_t e = tune(freq, bw, sf);
  if (e == RADIOLIB_ERR_NONE) {
    ChannelScanConfig_t cfg = { .cad = { cadSymbols(_symbols), (uint8_t)(sf + _peak), _min,
                                         RADIOLIB_SX126X_CAD_PARAM_DEFAULT, 0, RADIOLIB_IRQ_CAD_DEFAULT_FLAGS,
                                         RADIOLIB_IRQ_CAD_DEFAULT_MASK } };
    e = r->scanChannel(cfg);
  }
  if (e == RADIOLIB_LORA_DETECTED) {
    uint8_t f[8] = { PUSH_LORA_SCAN_HIT };
    put32(&f[1], freq);
    f[5] = bw;
    f[6] = sf;
    f[7] = _in_hot;
    serial->writeFrame(f, sizeof(f));
    if (_in_hot && _hot_rx) receiveAfterCad(serial);
  } else if (e != RADIOLIB_CHANNEL_FREE) {
    finish(serial, DONE_RADIO, e);
    return;
  }
  if (_in_hot) {
    _hot_i = (_hot_i + 1) % _hot_n;
  } else if (nextPoint()) {
    _passes++;
    uint8_t f[3] = { PUSH_LORA_SCAN_PASS };
    put16(&f[1], _passes);
    serial->writeFrame(f, sizeof(f));
    if (_hot_n && _hot_ms) {   // the hot points for a while, then the next pass
      _band_freq = _freq; _band_bw = _bw_i; _band_sf = _sf;
      _in_hot = true;
      _hot_until = millis() + _hot_ms;
    }
  }
}

// CAD found chirps on a hot point: receive there for a while, in case it was a preamble (a header comes then)
void LoraScan::receiveAfterCad(BaseSerialInterface* serial) {
  SX1262* r = sx1262();
  if (!_sync_set) {
    if (r->setSyncWord(_hot_sync) != RADIOLIB_ERR_NONE) return;
    _sync_set = true;
  }
  uint32_t symbol_us = (uint32_t)((1UL << _cur_sf) * 1000.0f / BANDWIDTHS[_cur_bw]);
  uint32_t raw = (uint32_t)((uint64_t)_hot_symbols * symbol_us * 64 / 1000);   // in 15.625 us
  if (r->SX126x::startReceive(raw, RADIOLIB_IRQ_RX_DEFAULT_FLAGS, RADIOLIB_IRQ_RX_DEFAULT_MASK, 0) != RADIOLIB_ERR_NONE) return;
  // the timer stops at a header; then the packet may take up to ~300 symbols
  uint32_t start = millis(), limit = _hot_symbols * symbol_us / 1000 + 20;
  for (;;) {
    uint32_t irq = r->getIrqFlags();
    if (irq & RADIOLIB_SX126X_IRQ_RX_DONE) {
      pushPacket(serial, irq);
      break;
    }
    if (irq & (RADIOLIB_SX126X_IRQ_TIMEOUT | RADIOLIB_SX126X_IRQ_HEADER_ERR)) break;
    if (irq & RADIOLIB_SX126X_IRQ_HEADER_VALID) limit = 300 * symbol_us / 1000 + 50;
    if (millis() - start > limit) break;
  }
  r->standby();
}

void LoraScan::pushPacket(BaseSerialInterface* serial, uint32_t irq) {
  SX1262* r = sx1262();
  uint8_t buf[256];
  size_t len = r->getPacketLength();
  int16_t e = r->readData(buf, len);
  uint8_t cr = 0;
  bool has_crc = false;
  r->getLoRaRxHeaderInfo(&cr, &has_crc);
  uint8_t f[MAX_FRAME_SIZE];
  int i = 0;
  f[i++] = PUSH_LORA_SCAN_PACKET;
  put32(&f[i], _cur_freq); i += 4;
  put32(&f[i], (uint32_t)(int32_t)r->getFrequencyError()); i += 4;
  f[i++] = (uint8_t)(int8_t)r->getRSSI();
  f[i++] = (uint8_t)(int8_t)(r->getSNR() * 4);
  f[i++] = cr + 4;
  f[i++] = (e == RADIOLIB_ERR_NONE && !(irq & RADIOLIB_SX126X_IRQ_CRC_ERR) ? 1 : 0) | (has_crc ? 2 : 0);
  f[i++] = _cur_bw;
  f[i++] = _cur_sf;
  f[i++] = len;
  int n = len < (size_t)(MAX_FRAME_SIZE - i) ? len : MAX_FRAME_SIZE - i;
  memcpy(&f[i], buf, n);
  serial->writeFrame(f, i + n);
  _packets++;
}

void LoraScan::listenStep(BaseSerialInterface* serial) {
  SX1262* r = sx1262();
  uint32_t irq = r->getIrqFlags();
  if (irq & RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED) {
    _preambles++;
    r->clearIrqFlags(RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED);
  }
  if (irq & RADIOLIB_SX126X_IRQ_HEADER_ERR) {
    _header_errors++;
    r->clearIrqFlags(RADIOLIB_SX126X_IRQ_HEADER_ERR);
  }
  if (!(irq & RADIOLIB_SX126X_IRQ_RX_DONE)) return;
  pushPacket(serial, irq);
  r->SX126x::startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF,
                          RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED),
                          RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
}

void LoraScan::finish(BaseSerialInterface* serial, uint8_t reason, int16_t err) {
  uint8_t op = _op;
  if (_owns_radio) {
    sx1262()->standby();
    radio_init();   // resets the chip
    radio_driver.begin();
    radio_driver.setParams(_prefs->freq, _prefs->bw, _prefs->sf, _prefs->cr);
    radio_driver.setTxPower(_prefs->tx_power_dbm);
    radio_driver.setRxBoostedGainMode(_prefs->rx_boosted_gain);
  }
  _op = 0;
  _owns_radio = false;
  _hot_n = 0;
  if (serial->isConnected()) {
    uint8_t f[12] = { PUSH_LORA_SCAN_DONE, op, reason };
    put16(&f[3], err);
    put16(&f[5], op == OP_LISTEN ? _packets : _passes);
    put16(&f[7], _preambles);
    put16(&f[9], _header_errors);
    serial->writeFrame(f, 11);
  }
}

void LoraScan::loop(BaseSerialInterface* serial) {
  if (_op == 0) return;
  if (!serial->isConnected()) {
    finish(serial, DONE_DISCONNECTED, 0);
    return;
  }
  if (!_owns_radio) {
    if (_stop_requested) {   // before it started: nothing to give back
      finish(serial, DONE_STOPPED, 0);
      return;
    }
    if (!radio_driver.isInRecvMode() || radio_driver.isReceiving()) return;   // the mesh is busy with a packet
    _owns_radio = true;
    begin();
  }
  if (_err != RADIOLIB_ERR_NONE) {
    finish(serial, DONE_RADIO, _err);
    return;
  }
  if (_stop_requested) {
    finish(serial, DONE_STOPPED, 0);
    return;
  }
  if (millis() - _started >= _duration_ms) {
    finish(serial, DONE_COMPLETE, 0);
    return;
  }
  if (serial->isWriteBusy()) return;   // let the link catch up: no detections dropped
  if (_op == OP_LISTEN) listenStep(serial);
  else scanStep(serial);
}

#else   // a build without the scan

int LoraScan::handleCommand(const uint8_t*, int) { return 1; }   // ERR_CODE_UNSUPPORTED_CMD
void LoraScan::loop(BaseSerialInterface*) { }

#endif
