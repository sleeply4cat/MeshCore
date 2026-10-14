#include "SpectrumSession.h"
#include <target.h>

#ifndef SPECTRUM_STREAM
  #if defined(NRF52_PLATFORM) && defined(USE_SX1262)
    #define SPECTRUM_STREAM 1
  #else
    #define SPECTRUM_STREAM 0
  #endif
#endif

#if SPECTRUM_STREAM

#include <new>
#include <type_traits>
#include <helpers/radiolib/SX126xSpectrumScanner.h>

extern RADIO_CLASS radio;   // in every variant's target.cpp

#define ERR_CODE_UNSUPPORTED_CMD  1   // as in MyMesh.cpp
#define ERR_CODE_BAD_STATE        4
#define ERR_CODE_ILLEGAL_ARG      6

// The radio as an SX1262, or NULL for other radios (some variants define USE_SX1262 for all their builds).
template <class R> static SX1262* asSX1262(R& r, std::true_type) { return &r; }
template <class R> static SX1262* asSX1262(R&, std::false_type) { return NULL; }
static SX1262* sx1262() { return asSX1262(radio, std::is_base_of<SX1262, RADIO_CLASS>{}); }

static void put16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void put32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static uint32_t get32(const uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

struct SpectrumSession::State {
  SX126xSpectrumScanner scanner { sx1262() };
  uint8_t run_state = STATE_STARTING;
  uint32_t keepalive_at;           // the latest OP_START
  uint32_t frame_at;               // the latest frame
  uint8_t frame_no = 0;

  // the latest settings
  uint8_t config = 0;              // changes with the sweep settings
  bool set_up = false;             // the radio sweeps with them
  float first_mhz, step_mhz, rbw_khz;
  uint8_t points, gain;
  uint16_t interval_ms;

  uint8_t level[MAX_POINTS];
};

int SpectrumSession::writeInfo(uint8_t* reply) {
  int i = 0;
  reply[i++] = RESP_CODE_SPECTRUM_INFO;
  if (_s == NULL) {   // STATE_OFF, and zeros
    memset(&reply[i], 0, 18);
    return i + 18;
  }
  reply[i++] = _s->run_state;
  reply[i++] = _s->config;
  reply[i++] = _s->points;
  put32(&reply[i], (uint32_t)(_s->first_mhz * 1e6 + 0.5)); i += 4;
  put32(&reply[i], (uint32_t)(_s->step_mhz * 1e6 + 0.5)); i += 4;
  put32(&reply[i], (uint32_t)(_s->rbw_khz * 1000 + 0.5f)); i += 4;
  put16(&reply[i], _s->interval_ms); i += 2;
  reply[i++] = _s->gain;
  return i;
}

int SpectrumSession::handleCommand(const uint8_t* frame, int len, uint8_t* reply) {
  if (len < 2) return -ERR_CODE_ILLEGAL_ARG;
  if (frame[1] == OP_STOP) {
    stop(NULL, STOP_CLIENT, 0);
    return writeInfo(reply);
  }
  if (frame[1] != OP_START || sx1262() == NULL) return -ERR_CODE_UNSUPPORTED_CMD;
  if (len < 14) return -ERR_CODE_ILLEGAL_ARG;

  uint32_t centre = get32(&frame[2]), span = get32(&frame[6]);
  uint8_t points = frame[10];
  uint16_t interval = frame[11] | frame[12] << 8;
  uint8_t gain = frame[13];
  if (centre == 0) centre = (uint32_t)(_prefs->freq * 1e6f + 0.5f);
  if (points < 16 || points > MAX_POINTS || span < points * 100u || span > 32000000) return -ERR_CODE_ILLEGAL_ARG;
  if (centre < 150000000 + span / 2 || centre > 960000000 - span / 2) return -ERR_CODE_ILLEGAL_ARG;
  if ((interval != 0 && interval < 10) || interval > 5000) return -ERR_CODE_ILLEGAL_ARG;
  if (gain > SX126xSpectrumScanner::GAIN_STEPS) return -ERR_CODE_ILLEGAL_ARG;

  if (_s == NULL) {
    _s = new (std::nothrow) State();   // the radio is taken in loop(), once the mesh has finished with it
    if (_s == NULL) return -ERR_CODE_BAD_STATE;
  }
  double step = span / 1e6 / points;
  float first_mhz = (centre - span / 2) / 1e6 + step / 2, step_mhz = step;
  float rbw_khz = SX126xSpectrumScanner::pickRxBw(step_mhz * 1000 * 2.5f);
  if (first_mhz != _s->first_mhz || step_mhz != _s->step_mhz || points != _s->points || gain != _s->gain
      || _s->config == 0) {   // else just a renewal: the frames on their way stay valid
    _s->first_mhz = first_mhz;
    _s->step_mhz = step_mhz;
    _s->rbw_khz = rbw_khz;
    _s->points = points;
    _s->gain = gain;
    _s->config++;
    _s->set_up = false;
  }
  _s->interval_ms = interval;
  _s->keepalive_at = millis();
  return writeInfo(reply);
}

void SpectrumSession::stop(BaseSerialInterface* serial, uint8_t reason, int16_t err) {
  if (_s == NULL) return;
  if (_s->run_state == STATE_RUNNING) {
    _s->scanner.end();
    radio_init();
    radio_driver.giveBack();
    radio_driver.setParams(_prefs->freq, _prefs->bw, _prefs->sf, _prefs->cr);
    radio_driver.setTxPower(_prefs->tx_power_dbm);
    radio_driver.setRxBoostedGainMode(_prefs->rx_boosted_gain);
  }
  delete _s;
  _s = NULL;
  if (serial) {
    uint8_t f[4] = { PUSH_CODE_SPECTRUM_STOPPED, reason, (uint8_t)err, (uint8_t)(err >> 8) };
    serial->writeFrame(f, sizeof(f));
  }
}

void SpectrumSession::returnRadio() { stop(NULL, STOP_RADIO, 0); }

void SpectrumSession::loop(BaseSerialInterface* serial) {
  if (_s == NULL) return;
  uint32_t now = millis();
  if (!serial->isConnected()) {
    stop(NULL, STOP_CLIENT, 0);   // no one to tell
    return;
  }
  if (now - _s->keepalive_at > KEEPALIVE_MS) {
    stop(serial, STOP_TIMEOUT, 0);
    return;
  }

  if (_s->run_state == STATE_STARTING) {
    if (!radio_driver.borrow(this)) return;   // the mesh is sending or receiving
  #ifdef SX126X_DIO3_TCXO_VOLTAGE
    float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
  #else
    float tcxo = 1.6f;
  #endif
  #ifdef SX126X_USE_REGULATOR_LDO
    bool ldo = SX126X_USE_REGULATOR_LDO;
  #else
    bool ldo = false;
  #endif
    _s->run_state = STATE_RUNNING;   // from here on, stop() gives the radio back to the mesh
    int16_t e = _s->scanner.begin(_prefs->freq, tcxo, ldo);
    if (e != RADIOLIB_ERR_NONE) {
      stop(serial, STOP_RADIO, e);
      return;
    }
  }
  if (!_s->set_up) {
    _s->scanner.setGain(_s->gain);
    int16_t e = _s->scanner.setSweep(_s->first_mhz, _s->step_mhz, _s->points, _s->rbw_khz);
    if (e == RADIOLIB_ERR_NONE) e = _s->scanner.startHold();
    if (e != RADIOLIB_ERR_NONE) {
      stop(serial, STOP_RADIO, e);
      return;
    }
    _s->frame_at = now;
    _s->set_up = true;
  }

  // a frame: once the interval is up (else as soon as the radio has swept the span again), and if the link keeps
  // up (else the radio goes on keeping the strongest levels for the next one)
  uint16_t ms = _s->interval_ms;
  if (serial->isWriteBusy() || (ms && now - _s->frame_at < ms)) return;
  bool got;
  int16_t e = _s->scanner.readFrame(_s->level, &got);
  if (e != RADIOLIB_ERR_NONE) {
    stop(serial, STOP_RADIO, e);
    return;
  }
  if (!got) return;
  if (ms) _s->frame_at = now - _s->frame_at < 2u * ms ? _s->frame_at + ms : now;   // keeps a steady beat

  int n = _s->points;
  _s->frame_no++;
  const int chunk = MAX_FRAME_SIZE - 4;
  for (int first = 0; first < n; first += chunk) {
    int len = n - first < chunk ? n - first : chunk;
    uint8_t f[MAX_FRAME_SIZE];
    f[0] = PUSH_CODE_SPECTRUM_FRAME;
    f[1] = _s->config;
    f[2] = _s->frame_no;
    f[3] = first;
    memcpy(&f[4], &_s->level[first], len);
    serial->writeFrame(f, 4 + len);
  }
}

#else   // a build without the spectrum stream

int SpectrumSession::handleCommand(const uint8_t*, int, uint8_t*) { return -1; }   // ERR_CODE_UNSUPPORTED_CMD
void SpectrumSession::loop(BaseSerialInterface*) { }
void SpectrumSession::returnRadio() { }
void SpectrumSession::stop(BaseSerialInterface*, uint8_t, int16_t) { }
int SpectrumSession::writeInfo(uint8_t*) { return 0; }

#endif
