#include "SpectrumSession.h"

#if SPECTRUM_STREAM && defined(NRF52_PLATFORM)

#include <new>
#include <type_traits>
#include <target.h>
#include <helpers/radiolib/SX126xSpectrumScanner.h>

extern RADIO_CLASS radio;   // in every variant's target.cpp

#if defined(BLE_PIN_CODE)
  #include <helpers/nrf52/SerialBLEInterface.h>
  extern SerialBLEInterface bluetooth_interface;   // main.cpp
  // A client that stops renewing its session has gone, but its link may still be up and keep the app out. It's
  // dropped where Bluetooth is the only companion link: elsewhere the client may have used another one.
  #if !defined(ENABLE_USB_INTERFACE) && !defined(SERIAL_RX) && !defined(ETHERNET_ENABLED) && !defined(WIFI_SSID)
    #define DROP_LINK_ON_TIMEOUT 1
  #endif
#endif

#define ERR_CODE_UNSUPPORTED_CMD  1   // as in MyMesh.cpp
#define ERR_CODE_BAD_STATE        4
#define ERR_CODE_ILLEGAL_ARG      6

#define DROP_LINK_DELAY_MS   300    // time for the last frames to go out before the link is dropped
#define RADIO_ERROR_MS      2000    // radio errors for this long end the session
#define TASK_STACK_WORDS    1024

// The radio as an SX1262, or NULL for other radios: the session compiles with all of them, and only works there.
template <class R> static SX1262* asSX1262(R& r, std::true_type) { return &r; }
template <class R> static SX1262* asSX1262(R&, std::false_type) { return NULL; }
static SX1262* sx1262() { return asSX1262(radio, std::is_base_of<SX1262, RADIO_CLASS>{}); }

static void put16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void put32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static uint32_t get32(const uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

struct SpectrumSession::State {
  SX126xSpectrumScanner scanner;
  uint8_t run_state;          // STATE_STARTING or STATE_RUNNING
  uint32_t keepalive_at;      // the latest OP_START
  uint32_t err_since;         // radio errors since then (0: none)
  uint32_t frame_at;          // the latest frame
  uint8_t frame_no;
  uint32_t rate_at, rate_sweeps;   // the sweep rate in RESP_SPECTRUM_INFO is counted from these
  uint16_t rate_x10;

  // the latest settings
  float first_mhz, step_mhz, rbw_khz;
  uint8_t points, agg;
  uint16_t interval_ms;

  // shared with the scan task
  TaskHandle_t task;
  volatile bool run, task_running;
  volatile uint8_t config, applied;   // 'config' changes with the sweep settings; the task sets 'applied'
  volatile int16_t err;               // of the latest sweep
  volatile uint32_t sweeps;
  // the sweeps since the latest frame: the task adds them up, loop() takes them (with the scheduler suspended,
  // so as to get a whole number of sweeps)
  uint8_t acc_max[MAX_POINTS];        // the strongest level, ie. the lowest -2 * dBm
  uint8_t acc_last[MAX_POINTS];
  float acc_power[MAX_POINTS];        // sum of the powers
  uint16_t acc_count;
  float power[256];                   // level -> power (mW)

  State() : scanner(sx1262()) {
    for (int l = 0; l < 256; l++) power[l] = powf(10.0f, -l / 20.0f);
    run_state = STATE_STARTING;
    run = task_running = false;
    config = 0;
    applied = 0xFF;
    err = RADIOLIB_ERR_NONE;
    err_since = 0;
    sweeps = rate_sweeps = 0;
    rate_x10 = 0;
    frame_no = 0;
    resetSums();
  }

  void resetSums() {
    memset(acc_max, 0xFF, sizeof(acc_max));
    memset(acc_last, 0xFF, sizeof(acc_last));
    memset(acc_power, 0, sizeof(acc_power));
    acc_count = 0;
  }

  static void taskEntry(void* self) { ((State *) self)->scanLoop(); }

  void scanLoop() {
    uint8_t level[MAX_POINTS];
    while (run) {
      uint8_t c = config;
      if (c != applied) {
        int16_t e = scanner.setSweep(first_mhz, step_mhz, points, rbw_khz);
        vTaskSuspendAll();
        resetSums();
        xTaskResumeAll();
        applied = c;
        if (e != RADIOLIB_ERR_NONE) { err = e; delay(10); continue; }
      }
      int16_t e = scanner.sweep(level);
      err = e;
      if (e != RADIOLIB_ERR_NONE) { delay(10); continue; }
      if (config != c) continue;   // of the previous settings
      vTaskSuspendAll();
      for (int i = 0; i < points; i++) {
        uint8_t v = level[i];
        if (v < acc_max[i]) acc_max[i] = v;
        acc_power[i] += power[v];
        acc_last[i] = v;
      }
      acc_count++;
      sweeps++;
      xTaskResumeAll();
    }
    task_running = false;
    vTaskDelete(NULL);
  }
};

bool SpectrumSession::ownsRadio() const { return _s != NULL && _s->run_state == STATE_RUNNING; }

int SpectrumSession::writeInfo(uint8_t* reply) {
  int i = 0;
  reply[i++] = RESP_SPECTRUM_INFO;
  if (_s == NULL) {   // STATE_OFF, and zeros
    memset(&reply[i], 0, 20);
    return i + 20;
  }
  uint32_t now = millis();
  if (_s->run_state == STATE_RUNNING && now - _s->rate_at >= 1000) {
    uint32_t n = _s->sweeps;
    _s->rate_x10 = (n - _s->rate_sweeps) * 10000 / (now - _s->rate_at);
    _s->rate_sweeps = n;
    _s->rate_at = now;
  }
  reply[i++] = _s->run_state;
  reply[i++] = _s->config;
  reply[i++] = _s->points;
  put32(&reply[i], (uint32_t)(_s->first_mhz * 1e6 + 0.5)); i += 4;
  put32(&reply[i], (uint32_t)(_s->step_mhz * 1e6 + 0.5)); i += 4;
  put32(&reply[i], (uint32_t)(_s->rbw_khz * 1000 + 0.5f)); i += 4;
  put16(&reply[i], _s->interval_ms); i += 2;
  reply[i++] = _s->agg;
  put16(&reply[i], _s->rate_x10); i += 2;
  return i;
}

int SpectrumSession::handleCommand(const uint8_t* frame, int len, uint8_t* reply) {
  if (len < 2) return -ERR_CODE_ILLEGAL_ARG;
  if (frame[1] == OP_STOP) {
    stop(NULL, STOP_CLIENT, 0);
    if (len >= 3 && (frame[2] & STOP_DISCONNECT)) dropLinkSoon();
    return writeInfo(reply);
  }
  if (frame[1] != OP_START || sx1262() == NULL) return -ERR_CODE_UNSUPPORTED_CMD;
  if (len < 18) return -ERR_CODE_ILLEGAL_ARG;

  uint32_t centre = get32(&frame[2]), span = get32(&frame[6]);
  uint8_t points = frame[10];
  uint32_t rbw = get32(&frame[11]);
  uint16_t interval = frame[15] | frame[16] << 8;
  uint8_t agg = frame[17];
  if (centre == 0) centre = (uint32_t)(_prefs->freq * 1e6f + 0.5f);
  if (points < 16 || points > MAX_POINTS || span < points * 100u || span > 32000000) return -ERR_CODE_ILLEGAL_ARG;
  if (centre < 150000000 + span / 2 || centre > 960000000 - span / 2) return -ERR_CODE_ILLEGAL_ARG;
  if ((interval != 0 && interval < 10) || interval > 5000 || agg > AGG_LAST) return -ERR_CODE_ILLEGAL_ARG;

  if (_s == NULL) {
    _s = new (std::nothrow) State();   // the radio is taken in loop(), once the mesh has finished with it
    if (_s == NULL) return -ERR_CODE_BAD_STATE;
  }
  double step = span / 1e6 / points;
  float first_mhz = (centre - span / 2) / 1e6 + step / 2, step_mhz = step;
  float rbw_khz = SX126xSpectrumScanner::pickRxBw(rbw ? rbw / 1000.0f : step_mhz * 1000 * 2.5f);
  if (first_mhz != _s->first_mhz || step_mhz != _s->step_mhz || rbw_khz != _s->rbw_khz || points != _s->points
      || _s->config == 0) {   // else just a renewal: the frames on their way stay valid
    _s->first_mhz = first_mhz;
    _s->step_mhz = step_mhz;
    _s->rbw_khz = rbw_khz;
    _s->points = points;
    _s->config++;
  }
  _s->agg = agg;
  _s->interval_ms = interval;
  _s->keepalive_at = millis();
  return writeInfo(reply);
}

void SpectrumSession::stop(BaseSerialInterface* serial, uint8_t reason, int16_t err) {
  if (_s == NULL) return;
  if (_s->task_running) {
    _s->run = false;
    for (uint32_t t = millis(); _s->task_running && millis() - t < 2500; ) delay(1);
    if (_s->task_running) board.reboot();   // stuck in a sweep: the radio can't be taken back safely
  }
  if (_s->run_state == STATE_RUNNING) {
    _s->scanner.end();
    radio_init();   // resets the chip, and so drops the plugin
    radio_driver.setParams(_prefs->freq, _prefs->bw, _prefs->sf, _prefs->cr);
    radio_driver.setTxPower(_prefs->tx_power_dbm);
    radio_driver.setRxBoostedGainMode(_prefs->rx_boosted_gain);
    radio_driver.onExternalReinit();
  }
  delete _s;
  _s = NULL;
  if (serial && serial->isConnected()) {
    uint8_t f[4] = { PUSH_SPECTRUM_STOPPED, reason, (uint8_t)err, (uint8_t)(err >> 8) };
    serial->writeFrame(f, sizeof(f));
  }
}

void SpectrumSession::loop(BaseSerialInterface* serial) {
  uint32_t now = millis();
  if (_drop_link_at != 0 && now - _drop_link_at > DROP_LINK_DELAY_MS) {
    _drop_link_at = 0;
  #if defined(BLE_PIN_CODE)
    bluetooth_interface.disconnect();
  #endif
  }
  if (_s == NULL) return;
  if (!serial->isConnected()) {
    stop(serial, STOP_DISCONNECTED, 0);
    return;
  }
  if (now - _s->keepalive_at > KEEPALIVE_MS) {
    stop(serial, STOP_TIMEOUT, 0);
  #if DROP_LINK_ON_TIMEOUT
    dropLinkSoon();
  #endif
    return;
  }

  if (_s->run_state == STATE_STARTING) {
    if (!radio_driver.isIdleInRecvMode()) return;   // the mesh is sending or receiving a packet
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
    if (e == RADIOLIB_ERR_NONE) {
      _s->run = _s->task_running = true;
      if (xTaskCreate(State::taskEntry, "spectrum", TASK_STACK_WORDS, _s, TASK_PRIO_LOW, &_s->task) != pdPASS) {
        _s->run = _s->task_running = false;
        e = RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (e != RADIOLIB_ERR_NONE) {
      stop(serial, STOP_RADIO, e);
      return;
    }
    _s->frame_at = _s->rate_at = now;
    return;
  }

  int16_t err = _s->err;
  if (err == RADIOLIB_ERR_NONE) {
    _s->err_since = 0;
  } else if (_s->err_since == 0) {
    _s->err_since = now | 1;
  } else if (now - _s->err_since > RADIO_ERROR_MS) {
    stop(serial, STOP_RADIO, err);
    return;
  }

  // a frame: once the interval is up, if there's a sweep of the current settings, and if the link keeps up
  // (else the sweeps go on adding up for the next one)
  uint16_t ms = _s->interval_ms;
  if (ms && now - _s->frame_at < ms) return;
  if (_s->applied != _s->config || _s->acc_count == 0 || serial->isWriteBusy()) return;
  if (ms) _s->frame_at = now - _s->frame_at < 2u * ms ? _s->frame_at + ms : now;   // keeps a steady beat

  int n = _s->points;
  uint8_t out[MAX_POINTS];
  vTaskSuspendAll();
  if (_s->agg == AGG_MAX) {
    memcpy(out, _s->acc_max, n);
  } else if (_s->agg == AGG_LAST) {
    memcpy(out, _s->acc_last, n);
  } else {
    for (int i = 0; i < n; i++) {
      int v = (int)(-20.0f * log10f(_s->acc_power[i] / _s->acc_count) + 0.5f);
      out[i] = v < 0 ? 0 : (v > 254 ? 254 : v);
    }
  }
  _s->resetSums();
  xTaskResumeAll();

  _s->frame_no++;
  const int chunk = MAX_FRAME_SIZE - 4;
  for (int first = 0; first < n; first += chunk) {
    int len = n - first < chunk ? n - first : chunk;
    uint8_t f[MAX_FRAME_SIZE];
    f[0] = PUSH_SPECTRUM_FRAME;
    f[1] = _s->config;
    f[2] = _s->frame_no;
    f[3] = first;
    memcpy(&f[4], &out[first], len);
    serial->writeFrame(f, 4 + len);
  }
}

#else   // a build without the spectrum stream

bool SpectrumSession::ownsRadio() const { return false; }
int SpectrumSession::handleCommand(const uint8_t*, int, uint8_t*) { return -1; }   // ERR_CODE_UNSUPPORTED_CMD
void SpectrumSession::loop(BaseSerialInterface*) { }
void SpectrumSession::stop(BaseSerialInterface*, uint8_t, int16_t) { }
int SpectrumSession::writeInfo(uint8_t*) { return 0; }

#endif
