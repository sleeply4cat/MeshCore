#pragma once

// The spectrum view of a companion UI: a live graph with peak hold and a waterfall of a band around the node's
// channel, from sweeps the SX1262 runs itself (SX126xSpectrumScanner). The UI around it (ui-new's page, or a
// variant's own UI) gives it a display to draw on and a Host for alerts and to leave; the mesh is paused while it
// owns the radio (ownsRadio(), RadioLibWrapper::borrow()). Without a display (start(NULL), see
// SpectrumHeadless.h) it only feeds a stream client.
// Needs the variant's target.h (board, radio_driver) included first, and FreeRTOS (nRF52, ESP32).

#include <Arduino.h>
#include <helpers/ui/DisplayDriver.h>
#include "NodePrefs.h"
#include <helpers/radiolib/SX126xSpectrumScanner.h>
#include <type_traits>

extern RADIO_CLASS radio;   // in every variant's target.cpp

// the radio as an SX1262, or NULL for other radios: the page builds with all of them, and only scans there
template <class R> static SX1262* spectrumRadio(R& r, std::true_type) { return &r; }
template <class R> static SX1262* spectrumRadio(R&, std::false_type) { return NULL; }

#ifndef SPECTRUM_GAIN
  #define SPECTRUM_GAIN        13      // a fixed gain step, the AGC's top one (7C): no AGC shelf after strong signals
#endif
#if SPECTRUM_BLE && !defined(SPECTRUM_BLE_FRAME_MS)
  #define SPECTRUM_BLE_FRAME_MS  50   // default interval of the frames for a stream client (it can change it)
#endif
#define SPECTRUM_POINTS      SX126xSpectrumScanner::MAX_POINTS   // at most; a sweep has as many points as the panel is wide
#ifndef SPECTRUM_FRAME_MS
  #define SPECTRUM_FRAME_MS      50    // min time between drawing frames (the panel refresh overlaps it)
#endif
#ifndef SPECTRUM_HEADER_MS
  #define SPECTRUM_HEADER_MS    250    // how often the peak frequency in the header is updated
#endif
#define SPECTRUM_PEAK_DB     10      // the loudest point counts as a signal this far above the noise
#define SPECTRUM_PEAK_FRAMES 3       // the peak marker follows the loudest point of the last this many frames
// SPECTRUM_HEADER_H: the header text line, in UI pixels (default: the display's line of text; 16 puts it in the
//   yellow band of two-colour OLEDs)
// SPECTRUM_WF_ROW_MS: a waterfall row per this many ms (the max over them), 0 = one per frame (default: one per
//   frame on e-ink, every 100ms elsewhere)
#ifndef SPECTRUM_HOLD_DECAY_DBPS
  #define SPECTRUM_HOLD_DECAY_DBPS 10  // peak-hold decay, dB per second
#endif
#define SPECTRUM_MIN_WF_ROWS 8       // below that, no waterfall: the graph gets the whole height
#define SPECTRUM_NO_SIGNAL   -140
#define SPECTRUM_RANGE_DB    50      // vertical range of the graph
#define SPECTRUM_WF_LO       3       // waterfall: dB above noise where the grey scale starts
#define SPECTRUM_WF_RANGE    32      //            and its span in dB, up to solid black

// SDR-like view of the band around the mesh frequency: live spectrum with decaying peak-hold, plus a waterfall.
// Sweeps run continuously in their own RTOS task, so they don't stop while the display is drawn or refreshed;
// each display frame shows the maximum seen at each point since the previous frame, so short bursts aren't lost.
// The view borrows the radio from the mesh meanwhile (RadioLibWrapper::borrow()): the mesh neither receives nor sends,
// and the app is still served.
class SpectrumView : public RadioBorrower {
public:
  // what the view needs from the UI around it
  struct Host {
    virtual void spectrumAlert(const char* text, int duration_ms) = 0;
    virtual void spectrumExit() = 0;   // leave the view: the host calls stop() on the way
    virtual void spectrumOpen() { }    // a stream client asks for the view: the host opens it (start())
    virtual bool spectrumOnDevice() { return false; }   // opened on the device's display (not by the stream client)
  };

  // A client of the sweeps beside the display, eg. a BLE service for a web page (SPECTRUM_BLE): frames go to it
  // from the scan task, and its requests are served from streamLoop(). All values little-endian.
  //  info: u8 version (1), u8 state (STATE_*), u8 span index, u8 points, u32 first point (Hz), u32 step (Hz),
  //        u32 RBW (Hz), u32 centre (Hz), u32 node frequency (Hz), u32 node bandwidth (Hz), i16 radio error,
  //        u8 config generation, u8 flags (FLAG_*), u16 frame interval (ms, 0: every sweep), u8 AGG_* (AGG_MAX), u8 0
  //  frames: the strongest level per point over the sweeps since the previous frame, -2 * dBm (255: no data)
  //  requests: CTRL_START [span], CTRL_STOP, CTRL_SPAN span, CTRL_CENTRE Hz (0: follow the node's channel),
  //        CTRL_INTERVAL ms (0: every sweep, else 10..2000), CTRL_AGG AGG_* (only AGG_MAX: the radio's held sweeps
  //        keep the strongest level per point, there's no mean or single sweep to send)
  struct Stream {
    enum { STATE_OFF = 0, STATE_STARTING = 1, STATE_RUNNING = 2 };
    enum { FLAG_FOLLOW = 1, FLAG_ON_DEVICE = 2 };   // centre on the node's channel; opened on the device (CTRL_STOP closes it there)
    enum { CTRL_START = 1, CTRL_STOP = 2, CTRL_SPAN = 3, CTRL_CENTRE = 4, CTRL_INTERVAL = 5, CTRL_AGG = 6 };
    enum { AGG_MAX = 0 };
    enum { INFO_LEN = 36 };
    struct Request {
      uint8_t op;
      uint32_t arg;   // 0xFFFFFFFF: none
    };
    virtual bool hasSubscribers() = 0;                          // anyone to send frames to
    virtual void setInfo(const uint8_t info[INFO_LEN]) = 0;     // the state: sent on change
    virtual bool sendFrame(uint8_t gen, const uint8_t* levels, uint8_t n) = 0;   // never waits for the link
    virtual bool takeRequest(Request& req) = 0;
  };
  enum Action { ZOOM_OUT, ZOOM_IN, EXIT };

private:
  Host* _host;
  NodePrefs* _prefs;
  DisplayDriver* _display;
  SX126xSpectrumScanner _scanner;
  bool _open, _pending_start, _need_wipe;
  uint8_t _span_idx;
  float _freq, _bw;      // the mesh channel shown; follows the node's prefs (render())
  float _center;         // centre of the view: the mesh channel, unless a stream client set another
  bool _follow;          // _center follows _freq

  // shared with the scan task
  TaskHandle_t _scan_task;
  volatile bool _run, _task_running;
  volatile uint8_t _span_req;
  volatile uint8_t _cfg_gen, _cfg_applied;   // bumped on every span or channel change; the scan task follows
  volatile int16_t _cur[SPECTRUM_POINTS];    // latest sweep, dBm
  volatile int16_t _fmax[SPECTRUM_POINTS];   // max since the last display frame
  volatile int16_t _sweep_err;               // of the latest sweep, RADIOLIB_ERR_NONE if it went fine

  // display side
  int _w, _scale_y, _graph_y, _graph_h, _wf_y, _wf_rows, _wf_stride, _wf_head, _wf_count;
  uint16_t _points;      // per sweep: one per pixel column, up to SPECTRUM_POINTS (set before the scan task starts)
  uint32_t _wf_pushed;
  uint8_t* _wf_mem;      // one allocation for all the buffers below
  uint8_t* _wf;          // waterfall ring buffer of dithered 1-bit rows
  int16_t* _wf_err;      // error diffusion: carried into the next row, and being collected for it (2 x (w + 2))
  int16_t* _wf_err_next;
  uint8_t* _px_idx;      // per pixel column: left point index and weight of the right one, for interpolation
  uint8_t* _px_frac;
  int16_t _frame[SPECTRUM_POINTS], _hold[SPECTRUM_POINTS];
  int16_t _recent[SPECTRUM_PEAK_FRAMES - 1][SPECTRUM_POINTS];   // previous frames, for the peak marker
  uint8_t _recent_head;
  int16_t _wf_acc[SPECTRUM_POINTS];   // max over the frames of the waterfall row being collected
  int16_t _floor_dbm, _frame_noise;
  uint32_t _header_at;
  int _peak;             // loudest point of the last few frames, -1 if it's not a signal: a marker on the scale
  int _hdr_peak;        // loudest signal point since the header was updated, -1 if none
  int16_t _hdr_peak_dbm;
  float _peak_mhz;       // frequency of the latest loudest signal, in the header
  uint32_t _hold_at, _wf_row_at;
  uint16_t _wf_row_ms;   // see SPECTRUM_WF_ROW_MS
  uint16_t _hold_frac;   // peak-hold decay not applied yet, in dB/1000

  // A point costs ~1 RSSI update, ~1.6/RBW ms (RBW in kHz), so the sweep time grows with points / RBW:
  // with 250 points ~56ms per sweep at 0.5MHz, ~21ms at 2MHz, ~5ms at 8MHz, ~4ms at 16MHz. Fewer points
  // (a narrower panel) mean wider steps, so a wider RBW too: faster on both counts.
  static float spanMHz(uint8_t idx) {
    static const float spans[] = { 0.5f, 2.0f, 8.0f, 16.0f };
    return spans[idx % 4];
  }
  static const uint16_t* spanTicksKHz(uint8_t idx) {   // frequency scale: ticks at round frequencies, 3 sizes
    static const uint16_t ticks[][3] = { { 25, 100, 500 }, { 100, 500, 1000 }, { 500, 1000, 5000 }, { 1000, 5000, 10000 } };
    return ticks[idx % 4];
  }
  static float spanRbwSteps(uint8_t idx) {   // RX bandwidth, in sweep steps
    static const float steps[] = { 3.0f, 2.0f, 2.0f, 1.5f };
    return steps[idx % 4];
  }
  float edgeMHz(uint8_t idx) const { return _center - spanMHz(idx) / 2; }

#if SPECTRUM_BLE
  // scan task only: the strongest level per point since the last stream frame
  Stream* _stream = NULL;
  uint8_t _str_max[SPECTRUM_POINTS];          // -2 * dBm, as the radio's: the strongest is the lowest
  uint32_t _str_at;
  volatile uint16_t _str_ms = SPECTRUM_BLE_FRAME_MS;   // frame interval, 0: every frame from the radio

  void streamReset() { memset(_str_max, 0xFF, sizeof(_str_max)); }

  // from the scan task, for every frame the radio hands over: not held up by the display, whose refresh may block
  // the UI loop
  void streamSweep(const uint8_t* level, int n, uint8_t gen) {
    if (!_stream->hasSubscribers()) return;   // nobody to send to
    for (int i = 0; i < n; i++) if (level[i] < _str_max[i]) _str_max[i] = level[i];
    uint32_t now = millis();
    uint16_t ms = _str_ms;
    if (ms) {
      if (now - _str_at < ms) return;
      _str_at = now - _str_at < 2u * ms ? _str_at + ms : now;   // on a steady beat
    }
    _stream->sendFrame(gen, _str_max, n);
    streamReset();
  }
#endif

  // --- scan task ---

  static void scanTaskEntry(void* self) { ((SpectrumView *) self)->scanLoop(); }

  // the radio sweeps on by itself with these settings (held sweeps), until the next setSweep() or end()
  int16_t applySpan(uint8_t idx) {
    int n = _points;
    float step = spanMHz(idx) / n;
    int16_t err = _scanner.setSweep(edgeMHz(idx) + step / 2, step, n, step * 1000.0f * spanRbwSteps(idx));
    if (err == RADIOLIB_ERR_NONE) err = _scanner.startHold();
    for (int i = 0; i < SPECTRUM_POINTS; i++) _cur[i] = _fmax[i] = SPECTRUM_NO_SIGNAL;
  #if SPECTRUM_BLE
    streamReset();
  #endif
    return err;
  }

  void scanLoop() {
    uint8_t level[SPECTRUM_POINTS];
    bool held = false;   // the radio sweeps with the latest settings
    while (_run) {
      uint8_t gen = _cfg_gen;
      if (gen != _cfg_applied || !held) {
        int16_t err = applySpan(_span_req);
        _cfg_applied = gen;
        held = err == RADIOLIB_ERR_NONE;
        _sweep_err = err;
        if (!held) {                      // shown in the header; keep trying
          delay(10);
          continue;
        }
      }
      // the strongest level per point over the whole rows since the previous frame: asked for on the first call,
      // taken once the radio ends the row under way
      bool got = false;
      int16_t err = _scanner.readFrame(level, &got);
      _sweep_err = err;
      if (err != RADIOLIB_ERR_NONE) {     // eg. no row ended in time: start the sweeps again
        held = false;
        delay(10);
        continue;
      }
      if (!got) {
        delay(1);
        continue;
      }
      if (_cfg_gen != gen) continue;      // span or channel changed meanwhile
      int n = _scanner.numPoints();
      vTaskSuspendAll();                  // publish the sweep as a whole: a frame never sees half of one
      for (int i = 0; i < n; i++) {
        int16_t a = -(level[i] + 1) / 2;  // -2 * dBm -> dBm
        _cur[i] = a;
        if (a > _fmax[i]) _fmax[i] = a;
      }
      xTaskResumeAll();
    #if SPECTRUM_BLE
      if (_stream) streamSweep(level, n, gen);
    #endif
    }
    _task_running = false;
    vTaskDelete(NULL);
  }

  bool startScanTask() {
    _run = _task_running = true;
    _cfg_applied = _cfg_gen + 1;     // forces applySpan() in the task
  #if defined(ESP32)
    // on the UI's core: vTaskSuspendAll() then keeps the task off while a frame is taken (stack in bytes here)
    BaseType_t ok = xTaskCreatePinnedToCore(scanTaskEntry, "spectrum", 4096, this, 1, &_scan_task, xPortGetCoreID());
  #else
    BaseType_t ok = xTaskCreate(scanTaskEntry, "spectrum", 1024, this, TASK_PRIO_LOW, &_scan_task);
  #endif
    if (ok != pdPASS) {
      _run = _task_running = false;
      return false;
    }
    return true;
  }

  // false if the task didn't finish (stuck on the radio): then it may still be using the radio
  bool stopScanTask() {
    _run = false;
    for (uint32_t t = millis(); _task_running && millis() - t < 2500; ) delay(1);
    return !_task_running;
  }

  // --- display side ---

  bool allocBuffers(DisplayDriver& d) {
    _w = d.nativeWidth();
    _points = _w < SPECTRUM_POINTS ? _w : SPECTRUM_POINTS;
    int h = d.nativeHeight();
  #ifdef SPECTRUM_HEADER_H
    int header_h = SPECTRUM_HEADER_H;
  #else
    int header_h = d.textLineHeight();
  #endif
  #ifdef SPECTRUM_WF_ROW_MS
    _wf_row_ms = SPECTRUM_WF_ROW_MS;
  #else
    _wf_row_ms = d.isEink() ? 0 : 100;
  #endif
    _scale_y = d.toNativeY(header_h) + 1;   // frequency scale: a line, ticks down from it
    _graph_y = _scale_y + 4;
    int avail = h - _scale_y;
    _graph_h = avail * 2 / 5 - 4;
    _wf_y = _graph_y + _graph_h + 3;
    _wf_rows = h - _wf_y;
    if (_wf_rows < SPECTRUM_MIN_WF_ROWS) {   // too short for a waterfall: all of it for the graph
      _wf_rows = 0;
      _graph_h = h - _graph_y - 3;           // room for the baseline and the channel marker
      _wf_y = h;
    }

    _wf_stride = (_w + 7) / 8;

    // int16 buffers first, for alignment
    uint8_t* mem = (uint8_t*) malloc(2 * (_w + 2) * sizeof(int16_t) + _wf_stride * _wf_rows + _w * 2);
    if (mem == NULL) return false;
    _wf_mem = mem;
    _wf_err = (int16_t*) mem;
    _wf_err_next = _wf_err + _w + 2;
    _wf = mem + 2 * (_w + 2) * sizeof(int16_t);
    _px_idx = _wf + _wf_stride * _wf_rows;
    _px_frac = _px_idx + _w;
    buildPixelMap();
    return true;
  }

  void buildPixelMap() {
    int n = _points;
    for (int x = 0; x < _w; x++) {   // point i is centred on pixel (i + 0.5) * w / points
      int pos = ((2 * x + 1) * n * 128) / _w - 128;   // in 1/256 of a point
      if (pos < 0) pos = 0;
      if (pos > (n - 1) * 256) pos = (n - 1) * 256;
      _px_idx[x] = pos >> 8;
      _px_frac[x] = pos & 0xFF;
    }
  }

  void freeBuffers() {
    free(_wf_mem);   // _wf_err and _wf_err_next swap places, so they can't be freed
    _wf_mem = _wf = _px_idx = _px_frac = NULL;
    _wf_err = _wf_err_next = NULL;
  }

  void clearHistory() {
    for (int i = 0; i < SPECTRUM_POINTS; i++) _frame[i] = _hold[i] = _wf_acc[i] = SPECTRUM_NO_SIGNAL;
    for (int f = 0; f < SPECTRUM_PEAK_FRAMES - 1; f++)
      for (int i = 0; i < SPECTRUM_POINTS; i++) _recent[f][i] = SPECTRUM_NO_SIGNAL;
    _recent_head = 0;
    memset(_wf, 0, _wf_stride * _wf_rows);
    memset(_wf_err, 0, 2 * (_w + 2) * sizeof(int16_t));
    _wf_head = _wf_count = 0;
    _wf_pushed = 0;
    _hold_frac = 0;
    _hold_at = _wf_row_at = millis();
  }

  // median of a sweep's values (dBm), clamped to SPECTRUM_NO_SIGNAL..0
  int medianDbm(const int16_t* v) const {
    uint8_t counts[-SPECTRUM_NO_SIGNAL + 1];
    memset(counts, 0, sizeof(counts));
    for (int i = 0; i < _points; i++) {
      int a = v[i] < SPECTRUM_NO_SIGNAL ? SPECTRUM_NO_SIGNAL : (v[i] > 0 ? 0 : v[i]);
      counts[a - SPECTRUM_NO_SIGNAL]++;
    }
    for (int i = 0, seen = 0; i <= -SPECTRUM_NO_SIGNAL; i++) {
      seen += counts[i];
      if (seen * 2 >= _points) return i + SPECTRUM_NO_SIGNAL;
    }
    return 0;
  }

  // take the per-point max since the last frame, and push it into the waterfall
  void takeFrame(uint32_t now) {
    int n = _points;
    vTaskSuspendAll();      // take and reset the accumulated sweeps as a whole (see scanLoop)
    for (int i = 0; i < n; i++) {
      int16_t v = _fmax[i];
      _fmax[i] = _cur[i];   // a point may not have been re-measured yet: keep its latest value
      _frame[i] = v <= SPECTRUM_NO_SIGNAL ? _cur[i] : v;
    }
    xTaskResumeAll();

    // peak-hold, decaying at a fixed rate whatever the frame rate (e-ink ~10, OLED ~20 FPS)
    uint32_t dt = now - _hold_at;
    _hold_at = now;
    uint32_t decay = (dt > 1000 ? 1000 : dt) * SPECTRUM_HOLD_DECAY_DBPS + _hold_frac;   // dB/1000
    int dec = decay / 1000;
    _hold_frac = decay % 1000;
    for (int i = 0; i < n; i++) {
      int h = _hold[i] - dec;
      _hold[i] = _frame[i] > h ? _frame[i] : h;
      if (_frame[i] > _wf_acc[i]) _wf_acc[i] = _frame[i];
    }

    // noise of this frame: median of its values. A frame is the max of the sweeps since the last one, so it
    // sits above a single sweep's median, and the waterfall scale must follow it. Every RSSI update is equally
    // noisy at any RBW (the update period is ~1.6/RBW), but wide spans get more sweeps per frame (~2 at 0.5MHz,
    // ~15 at 16MHz): the max lifts their noise more (+2 vs +6dB).
    _frame_noise = medianDbm(_frame);
    int lo = _frame_noise - 6;   // graph baseline a little below the noise, in 5dB steps to keep it stable
    _floor_dbm = lo - ((lo % 5) + 5) % 5;

    if (_wf_rows == 0 || now - _wf_row_at < _wf_row_ms) return;
    _wf_row_at = (now - _wf_row_at < 2u * _wf_row_ms) ? _wf_row_at + _wf_row_ms : now;   // keep the cadence
    // the row is the max over its frames, so its noise sits higher still: the grey scale follows its own median
    ditherRow(&_wf[_wf_head * _wf_stride], _wf_acc, medianDbm(_wf_acc));
    for (int i = 0; i < n; i++) _wf_acc[i] = SPECTRUM_NO_SIGNAL;
    _wf_head = (_wf_head + 1) % _wf_rows;
    if (_wf_count < _wf_rows) _wf_count++;
    _wf_pushed++;
  }

  // Floyd-Steinberg, row by row as rows come: the error flows to the right and into the next row.
  // Each row is dithered once and then only scrolls, so old rows never change, and there's no fixed
  // pattern for the scroll to invert (which flickers on e-ink, and crawls on an OLED).
  void ditherRow(uint8_t* bits, const int16_t* row, int noise) {
    memset(bits, 0, _wf_stride);
    int16_t* carry = _wf_err + 1;        // [-1 .. w] valid
    int16_t* next = _wf_err_next + 1;
    memset(_wf_err_next, 0, (_w + 2) * sizeof(int16_t));
    bool reverse = _wf_pushed & 1;       // serpentine, against directional artifacts
    int dir = reverse ? -1 : 1;
    int ahead = 0;
    for (int k = 0; k < _w; k++) {
      int x = reverse ? _w - 1 - k : k;
      int db = interp(row, x) - noise - SPECTRUM_WF_LO;
      int want = db <= 0 ? 0 : (db >= SPECTRUM_WF_RANGE ? 256 : db * 256 / SPECTRUM_WF_RANGE);
      int val = want + carry[x] + ahead;
      bool black = val >= 128;
      if (black) bits[x >> 3] |= 0x80 >> (x & 7);
      int e = val - (black ? 256 : 0);
      if (e > 256) e = 256;
      if (e < -256) e = -256;
      ahead = e * 7 / 16;
      next[x - dir] += e * 3 / 16;
      next[x] += e * 5 / 16;
      next[x + dir] += e / 16;
    }
    int16_t* t = _wf_err; _wf_err = _wf_err_next; _wf_err_next = t;
  }

  int interp(const int16_t* v, int x) const {
    int i = _px_idx[x], f = _px_frac[x];
    return f == 0 ? v[i] : (v[i] * (256 - f) + v[i + 1] * f) / 256;
  }

  int dbmToH(int dbm) const {
    int v = (dbm - _floor_dbm) * _graph_h / SPECTRUM_RANGE_DB;
    return v < 0 ? 0 : (v > _graph_h ? _graph_h : v);
  }

  int freqToX(float mhz) const { return (int)((mhz - edgeMHz(_span_idx)) / spanMHz(_span_idx) * _w); }

  void renderGraph(DisplayDriver& d) {
    int bottom = _graph_y + _graph_h;
    for (int x = 0; x < _w; x++) {
      int a = dbmToH(interp(_frame, x));
      if (a > 0) d.fillRectNative(x, bottom - a, 1, a);
      int hold = dbmToH(interp(_hold, x));
      if (hold > a + 1 && (x & 1) == 0) d.fillRectNative(x, bottom - hold, 1, 1);   // dotted peak-hold
    }
    d.fillRectNative(0, bottom, _w, 1);
    // frequency scale above the graph: ticks get longer at rounder frequencies
    const uint16_t* ticks = spanTicksKHz(_span_idx);
    double edge_khz = edgeMHz(_span_idx) * 1000.0, span_khz = spanMHz(_span_idx) * 1000.0;
    d.fillRectNative(0, _scale_y, _w, 1);
    for (long k = (long)ceil(edge_khz / ticks[0]) * ticks[0]; k < edge_khz + span_khz; k += ticks[0]) {
      int x = (int)((k - edge_khz) / span_khz * _w);
      if (x >= 0 && x < _w) d.fillRectNative(x, _scale_y + 1, 1, k % ticks[2] == 0 ? 3 : (k % ticks[1] == 0 ? 2 : 1));
    }
    if (_peak >= 0) {   // the loudest signal: a small triangle over the ticks
      int px = (2 * _peak + 1) * _w / (2 * _points);
      for (int r = 0; r < 3; r++) d.fillRectNative(px - 2 + r, _scale_y + 1 + r, 5 - 2 * r, 1);
    }
    for (int db = 10; db < SPECTRUM_RANGE_DB; db += 10) {   // 10dB ticks
      d.fillRectNative(0, bottom - db * _graph_h / SPECTRUM_RANGE_DB, 3, 1);
    }
    // mesh channel: dotted centre line, and its bandwidth on the baseline
    int cx = freqToX(_freq);
    int half_bw = (int)(_bw / 2000.0f / spanMHz(_span_idx) * _w);
    if (cx >= 0 && cx < _w) {   // off the view when a stream client moved its centre
      for (int y = _graph_y; y < bottom; y += 3) d.fillRectNative(cx, y, 1, 1);
    }
    int bl = cx - half_bw < 0 ? 0 : cx - half_bw, br = cx + half_bw >= _w ? _w - 1 : cx + half_bw;
    if (br >= bl) d.fillRectNative(bl, bottom + 1, br - bl + 1, 2);
  }

  void renderWaterfall(DisplayDriver& d) {
    for (int r = 0; r < _wf_count; r++) {   // newest on top
      const uint8_t* bits = &_wf[((_wf_head - 1 - r + _wf_rows) % _wf_rows) * _wf_stride];
      int y = _wf_y + r;
      int run = -1;
      for (int x = 0; x <= _w; x++) {
        bool on = x < _w && (bits[x >> 3] & (0x80 >> (x & 7)));
        if (on && run < 0) run = x;
        if (!on && run >= 0) { d.fillRectNative(run, y, x - run, 1); run = -1; }
      }
    }
  }

  // edges of the span on the left, the loudest signal's frequency on the right; a narrow panel (128px OLED:
  // 21 characters) gets the centre and the span instead of the edges
  void renderHeader(DisplayDriver& d) {
    char left[24], right[24];
    float edge = edgeMHz(_span_idx), span = spanMHz(_span_idx);
    int16_t err = _sweep_err;
    bool ok = err == RADIOLIB_ERR_NONE;
    sprintf(left, "%.3f-%.3f", edge, edge + span);
    if (ok) sprintf(right, "%.3f", _peak_mhz);
    else sprintf(right, "radio err %d", err);

    const int gap = 6;
    int wr = d.getTextWidth(right);
    if (d.getTextWidth(left) + gap + wr > d.width()) {
      sprintf(left, span < 1 ? "%.3f %.1fM" : "%.3f %.0fM", _center, span);
    }
    if (d.getTextWidth(left) + gap + wr <= d.width()) {   // else only the error: it matters more
      d.setCursor(0, 0);
      d.print(left);
    }
    d.drawTextRightAlign(d.width() - 1, 0, right);
  }

  void restoreRadio() {
    _scanner.end();   // resets the SX126x, which drops the plugin
    bool ok = radio_init();
    radio_driver.giveBack();   // before the settings: the wrapper keeps them from the chip while it's borrowed
    radio_driver.setParams(_prefs->freq, _prefs->bw, _prefs->sf, _prefs->cr);
    radio_driver.setTxPower(_prefs->tx_power_dbm);
    radio_driver.setRxBoostedGainMode(_prefs->rx_boosted_gain);
    if (!ok) _host->spectrumAlert("Radio restore failed!", 3000);
  }

  void changeSpan(uint8_t idx, bool wipe = true) {
    _span_idx = idx % 4;
    _span_req = _span_idx;
    _cfg_gen++;
    if (wipe) _need_wipe = true;
    _peak = _hdr_peak = -1;
    if (_wf) {
      clearHistory();
      buildPixelMap();
    }
  }

public:
  SpectrumView(Host* host, NodePrefs* prefs)
      : _host(host), _prefs(prefs), _display(NULL), _open(false), _scanner(spectrumRadio(radio, std::is_base_of<SX1262, RADIO_CLASS>{})), _pending_start(false), _need_wipe(false), _span_idx(1),
        _center(0), _follow(true), _run(false), _task_running(false), _span_req(1), _cfg_gen(0), _cfg_applied(0), _wf_mem(NULL), _wf(NULL), _wf_err(NULL), _wf_err_next(NULL) {
    _points = 0;   // set from the panel's width on the first start()
  #if SPECTRUM_BLE
    streamReset();
  #endif
  }

  bool ownsRadio() const { return _scanner.isActive(); }
  void returnRadio() override { stop(); }   // wanted back at once (RadioLibWrapper::powerOff())
  bool isOpen() const { return _open; }   // between start() and stop()

  void start(DisplayDriver& d) { start(&d); }

  // d: NULL without a display (eg. none found, or none on the board): the sweeps only go to the stream client
  void start(DisplayDriver* d) {
    if (d == NULL) {
      _points = SPECTRUM_POINTS;
    } else if (_wf == NULL && !allocBuffers(*d)) {
      _host->spectrumAlert("Out of memory", 1500);
      return;
    }
    _freq = _prefs->freq;
    _bw = _prefs->bw;
    if (_follow) _center = _freq;
    _span_req = _span_idx;
    _sweep_err = RADIOLIB_ERR_NONE;
    _frame_noise = -115;
    _floor_dbm = -125;
    _peak = _hdr_peak = -1;
    _peak_mhz = _center;
    for (int i = 0; i < SPECTRUM_POINTS; i++) _cur[i] = _fmax[i] = SPECTRUM_NO_SIGNAL;
    if (d) clearHistory();
    _display = d;
    _open = true;
    _pending_start = true;   // wait for any in-flight TX to finish, see poll()
  }

  void stop() {
    if (!isOpen()) return;   // already stopped (or never started)
    _pending_start = false;
    if (_task_running && !stopScanTask()) {
      board.reboot();   // restoring the radio under the stuck task's feet could leave it in any state
    }
    if (_scanner.isActive()) restoreRadio();
    if (_display) {
      _display->wipe();   // the live view's picture to white, with its own waveform
      _display->setFastRefresh(false);
      _display->wipe();   // and then a clean panel for the next screen (on e-ink, its own full refresh)
    }
    freeBuffers();
    _open = false;
  }

  void poll() {
    if (_pending_start && !_scanner.hasRadio()) {   // the board has another radio
      _pending_start = false;
      _host->spectrumAlert("Needs an SX1262", 2000);
      _host->spectrumExit();
      return;
    }
    if (!_pending_start || !radio_driver.borrow(this)) return;   // not mid-TX, mid-RX, with a packet to read, or lent
    _pending_start = false;
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
    int16_t err = _scanner.begin(_freq, tcxo, ldo);
    _scanner.setGain(SPECTRUM_GAIN);
    if (err != RADIOLIB_ERR_NONE || !startScanTask()) {
      char tmp[32];
      sprintf(tmp, "Scan failed: %d", err);
      restoreRadio();
      _host->spectrumAlert(tmp, 2000);
      _host->spectrumExit();
      return;
    }
    _header_at = millis();
    if (_display) _display->setFastRefresh(true);
    _need_wipe = true;
  }

  int render(DisplayDriver& d) {
    uint32_t t0 = millis();
    d.setTextSize(1);
    d.setColor(UIColor::primary_txt);   // no title band behind it (title_txt is white on colour TFTs)
    if (!_scanner.isActive()) {
      d.drawTextCentered(d.width() / 2, 20, _pending_start ? "waiting for radio..." : "spectrum stopped");
      return 200;
    }
    if (_prefs->freq != _freq || _prefs->bw != _bw) {   // the node's channel changed (eg. from the app): follow it,
      _freq = _prefs->freq;                               // without a wipe of the panel
      _bw = _prefs->bw;
      if (_follow) {
        _center = _freq;
        _peak_mhz = _freq;
        changeSpan(_span_idx, false);
      }
    }
    if (_need_wipe) {   // start from a clean panel: on entry and when the span changes
      d.wipe();
      _need_wipe = false;
    }
    takeFrame(t0);
    int n = _points;
    // the loudest point of the last few frames (not of the peak-hold, which lingers for seconds), if it
    // stands out
    _peak = -1;
    int16_t peak_dbm = SPECTRUM_NO_SIGNAL;
    for (int i = 0; i < n; i++) {
      int16_t v = _frame[i];
      for (int f = 0; f < SPECTRUM_PEAK_FRAMES - 1; f++) if (_recent[f][i] > v) v = _recent[f][i];
      if (v > peak_dbm) { peak_dbm = v; _peak = i; }
    }
    if (SPECTRUM_PEAK_FRAMES > 1) {
      memcpy(_recent[_recent_head], _frame, sizeof(_frame));
      _recent_head = (_recent_head + 1) % (SPECTRUM_PEAK_FRAMES - 1);
    }
    if (peak_dbm < _frame_noise + SPECTRUM_PEAK_DB) _peak = -1;
    if (_peak >= 0 && (_hdr_peak < 0 || peak_dbm > _hdr_peak_dbm)) {
      _hdr_peak = _peak;
      _hdr_peak_dbm = peak_dbm;
    }
    // the number: the loudest signal since the last update, a few times a second (readable, and each change
    // is a little e-ink residue); with no signal it keeps the last one
    if (t0 - _header_at >= SPECTRUM_HEADER_MS) {
      _header_at = t0;
      if (_hdr_peak >= 0) _peak_mhz = edgeMHz(_span_idx) + spanMHz(_span_idx) * (_hdr_peak + 0.5f) / n;
      _hdr_peak = -1;
    }
    renderHeader(d);

    d.setColor(UIColor::primary_txt);
    renderGraph(d);
    renderWaterfall(d);

    int spent = millis() - t0;
    return spent < SPECTRUM_FRAME_MS ? SPECTRUM_FRAME_MS - spent : 0;
  }

  bool act(Action a) {
    if (a == ZOOM_OUT) {
      changeSpan(_span_idx + 1);
    } else if (a == ZOOM_IN) {
      changeSpan(_span_idx + 3);
    } else {
      _host->spectrumAlert("Mesh radio resumed", 1000);
      _host->spectrumExit();   // the host stops the view on the way
    }
    return true;
  }

#if SPECTRUM_BLE
  void setStream(Stream* stream) { _stream = stream; }

  // From the UI loop, whatever the screen: the stream client's requests, and the state it sees
  void streamLoop() {
    if (_stream == NULL) return;
    Stream::Request req;
    while (_stream->takeRequest(req)) {
      bool has_arg = req.arg != 0xFFFFFFFF;
      switch (req.op) {
        case Stream::CTRL_START:
          if (has_arg) setSpan(req.arg);
          if (!isOpen()) _host->spectrumOpen();
          break;
        case Stream::CTRL_STOP:
          if (isOpen()) {
            _host->spectrumAlert("Mesh radio resumed", 1000);
            _host->spectrumExit();
          }
          break;
        case Stream::CTRL_SPAN:
          if (has_arg) setSpan(req.arg);
          break;
        case Stream::CTRL_INTERVAL:
          if (has_arg && ((req.arg & 0xFFFF) == 0 || ((req.arg & 0xFFFF) >= 10 && (req.arg & 0xFFFF) <= 2000))) {
            _str_ms = req.arg & 0xFFFF;
          }
          break;
        case Stream::CTRL_AGG:
          break;   // AGG_MAX only
        case Stream::CTRL_CENTRE:
          if (!has_arg) break;
          if (req.arg == 0) {
            _follow = true;
            _center = _freq;
          } else if (req.arg >= SX126xSpectrumScanner::MIN_MHZ * 1e6f && req.arg <= SX126xSpectrumScanner::MAX_MHZ * 1e6f) {
            _follow = false;
            _center = req.arg / 1e6f;
          } else {
            break;
          }
          if (isOpen()) {
            _peak_mhz = _center;
            changeSpan(_span_idx, false);
          }
          break;
      }
    }
    publishInfo();
  }

private:
  void setSpan(uint32_t idx) {
    if (idx > 3) return;
    if (isOpen()) {
      if (idx != _span_idx) changeSpan(idx);
    } else {
      _span_idx = idx;
    }
  }

  static void put32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

  void publishInfo() {
    uint8_t info[Stream::INFO_LEN];
    memset(info, 0, sizeof(info));
    float node = _prefs->freq, center = !isOpen() && _follow ? node : _center;
    int n = _points ? _points : SPECTRUM_POINTS;   // the panel's width, once known
    double step = spanMHz(_span_idx) / n;
    info[0] = 1;
    info[1] = !isOpen() ? Stream::STATE_OFF : (_scanner.isActive() ? Stream::STATE_RUNNING : Stream::STATE_STARTING);
    info[2] = _span_idx;
    info[3] = n;
    put32(&info[4], (uint32_t)((center - spanMHz(_span_idx) / 2 + step / 2) * 1e6 + 0.5));
    put32(&info[8], (uint32_t)(step * 1e6 + 0.5));
    put32(&info[12], (uint32_t)(SX126xSpectrumScanner::pickRxBw(step * 1000.0 * spanRbwSteps(_span_idx)) * 1000 + 0.5f));
    put32(&info[16], (uint32_t)(center * 1e6 + 0.5));
    put32(&info[20], (uint32_t)(node * 1e6 + 0.5));
    put32(&info[24], (uint32_t)(_prefs->bw * 1000 + 0.5f));
    int16_t err = _sweep_err;
    info[28] = err;
    info[29] = err >> 8;
    info[30] = _cfg_gen;
    info[31] = (_follow ? Stream::FLAG_FOLLOW : 0) | (isOpen() && _host->spectrumOnDevice() ? Stream::FLAG_ON_DEVICE : 0);
    info[32] = _str_ms;
    info[33] = _str_ms >> 8;
    info[34] = Stream::AGG_MAX;
    _stream->setInfo(info);
  }
#endif
};
