#pragma once

#include "target.h"   // board, radio_driver: for the view
#include <SpectrumView.h>
#include "CardputerScreen.h"

class CardputerUITask;

#if SPECTRUM_BLE
  #include <helpers/esp32/SpectrumBLEService.h>
  extern SpectrumBLEService spectrum_ble;   // main.cpp
#endif

// The companion's spectrum view (examples/companion_radio/SpectrumView.h) on the Cardputer: drawn into a sprite
// and shown in one push, as the panel is drawn on directly (clearing it for every frame would flicker). The sprite
// is in colour (a colour waterfall), or 1-bit where there's no memory for that.
// , and / zoom in and out, Esc or Enter leave it.
class SpectrumScreen : public CardputerScreen, SpectrumView::Host {
  // the sprite, as the display the view draws on
  class Canvas : public DisplayDriver {
    lgfx::LGFX_Sprite _spr;
    uint8_t _ink = 1;   // 1-bit: palette index, 0 background, 1 foreground
    uint16_t _ink565;   // colour: RGB565
    bool _created = false, _color = false;

    void fill(int x, int y, int w, int h) { if (_color) _spr.fillRect(x, y, w, h, _ink565); else _spr.fillRect(x, y, w, h, _ink); }

  public:
    explicit Canvas(lgfx::LGFX_Device* lcd) : DisplayDriver(240, 135), _spr(lcd) { }
    bool create();
    void push() { _spr.pushSprite(0, 0); }

    bool isOn() override { return true; }
    void turnOn() override { }
    void turnOff() override { }
    void clear() override { _spr.fillScreen(0); }
    void startFrame(ColorVal bkg = UIColor::window_bkg) override { _spr.fillScreen(0); setColor(UIColor::primary_txt); }
    void setTextSize(int sz) override { _spr.setTextSize(sz); }
    void setColor(ColorVal c) override;
    void setCursor(int x, int y) override { _spr.setCursor(x, y); }
    void print(const char* str) override { _spr.print(str); }
    void fillRect(int x, int y, int w, int h) override { fill(x, y, w, h); }
    void drawRect(int x, int y, int w, int h) override;
    void drawXbm(int x, int y, const uint8_t* bits, int w, int h) override;
    uint16_t getTextWidth(const char* str) override { return _spr.textWidth(str); }
    int textLineHeight() override { return 9; }
    bool hasColor() override { return _color; }
    void drawRgbRowNative(int x, int y, const uint16_t* rgb, int w) override;
    void endFrame() override { }
  };

#if SPECTRUM_BLE
  // the BLE service as the view's stream client
  struct BleStream : SpectrumView::Stream {
    bool hasSubscribers() override { return spectrum_ble.hasSubscribers(); }
    void setInfo(const uint8_t info[INFO_LEN]) override { spectrum_ble.setInfo(info); }
    bool sendFrame(uint8_t gen, const uint8_t* levels, uint8_t n) override { return spectrum_ble.sendFrame(gen, levels, n); }
    bool takeRequest(Request& req) override {
      SpectrumBLEService::Request r;
      if (!spectrum_ble.takeRequest(r)) return false;
      req.op = r.op;
      req.arg = r.arg;
      return true;
    }
  } _ble;
#endif
  CardputerUITask* _task;
  Canvas _canvas;
  SpectrumView _view;

public:
  SpectrumScreen(CardputerUITask* task, NodePrefs* prefs, CardputerDisplay& lcd)
      : _task(task), _canvas(lcd.device()), _view(this, prefs) {
  #if SPECTRUM_BLE
    _view.setStream(&_ble);
  #endif
  }

  bool start();   // false if there's no memory for it
  void stop() { _view.stop(); }
  bool ownsRadio() const { return _view.ownsRadio(); }

  void spectrumAlert(const char* text, int duration_ms) override;
  void spectrumExit() override;
  void spectrumOpen() override;   // a BLE client asks for the view
#if SPECTRUM_BLE
  void streamLoop() { _view.streamLoop(); }   // the BLE client's requests and the state it sees, from the UI loop
#endif

  bool drawsWholeFrame() override { return true; }
  int render(CardputerDisplay& lcd) override;
  bool handleInput(Keyboard::Event& e) override;
  void poll() override { _view.poll(); }
};
