#pragma once

#include "DisplayDriver.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#define SSD1306_NO_SPLASH
#include <Adafruit_SSD1306.h>
#define SH110X_NO_SPLASH
#include <Adafruit_SH110X.h>
#include <helpers/RefCountedDigitalPin.h>

#ifndef PIN_OLED_RESET
  #define PIN_OLED_RESET  -1
#endif
#ifndef DISPLAY_ADDRESS
  #define DISPLAY_ADDRESS 0x3C   // the other address of the pair (0x3D) is tried too
#endif
#ifndef OLED_HEIGHT
  #define OLED_HEIGHT     64     // the panel's rows can't be read from the controller; the companion UI expects 64
#endif

// 128-column monochrome I2C OLED of the kind sold for DIY nodes (eg. "faketec" ProMicro builds), whose
// controller varies from batch to batch: SSD1306 / SSD1315 (0.96") or SH1106 (1.3", 132-column RAM, shifted
// by 2 if driven as an SSD1306). The controller is told apart at begin() by its status byte (as Meshtastic
// does); -D OLED_CONTROLLER=1306 or =1106 skips the detection. Anything unrecognised is driven as an SSD1306.
class AutoOLEDDisplay : public DisplayDriver {
public:
  enum Controller : uint8_t { NONE = 0, SSD1306 = 1, SH1106 = 2 };

private:
  Adafruit_SSD1306 _ssd;   // only the detected one is begun: the frame buffer is allocated in begin()
  Adafruit_SH1106G _sh;
  Adafruit_GFX* _gfx;
  Controller _ctrl;
  uint8_t _addr;
  bool _isOn;
  uint8_t _color;
  RefCountedDigitalPin* _peripher_power;

  static bool i2c_probe(TwoWire& wire, uint8_t addr);
  static Controller detect(TwoWire& wire, uint8_t addr);
  void command(uint8_t c);

public:
  AutoOLEDDisplay(RefCountedDigitalPin* peripher_power = NULL)
      : DisplayDriver(128, OLED_HEIGHT),
        _ssd(128, OLED_HEIGHT, &Wire, PIN_OLED_RESET),
        _sh(128, OLED_HEIGHT, &Wire, PIN_OLED_RESET),
        _gfx(&_ssd), _ctrl(NONE), _addr(DISPLAY_ADDRESS), _isOn(false), _color(1), _peripher_power(peripher_power) { }

  bool begin();
  Controller controller() const { return _ctrl; }
  const char* controllerName() const { return _ctrl == SH1106 ? "SH1106" : (_ctrl == SSD1306 ? "SSD1306" : "none"); }

  bool isOn() override { return _isOn; }
  void turnOn() override;
  void turnOff() override;
  void clear() override;
  void startFrame(ColorVal bkg = UIColor::window_bkg) override;
  void setTextSize(int sz) override;
  void setColor(ColorVal c) override;
  void setCursor(int x, int y) override;
  void print(const char* str) override;
  void fillRect(int x, int y, int w, int h) override;
  void drawRect(int x, int y, int w, int h) override;
  void drawXbm(int x, int y, const uint8_t* bits, int w, int h) override;
  uint16_t getTextWidth(const char* str) override;
  int textLineHeight() override { return 8; }
  void endFrame() override;
};
