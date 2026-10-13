#include "AutoOLEDDisplay.h"
#include <MeshCore.h>

// Color scheme (SSD1306_WHITE == SH110X_WHITE == 1)
ColorVal UIColor::window_bkg = SSD1306_BLACK;
ColorVal UIColor::title_bkg = SSD1306_BLACK;
ColorVal UIColor::title_txt = SSD1306_WHITE;
ColorVal UIColor::primary_txt = SSD1306_WHITE;
ColorVal UIColor::secondary_txt = SSD1306_WHITE;
ColorVal UIColor::warning_txt = SSD1306_WHITE;
ColorVal UIColor::popup_bkg = SSD1306_BLACK;
ColorVal UIColor::popup_txt = SSD1306_WHITE;
ColorVal UIColor::corp_blue = SSD1306_WHITE;

bool AutoOLEDDisplay::i2c_probe(TwoWire& wire, uint8_t addr) {
  wire.beginTransmission(addr);
  return wire.endTransmission() == 0;
}

// A read of the controller returns its status byte. The low nibble is a fixed ID: 0x08 (or 0x00) on the
// SH1106, 0x03..0x07 on the SSD1306 and its clones (SSD1315 included). Read until two reads agree.
AutoOLEDDisplay::Controller AutoOLEDDisplay::detect(TwoWire& wire, uint8_t addr) {
#if defined(OLED_CONTROLLER) && OLED_CONTROLLER == 1106
  return SH1106;
#elif defined(OLED_CONTROLLER) && OLED_CONTROLLER == 1306
  return SSD1306;
#else
  int r = -1, prev;
  for (int i = 0; i < 4; i++) {
    prev = r;
    wire.beginTransmission(addr);
    wire.write((uint8_t)0x00);   // control byte, no command
    if (wire.endTransmission() != 0) break;
    if (wire.requestFrom(addr, (uint8_t)1) != 1) break;
    r = wire.read() & 0x0F;
    if (r == prev) break;
  }
  if (r == 0x08 || r == 0x00) return SH1106;
  return SSD1306;   // 0x03..0x07, or no readable status (modules without SDA out): the common one
#endif
}

bool AutoOLEDDisplay::begin() {
  if (!_isOn) {
    if (_peripher_power) _peripher_power->claim();
    _isOn = true;
  }
  bool found = true;
  if (_ctrl == NONE) {   // the first time; on a power cycle the controller is already known
    if (i2c_probe(Wire, DISPLAY_ADDRESS)) {
      _addr = DISPLAY_ADDRESS;
    } else if (i2c_probe(Wire, DISPLAY_ADDRESS ^ 1)) {
      _addr = DISPLAY_ADDRESS ^ 1;
    } else {
      found = false;     // still begin a driver below: it allocates the frame buffer the UI draws into
    }
    _ctrl = found ? detect(Wire, _addr) : SSD1306;
    MESH_DEBUG_PRINTLN("AutoOLEDDisplay: %s at 0x%02X", found ? controllerName() : "nothing", _addr);
  }

  bool ok;
  if (_ctrl == SH1106) {
    ok = _sh.begin(_addr, true);
    _gfx = &_sh;
  } else {
    ok = _ssd.begin(SSD1306_SWITCHCAPVCC, _addr, true, false);
    _gfx = &_ssd;
  }
#ifdef DISPLAY_ROTATION
  _gfx->setRotation(DISPLAY_ROTATION);
#endif
  return found && ok;
}

void AutoOLEDDisplay::command(uint8_t c) {
  if (_ctrl == SH1106) _sh.oled_command(c);
  else _ssd.ssd1306_command(c);
}

void AutoOLEDDisplay::turnOn() {
  if (!_isOn) {
    if (_peripher_power) _peripher_power->claim();
    _isOn = true;    // set before begin() to prevent double claim
    if (_peripher_power) begin();   // re-init the panel after its power was cut
  }
  command(SSD1306_DISPLAYON);   // == SH110X_DISPLAYON (0xAF)
}

void AutoOLEDDisplay::turnOff() {
  command(SSD1306_DISPLAYOFF);  // == SH110X_DISPLAYOFF (0xAE)
  if (_isOn) {
    if (_peripher_power) {
#if PIN_OLED_RESET >= 0
      digitalWrite(PIN_OLED_RESET, LOW);
#endif
      _peripher_power->release();
    }
    _isOn = false;
  }
}

void AutoOLEDDisplay::clear() {
  if (_ctrl == SH1106) { _sh.clearDisplay(); _sh.display(); }
  else { _ssd.clearDisplay(); _ssd.display(); }
}

void AutoOLEDDisplay::startFrame(ColorVal bkg) {
  if (_ctrl == SH1106) _sh.clearDisplay();
  else _ssd.clearDisplay();   // TODO: apply 'bkg'
  _color = SSD1306_WHITE;
  _gfx->setTextColor(_color);
  _gfx->setTextSize(1);
  _gfx->cp437(true);          // Use full 256 char 'Code Page 437' font
}

void AutoOLEDDisplay::setTextSize(int sz) {
  _gfx->setTextSize(sz);
}

void AutoOLEDDisplay::setColor(ColorVal c) {
  _color = c;
  _gfx->setTextColor(_color);
}

void AutoOLEDDisplay::setCursor(int x, int y) {
  _gfx->setCursor(x, y);
}

void AutoOLEDDisplay::print(const char* str) {
  _gfx->print(str);
}

void AutoOLEDDisplay::fillRect(int x, int y, int w, int h) {
  _gfx->fillRect(x, y, w, h, _color);
}

void AutoOLEDDisplay::drawRect(int x, int y, int w, int h) {
  _gfx->drawRect(x, y, w, h, _color);
}

void AutoOLEDDisplay::drawXbm(int x, int y, const uint8_t* bits, int w, int h) {
  _gfx->drawBitmap(x, y, bits, w, h, _color);
}

uint16_t AutoOLEDDisplay::getTextWidth(const char* str) {
  int16_t x1, y1;
  uint16_t w, h;
  _gfx->getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
  return w;
}

void AutoOLEDDisplay::endFrame() {
  if (_ctrl == SH1106) _sh.display();
  else _ssd.display();
}
