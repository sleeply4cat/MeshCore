#include "SpectrumScreen.h"
#include "../CardputerUITask.h"

bool SpectrumScreen::Canvas::create() {
  if (_created) return true;
  _spr.setColorDepth(16);   // 65KB
  _color = _spr.createSprite(width(), height()) != nullptr;
  if (!_color) {
    _spr.setColorDepth(1);
    if (_spr.createSprite(width(), height()) == nullptr) return false;
    _spr.createPalette();
    _spr.setPaletteColor(0, (uint32_t)TFT_BLACK);
    _spr.setPaletteColor(1, (uint32_t)TFT_GREEN);
  }
  _spr.setFont(&fonts::Font0);
  _spr.setTextWrap(false);
  _created = true;
  return true;
}

void SpectrumScreen::Canvas::setColor(ColorVal c) {
  _ink = c == UIColor::window_bkg ? 0 : 1;
  _ink565 = _ink ? TFT_GREEN : TFT_BLACK;
  if (_color) _spr.setTextColor(_ink565);
  else _spr.setTextColor(_ink);
}

void SpectrumScreen::Canvas::drawRect(int x, int y, int w, int h) {
  if (_color) _spr.drawRect(x, y, w, h, _ink565);
  else _spr.drawRect(x, y, w, h, _ink);
}

void SpectrumScreen::Canvas::drawXbm(int x, int y, const uint8_t* bits, int w, int h) {
  if (_color) _spr.drawBitmap(x, y, bits, w, h, _ink565);
  else _spr.drawBitmap(x, y, bits, w, h, _ink);
}

void SpectrumScreen::Canvas::drawRgbRowNative(int x, int y, const uint16_t* rgb, int w) {
  if (!_color) return;
  for (int i = 0; i < w; i++) _spr.drawPixel(x + i, y, rgb[i]);   // uint16_t: RGB565
}

bool SpectrumScreen::start() {
  if (!_canvas.create()) {
    _task->showAlert("Out of memory", 1500);
    return false;
  }
  _view.start(_canvas);
  return true;
}

void SpectrumScreen::spectrumAlert(const char* text, int duration_ms) {
  _task->showAlert(text, duration_ms);
}

void SpectrumScreen::spectrumExit() {
  _task->gotoToolsScreen();   // stops the view, see CardputerUITask::setCurrScreen()
}

void SpectrumScreen::spectrumOpen() {
  _task->gotoSpectrumScreen();
}

int SpectrumScreen::render(CardputerDisplay& lcd) {
  _canvas.startFrame();
  int ms = _view.render(_canvas);
  _canvas.push();
  if (ownsRadio()) _task->extendAutoOff();   // a live view: no display off or light sleep under it
  return ms;
}

bool SpectrumScreen::handleInput(Keyboard::Event& e) {
  if (e.key == Keyboard::ARROW_LEFT) return _view.act(SpectrumView::ZOOM_IN);
  if (e.key == Keyboard::ARROW_RIGHT) return _view.act(SpectrumView::ZOOM_OUT);
  if (e.key == Keyboard::KEY_ESC || e.key == Keyboard::KEY_RETURN) return _view.act(SpectrumView::EXIT);
  return false;
}
