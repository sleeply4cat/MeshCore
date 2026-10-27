#pragma once

#include <target.h>
#include "SpectrumBLEStream.h"

// The spectrum view where no UI has a page for it (no display, or ui-orig / ui-tiny): its sweeps go to the BLE
// client alone, which opens and closes it (main.cpp, SPECTRUM_HEADLESS), and it lasts while the client listens.
// The view borrows the radio from the mesh meanwhile (RadioLibWrapper::borrow()).
class SpectrumHeadless : SpectrumView::Host {
  SpectrumView _view;
  SpectrumBLEStream _ble;

public:
  SpectrumHeadless(NodePrefs* prefs) : _view(this, prefs) { _view.setStream(&_ble); }

  void spectrumAlert(const char* text, int duration_ms) override { }   // nowhere to show it: the client sees the state
  void spectrumExit() override { _view.stop(); }
  void spectrumOpen() override { _view.start((DisplayDriver*) NULL); }

  void loop() {
    _view.streamLoop();
    if (_view.isOpen() && !_ble.hasSubscribers()) _view.stop();   // the client is gone: the radio back to the mesh
    _view.poll();
  }
};
