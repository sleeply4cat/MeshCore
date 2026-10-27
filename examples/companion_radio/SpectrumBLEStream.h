#pragma once

#include "SpectrumView.h"
#include "SpectrumBLEService.h"

extern SpectrumBLEService spectrum_ble;   // main.cpp

// the BLE service as the spectrum view's stream client
struct SpectrumBLEStream : SpectrumView::Stream {
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
};
