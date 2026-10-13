#include "UITask.h"
#include <helpers/TxtDataHelpers.h>
#include "../MyMesh.h"
#include "target.h"
#include <time.h>

#ifndef UI_SPECTRUM_PAGE   // the spectrum page, where its scan can run in a task of its own (FreeRTOS)
  #if defined(NRF52_PLATFORM) || defined(ESP32)
    #define UI_SPECTRUM_PAGE 1
  #endif
#endif
#ifdef ENABLE_WIFI_INTERFACE
  #include <WiFi.h>
#endif

#ifndef AUTO_OFF_MILLIS
  #define AUTO_OFF_MILLIS     15000   // 15 seconds
#endif
#define BOOT_SCREEN_MILLIS   3000   // 3 seconds

#ifdef PIN_STATUS_LED
#define LED_ON_MILLIS     20
#define LED_ON_MSG_MILLIS 200
#define LED_CYCLE_MILLIS  4000
#endif

#define LONG_PRESS_MILLIS   1200

// Used both for recent adverts and discovered nodes
#ifndef UI_RECENT_LIST_SIZE
  #define UI_RECENT_LIST_SIZE 4
#endif

#if UI_HAS_JOYSTICK || UI_HAS_ROTARY_INPUT
  #define PRESS_LABEL "press Enter"
#else
  #define PRESS_LABEL "long press"
#endif

#include "icons.h"

class SplashScreen : public UIScreen {
  UITask* _task;
  unsigned long dismiss_after;
  char _version_info[12];

public:
  SplashScreen(UITask* task) : _task(task) {
    // strip off dash and commit hash by changing dash to null terminator
    // e.g: v1.2.3-abcdef -> v1.2.3
    const char *ver = FIRMWARE_VERSION;
    const char *dash = strchr(ver, '-');

    int len = dash ? dash - ver : strlen(ver);
    if (len >= sizeof(_version_info)) len = sizeof(_version_info) - 1;
    memcpy(_version_info, ver, len);
    _version_info[len] = 0;

    dismiss_after = millis() + BOOT_SCREEN_MILLIS;
  }

  int render(DisplayDriver& display) override {
    // meshcore logo
    display.setColor(UIColor::corp_blue);
    int logoWidth = 128;
    display.drawXbm((display.width() - logoWidth) / 2, 3, meshcore_logo, logoWidth, 13);

    // meshcore website
    const char* website = "https://meshcore.io";
    display.setColor(UIColor::primary_txt);
    display.setTextSize(1);
    uint16_t websiteWidth = display.getTextWidth(website);
    display.setCursor((display.width() - websiteWidth) / 2, 22);
    display.print(website);

    // version info
    display.setColor(UIColor::primary_txt);
    display.setTextSize(1);
    display.drawTextCentered(display.width()/2, 35, _version_info);

    display.setColor(UIColor::secondary_txt);
    display.setTextSize(1);
    display.drawTextCentered(display.width()/2, 48, FIRMWARE_BUILD_DATE);

    return 1000;
  }

  void poll() override {
    if (millis() >= dismiss_after) {
      _task->gotoHomeScreen();
    }
  }
};

class HomeScreen : public UIScreen {
  enum HomePage {
    FIRST,
    RECENT,
    RADIO,
    BLUETOOTH,
    ADVERT,
#if ENV_INCLUDE_GPS == 1
    GPS,
#endif
#if UI_SENSORS_PAGE == 1
    SENSORS,
#endif
#if UI_DISCOVER_SCREEN
    DISCOVERY,
#endif
#if UI_SPECTRUM_PAGE == 1
    SPECTRUM,
#endif
#ifndef UI_NO_HIBERNATE
    SHUTDOWN,
#endif
    Count    // keep as last
  };

  UITask* _task;
  mesh::RTCClock* _rtc;
  SensorManager* _sensors;
  NodePrefs* _node_prefs;
  uint8_t _page;
  bool _shutdown_init;
  AdvertPath recent[UI_RECENT_LIST_SIZE];
#if UI_DISCOVER_SCREEN
  DiscoveredNode discovered[DISCOVERED_NODES_TABLE_SIZE]; // not circular, latest discovered nodes are not kept
  uint32_t disc_node_req_tag = 0;
  uint32_t disc_nodes_count = 0;
  uint32_t discovery_req_time = 0;
  bool discovery_disp_names = true; // by default desplay names if available (removes SNR_O)
#endif

  void renderBatteryIndicator(DisplayDriver& display, uint16_t batteryMilliVolts) {
    // Convert millivolts to percentage
#ifndef BATT_MIN_MILLIVOLTS
  #define BATT_MIN_MILLIVOLTS 3000
#endif
#ifndef BATT_MAX_MILLIVOLTS
  #define BATT_MAX_MILLIVOLTS 4200
#endif
    const int minMilliVolts = BATT_MIN_MILLIVOLTS;
    const int maxMilliVolts = BATT_MAX_MILLIVOLTS;
    int batteryPercentage = ((batteryMilliVolts - minMilliVolts) * 100) / (maxMilliVolts - minMilliVolts);
    if (batteryPercentage < 0) batteryPercentage = 0; // Clamp to 0%
    if (batteryPercentage > 100) batteryPercentage = 100; // Clamp to 100%

    // battery icon
    int iconWidth = 24;
    int iconHeight = 10;
    int iconX = display.width() - iconWidth - 5; // Position the icon near the top-right corner
    int iconY = 0;
    display.setColor(UIColor::title_txt);

    // battery outline
    display.drawRect(iconX, iconY, iconWidth, iconHeight);

    // battery "cap"
    display.fillRect(iconX + iconWidth, iconY + (iconHeight / 4), 3, iconHeight / 2);

    // fill the battery based on the percentage
    int fillWidth = (batteryPercentage * (iconWidth - 4)) / 100;
    display.fillRect(iconX + 2, iconY + 2, fillWidth, iconHeight - 4);

    // while charging, show a bolt (or a plug once full) just left of the battery,
    // keeping the fill bar itself clean and uninterrupted
    bool charging = board.isExternalPowered();
    if (charging) {
      // There's no charge-complete signal on most boards, so "full" is a high
      // voltage band rather than an exact 100% (a real pack rarely reads 4.2V).
      const int BATT_FULL_PCT = 95;
      const uint8_t* symbol = (batteryPercentage >= BATT_FULL_PCT) ? plug_icon : charging_icon;
      display.setColor(UIColor::title_txt);
      display.drawXbm(iconX - 9, iconY + 1, symbol, 8, 8);
    }

    // show muted icon if buzzer is muted (shifted further left when the charging
    // icon already occupies the slot immediately left of the battery)
#ifdef PIN_BUZZER
    if (_task->isBuzzerQuiet()) {
      display.setColor(UIColor::warning_txt);
      display.drawXbm(iconX - (charging ? 18 : 9), iconY + 1, muted_icon, 8, 8);
    }
#endif
  }

  CayenneLPP sensors_lpp;
  int sensors_nb = 0;
  bool sensors_scroll = false;
  int sensors_scroll_offset = 0;
  int next_sensors_refresh = 0;

  void refresh_sensors() {
    if (millis() > next_sensors_refresh) {
      sensors_lpp.reset();
      sensors_nb = 0;
      sensors_lpp.addVoltage(TELEM_CHANNEL_SELF, (float)board.getBattMilliVolts() / 1000.0f);
      sensors.querySensors(0xFF, sensors_lpp);
      LPPReader reader (sensors_lpp.getBuffer(), sensors_lpp.getSize());
      uint8_t channel, type;
      while(reader.readHeader(channel, type)) {
        reader.skipData(type);
        sensors_nb ++;
      }
      sensors_scroll = sensors_nb > UI_RECENT_LIST_SIZE;
#if AUTO_OFF_MILLIS > 0
      next_sensors_refresh = millis() + 5000; // refresh sensor values every 5 sec
#else
      next_sensors_refresh = millis() + 60000; // refresh sensor values every 1 min
#endif
    }
  }

public:
  HomeScreen(UITask* task, mesh::RTCClock* rtc, SensorManager* sensors, NodePrefs* node_prefs)
     : _task(task), _rtc(rtc), _sensors(sensors), _node_prefs(node_prefs), _page(0),
       _shutdown_init(false), sensors_lpp(200) {  }

#if UI_DISCOVER_SCREEN
  bool sendDiscoverRequest() {
    uint8_t cmd_bytes[6];
    cmd_bytes[0] = CTL_TYPE_NODE_DISCOVER_REQ | 1; // DISCOVER_REQ | prefix only
    cmd_bytes[1] = 0xFF;     // Repeaters
    the_mesh.getRNG()->random((uint8_t *) &disc_node_req_tag, 4);  // generate random tag
    memcpy(&cmd_bytes[2], &disc_node_req_tag, 4);
    disc_nodes_count = 0;
    mesh::Packet* req = the_mesh.createControlData(cmd_bytes, sizeof(cmd_bytes));
    if (req) {
      the_mesh.sendZeroHop(req);
      discovery_req_time = millis();
      return true;
    }
    return false;
  }

  void handleDiscoverResponse(const mesh::Packet* packet) {
    if (disc_nodes_count < DISCOVERED_NODES_TABLE_SIZE && memcmp(&packet->payload[2], &disc_node_req_tag, 4) == 0) {
      auto d = &discovered[disc_nodes_count++];
      memcpy(d->pubkey_prefix, &packet->payload[6], 8);
      d->type = packet->payload[0] & 0xF;
      d->snr_out = ((int8_t)packet->payload[1]) / 4.0;
      d->snr_in = radio_driver.getLastSNR();
      ContactInfo* c = the_mesh.lookupContactByPubKey(&packet->payload[6], 8);
      if (c != NULL) {
        strncpy(d->name, c->name, 32);
      } else {
        d->name[0] = 0;
      }
    }
  }
#endif

  void poll() override {
    if (_shutdown_init && !_task->isButtonPressed()) {  // must wait for USR button to be released
      _task->shutdown();
    }
  }

  int render(DisplayDriver& display) override {
    display.setColor(UIColor::title_bkg);
    display.fillRect(0, 0, display.width(), 12);
    char tmp[80];
    // node name
    display.setTextSize(1);
    display.setColor(UIColor::title_txt);
    char filtered_name[sizeof(_node_prefs->node_name)];
    display.translateUTF8ToBlocks(filtered_name, _node_prefs->node_name, sizeof(filtered_name));
    display.setCursor(0, 2);
    display.print(filtered_name);

    // battery voltage
    renderBatteryIndicator(display, _task->getBattMilliVolts());

    // curr page indicator
    if (UIColor::title_bkg == UIColor::window_bkg) {
      display.setColor(UIColor::title_txt);
    } else {
      display.setColor(UIColor::title_bkg);
    }
    int y = 14;
    int x = display.width() / 2 - 5 * (HomePage::Count-1);
    for (uint8_t i = 0; i < HomePage::Count; i++, x += 10) {
      if (i == _page) {
        display.fillRect(x-1, y-1, 4, 4);
      } else {
        display.fillRect(x, y, 2, 2);
      }
    }

    if (_page == HomePage::FIRST) {
      display.setColor(UIColor::primary_txt);
      #ifdef ENABLE_WIFI_INTERFACE
      // smaller unread message ui for wifi variants
      display.setTextSize(1);
      sprintf(tmp, "Unread Msg: %d", _task->getMsgCount());
      display.drawTextCentered(display.width() / 2, 22, tmp);
      #else
      // larger unread message ui for non wifi variants
      display.setTextSize(2);
      sprintf(tmp, "MSG: %d", _task->getMsgCount());
      display.drawTextCentered(display.width() / 2, 22, tmp);
      #endif
      #ifdef UI_SHOW_CLOCK
      display.setTextSize(3);
      uint32_t now = _rtc->getCurrentTime();
      now += (int32_t)_node_prefs->tz_offset * 3600;
      DateTime dt (now);
      sprintf(tmp, "%02d:%02d", dt.hour(), dt.minute());
      display.drawTextCentered(display.width() / 2, 60, tmp);
      display.setTextSize(1);
      sprintf(tmp, "%02d/%02d/%d", dt.day(), dt.month(), dt.year());
      display.drawTextCentered(display.width() / 2, 80, tmp);
      #endif
      #ifdef ENABLE_WIFI_INTERFACE
        IPAddress ip = WiFi.localIP();
        snprintf(tmp, sizeof(tmp), "IP: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
        display.setTextSize(1);
        display.drawTextCentered(display.width() / 2, 32, tmp);
      #endif
      if (_task->hasConnection()) {
        display.setColor(UIColor::warning_txt);
        display.setTextSize(1);
        #ifdef UI_SHOW_CLOCK
        display.drawTextCentered(display.width() / 2, 110, "< Connected >");
        #else
        display.drawTextCentered(display.width() / 2, 43, "< Connected >");
        #endif
      } else if (the_mesh.getBLEPin() != 0) { // BT pin
        display.setColor(UIColor::warning_txt);
        sprintf(tmp, "Pin:%d", the_mesh.getBLEPin());
        #ifdef UI_SHOW_CLOCK
        display.setTextSize(1);
        display.drawTextCentered(display.width() / 2, 110, tmp);
        #else
        display.setTextSize(2);
        display.drawTextCentered(display.width() / 2, 43, tmp);
        #endif
      }
    } else if (_page == HomePage::RECENT) {
      the_mesh.getRecentlyHeard(recent, UI_RECENT_LIST_SIZE);
      display.setColor(UIColor::primary_txt);
      int y = 20;
      for (int i = 0; i < UI_RECENT_LIST_SIZE; i++, y += 11) {
        auto a = &recent[i];
        if (a->name[0] == 0) continue;  // empty slot
        int secs = _rtc->getCurrentTime() - a->recv_timestamp;
        if (secs < 60) {
          sprintf(tmp, "%ds", secs);
        } else if (secs < 60*60) {
          sprintf(tmp, "%dm", secs / 60);
        } else {
          sprintf(tmp, "%dh", secs / (60*60));
        }

        int timestamp_width = display.getTextWidth(tmp);
        int max_name_width = display.width() - timestamp_width - 1;

        char filtered_recent_name[sizeof(a->name)];
        display.translateUTF8ToBlocks(filtered_recent_name, a->name, sizeof(filtered_recent_name));
        display.drawTextEllipsized(0, y, max_name_width, filtered_recent_name);
        display.setCursor(display.width() - timestamp_width - 1, y);
        display.print(tmp);
      }
    } else if (_page == HomePage::RADIO) {
      display.setColor(UIColor::primary_txt);
      display.setTextSize(1);
      // freq / sf
      display.setCursor(0, 20);
      sprintf(tmp, "FQ: %06.3f   SF: %d", _node_prefs->freq, _node_prefs->sf);
      display.print(tmp);

      display.setCursor(0, 31);
      sprintf(tmp, "BW: %03.2f     CR: %d", _node_prefs->bw, _node_prefs->cr);
      display.print(tmp);

      // tx power,  noise floor
      display.setCursor(0, 42);
      sprintf(tmp, "TX: %ddBm", _node_prefs->tx_power_dbm);
      display.print(tmp);
      display.setCursor(0, 53);
      sprintf(tmp, "Noise floor: %d", radio_driver.getNoiseFloor());
      display.print(tmp);
    } else if (_page == HomePage::BLUETOOTH) {
      display.setColor(UIColor::corp_blue);
      display.drawXbm((display.width() - 32) / 2, 18,
          _task->isBluetoothEnabled() ? bluetooth_on : bluetooth_off,
          32, 32);
      display.setColor(UIColor::secondary_txt);
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 64 - 11, "toggle: " PRESS_LABEL);
    } else if (_page == HomePage::ADVERT) {
      display.setColor(UIColor::corp_blue);
      display.drawXbm((display.width() - 32) / 2, 18, advert_icon, 32, 32);
      display.setColor(UIColor::secondary_txt);
      display.drawTextCentered(display.width() / 2, 64 - 11, "advert: " PRESS_LABEL);
#if ENV_INCLUDE_GPS == 1
    } else if (_page == HomePage::GPS) {
      LocationProvider* nmea = sensors.getLocationProvider();
      char buf[50];
      int y = 18;
      bool gps_state = _task->getGPSState();
#ifdef PIN_GPS_SWITCH
      bool hw_gps_state = digitalRead(PIN_GPS_SWITCH);
      if (gps_state != hw_gps_state) {
        strcpy(buf, gps_state ? "gps off(hw)" : "gps off(sw)");
      } else {
        strcpy(buf, gps_state ? "gps on" : "gps off");
      }
#else
      strcpy(buf, gps_state ? "gps on" : "gps off");
#endif
      display.setColor(UIColor::primary_txt);
      display.drawTextLeftAlign(0, y, buf);
      if (nmea == NULL) {
        y = y + 12;
        display.setColor(UIColor::secondary_txt);
        display.drawTextLeftAlign(0, y, "Can't access GPS");
      } else {
        display.setColor(UIColor::primary_txt);
        strcpy(buf, nmea->isValid()?"fix":"no fix");
        display.drawTextRightAlign(display.width()-1, y, buf);
        y = y + 12;
        display.setColor(UIColor::secondary_txt);
        display.drawTextLeftAlign(0, y, "sat");
        display.setColor(UIColor::primary_txt);
        sprintf(buf, "%d", nmea->satellitesCount());
        display.drawTextRightAlign(display.width()-1, y, buf);
        y = y + 12;
        display.setColor(UIColor::secondary_txt);
        display.drawTextLeftAlign(0, y, "pos");
        display.setColor(UIColor::primary_txt);
        sprintf(buf, "%.4f %.4f",
          nmea->getLatitude()/1000000., nmea->getLongitude()/1000000.);
        display.drawTextRightAlign(display.width()-1, y, buf);
        y = y + 12;
        display.setColor(UIColor::secondary_txt);
        display.drawTextLeftAlign(0, y, "alt");
        display.setColor(UIColor::primary_txt);
        sprintf(buf, "%.2f", nmea->getAltitude()/1000.);
        display.drawTextRightAlign(display.width()-1, y, buf);
        y = y + 12;
      }
#endif
#if UI_SENSORS_PAGE == 1
    } else if (_page == HomePage::SENSORS) {
      int y = 18;
      refresh_sensors();
      char buf[30];
      char name[30];
      LPPReader r(sensors_lpp.getBuffer(), sensors_lpp.getSize());

      for (int i = 0; i < sensors_scroll_offset; i++) {
        uint8_t channel, type;
        r.readHeader(channel, type);
        r.skipData(type);
      }

      for (int i = 0; i < (sensors_scroll?UI_RECENT_LIST_SIZE:sensors_nb); i++) {
        uint8_t channel, type;
        if (!r.readHeader(channel, type)) { // reached end, reset
          r.reset();
          r.readHeader(channel, type);
        }

        display.setCursor(0, y);
        float v;
        switch (type) {
          case LPP_GPS: // GPS
            float lat, lon, alt;
            r.readGPS(lat, lon, alt);
            strcpy(name, "gps"); sprintf(buf, "%.4f %.4f", lat, lon);
            break;
          case LPP_VOLTAGE:
            r.readVoltage(v);
            strcpy(name, "voltage"); sprintf(buf, "%6.2f", v);
            break;
          case LPP_CURRENT:
            r.readCurrent(v);
            strcpy(name, "current"); sprintf(buf, "%.3f", v);
            break;
          case LPP_TEMPERATURE:
            r.readTemperature(v);
            strcpy(name, "temperature"); sprintf(buf, "%.2f", v);
            break;
          case LPP_RELATIVE_HUMIDITY:
            r.readRelativeHumidity(v);
            strcpy(name, "humidity"); sprintf(buf, "%.2f", v);
            break;
          case LPP_BAROMETRIC_PRESSURE:
            r.readPressure(v);
            strcpy(name, "pressure"); sprintf(buf, "%.2f", v);
            break;
          case LPP_ALTITUDE:
            r.readAltitude(v);
            strcpy(name, "altitude"); sprintf(buf, "%.0f", v);
            break;
          case LPP_POWER:
            r.readPower(v);
            strcpy(name, "power"); sprintf(buf, "%6.2f", v);
            break;
          default:
            r.skipData(type);
            strcpy(name, "unk"); sprintf(buf, "");
        }
        display.setCursor(0, y);
        display.setColor(UIColor::secondary_txt);
        display.print(name);
        display.setColor(UIColor::primary_txt);
        display.setCursor(
          display.width()-display.getTextWidth(buf)-1, y
        );
        display.print(buf);
        y = y + 12;
      }
      if (sensors_scroll) sensors_scroll_offset = (sensors_scroll_offset+1)%sensors_nb;
      else sensors_scroll_offset = 0;
#endif
#if UI_DISCOVER_SCREEN
    } else if (_page == HomePage::DISCOVERY) {
      int count = disc_nodes_count;
      display.setColor(UIColor::primary_txt);
      int y = 20;
      for (int i = 0; i < count; i++, y += 11) {
        char name[32];
        auto a = &discovered[i];
        if ((a->name[0] == 0) || !discovery_disp_names) {
          mesh::Utils::toHex(name, a->pubkey_prefix, 4);
        } else {
          strncpy(name, a->name, 32);
        }
        char filtered_name[sizeof(name)];
        char snr_s[12];
        if (strlen(name) <= 8) { // display snr_o
          sprintf(snr_s, "%02.1f>%02.1f", a->snr_out, a->snr_in);
        } else {
          sprintf(snr_s, "%02.1f", a->snr_in);
        }
        int snr_width = display.getTextWidth(snr_s);
        int max_name_width = display.width() - snr_width - 1;
        display.translateUTF8ToBlocks(filtered_name, name, sizeof(filtered_name));
        display.drawTextEllipsized(0, y, max_name_width, filtered_name);
        display.setCursor(display.width() - snr_width - 1, y);
        display.print(snr_s);
      }
      if (millis() < discovery_req_time + 5000) {
        return 1000; // more frequent updates just after req
      } else if (count < DISCOVERED_NODES_TABLE_SIZE -1) { // show only 5 sec after last disc
        y = 10 + 11 * DISCOVERED_NODES_TABLE_SIZE;
        display.drawTextCentered(display.width() / 2, y, "discover: " PRESS_LABEL);
      }
#endif
#if UI_SPECTRUM_PAGE == 1
    } else if (_page == HomePage::SPECTRUM) {
      // little spectrum glyph
      static const uint8_t bars[] = { 2, 3, 2, 4, 3, 5, 9, 14, 9, 5, 3, 2, 3, 4, 2, 3 };
      display.setColor(UIColor::corp_blue);
      int x0 = (display.width() - 32) / 2;
      for (int i = 0; i < sizeof(bars); i++) {
        display.fillRect(x0 + i * 2, 40 - bars[i] * 2, 2, bars[i] * 2);
      }
      display.fillRect(x0, 40, 32, 1);
      display.setColor(UIColor::secondary_txt);
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 64 - 11, "spectrum: " PRESS_LABEL);
#endif
#ifndef UI_NO_HIBERNATE
    } else if (_page == HomePage::SHUTDOWN) {
      display.setColor(UIColor::corp_blue);
      display.setTextSize(1);
      if (_shutdown_init) {
        display.setColor(UIColor::warning_txt);
        display.drawTextCentered(display.width() / 2, 34, "hibernating...");
      } else {
        display.setColor(UIColor::secondary_txt);
        display.drawXbm((display.width() - 32) / 2, 18, power_icon, 32, 32);
        display.drawTextCentered(display.width() / 2, 64 - 11, "hibernate:" PRESS_LABEL);
      }
#endif
    }
    return 5000;   // next render after 5000 ms
  }

  bool handleInput(char c) override {
    if (c == KEY_LEFT || c == KEY_PREV) {
      _page = (_page + HomePage::Count - 1) % HomePage::Count;
      return true;
    }
    if (c == KEY_NEXT || c == KEY_RIGHT) {
      _page = (_page + 1) % HomePage::Count;
      if (_page == HomePage::RECENT) {
        _task->showAlert("Recent adverts", 800);
      }
#if UI_DISCOVER_SCREEN
      if (_page == HomePage::DISCOVERY) {
        _task->showAlert("Repeater disc", 800);
      }
#endif
      return true;
    }
    if (c == KEY_ENTER && _page == HomePage::BLUETOOTH) {
      if (_task->isBluetoothEnabled()) {  // toggle Bluetooth on/off
        _task->disableBluetooth();
      } else {
        _task->enableBluetooth();
      }
      return true;
    }
    if (c == KEY_ENTER && _page == HomePage::ADVERT) {
      _task->notify(UIEventType::ack);
      if (the_mesh.advert()) {
        _task->showAlert("Advert sent!", 1000);
      } else {
        _task->showAlert("Advert failed..", 1000);
      }
      return true;
    }
#if ENV_INCLUDE_GPS == 1
    if (c == KEY_ENTER && _page == HomePage::GPS) {
      _task->toggleGPS();
      return true;
    }
#endif
#if UI_SENSORS_PAGE == 1
    if (c == KEY_ENTER && _page == HomePage::SENSORS) {
      _task->toggleGPS();
      next_sensors_refresh=0;
      return true;
    }
#endif
#if UI_DISCOVER_SCREEN
    if (c == KEY_ENTER && _page == HomePage::DISCOVERY) {
      if (millis() > discovery_req_time + 5000) { // rate limiter
        sendDiscoverRequest();
      }
      return true;
    }
    if (c == KEY_SELECT && _page == HomePage::DISCOVERY) {
      discovery_disp_names = !discovery_disp_names;
      return true;
    }
#endif
#if UI_SPECTRUM_PAGE == 1
    if (c == KEY_ENTER && _page == HomePage::SPECTRUM) {
      _task->gotoSpectrumScreen();
      return true;
    }
#endif
#ifndef UI_NO_HIBERNATE
    if (c == KEY_ENTER && _page == HomePage::SHUTDOWN) {
      _shutdown_init = true;  // need to wait for button to be released
      return true;
    }
#endif
    return false;
  }
};

#ifndef UI_MSG_PREVIEW_SIZE
  #define UI_MSG_PREVIEW_SIZE 78
#endif

class MsgPreviewScreen : public UIScreen {
  UITask* _task;
  mesh::RTCClock* _rtc;

  struct MsgEntry {
    uint32_t timestamp;
    char origin[62];
    char msg[UI_MSG_PREVIEW_SIZE];
  };
  #define MAX_UNREAD_MSGS   32
  int num_unread;
  int head = MAX_UNREAD_MSGS - 1; // index of latest unread message
  MsgEntry unread[MAX_UNREAD_MSGS];

public:
  MsgPreviewScreen(UITask* task, mesh::RTCClock* rtc) : _task(task), _rtc(rtc) { num_unread = 0; }

  void addPreview(uint8_t path_len, const char* from_name, const char* msg) {
    head = (head + 1) % MAX_UNREAD_MSGS;
    if (num_unread < MAX_UNREAD_MSGS) num_unread++;

    auto p = &unread[head];
    p->timestamp = _rtc->getCurrentTime();
    if (path_len == 0xFF) {
      sprintf(p->origin, "(D) %s:", from_name);
    } else {
      sprintf(p->origin, "(%d) %s:", (uint32_t) path_len, from_name);
    }
    StrHelper::strncpy(p->msg, msg, sizeof(p->msg));
  }

  int render(DisplayDriver& display) override {
    char tmp[16];
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setColor(UIColor::corp_blue);
    sprintf(tmp, "Unread: %d", num_unread);
    display.print(tmp);

    auto p = &unread[head];

    int secs = _rtc->getCurrentTime() - p->timestamp;
    if (secs < 60) {
      sprintf(tmp, "%ds", secs);
    } else if (secs < 60*60) {
      sprintf(tmp, "%dm", secs / 60);
    } else {
      sprintf(tmp, "%dh", secs / (60*60));
    }
    display.setCursor(display.width() - display.getTextWidth(tmp) - 2, 0);
    display.print(tmp);

    display.drawRect(0, 11, display.width(), 1);  // horiz line

    display.setCursor(0, 14);
    display.setColor(UIColor::secondary_txt);
    char filtered_origin[sizeof(p->origin)];
    display.translateUTF8ToBlocks(filtered_origin, p->origin, sizeof(filtered_origin));
    display.print(filtered_origin);

    display.setCursor(0, 25);
    display.setColor(UIColor::primary_txt);
    char filtered_msg[sizeof(p->msg)];
    display.translateUTF8ToBlocks(filtered_msg, p->msg, sizeof(filtered_msg));
    display.printWordWrap(filtered_msg, display.width());

#if AUTO_OFF_MILLIS==0 // probably e-ink
    return 10000; // 10 s
#else
    return 1000;  // next render after 1000 ms
#endif
  }

  bool handleInput(char c) override {
    if (c == KEY_NEXT || c == KEY_RIGHT) {
      head = (head + MAX_UNREAD_MSGS - 1) % MAX_UNREAD_MSGS;
      num_unread--;
      if (num_unread == 0) {
        _task->gotoHomeScreen();
      }
      return true;
    }
    if (c == KEY_ENTER) {
      num_unread = 0;  // clear unread queue
      _task->gotoHomeScreen();
      return true;
    }
    return false;
  }
};

#if UI_SPECTRUM_PAGE == 1
#include <helpers/radiolib/SX126xSpectrumScanner.h>
#include <type_traits>

extern RADIO_CLASS radio;   // in every variant's target.cpp

// the radio as an SX1262, or NULL for other radios: the page builds with all of them, and only scans there
template <class R> static SX1262* spectrumRadio(R& r, std::true_type) { return &r; }
template <class R> static SX1262* spectrumRadio(R&, std::false_type) { return NULL; }

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
#ifndef SPECTRUM_AUTO_OFF_MILLIS
  #define SPECTRUM_AUTO_OFF_MILLIS 300000   // with display auto-off: idle time after which the view hands the radio back
#endif
#define SPECTRUM_MIN_WF_ROWS 8       // below that, no waterfall: the graph gets the whole height
#define SPECTRUM_NO_SIGNAL   -140
#define SPECTRUM_RANGE_DB    50      // vertical range of the graph
#define SPECTRUM_WF_LO       3       // waterfall: dB above noise where the grey scale starts
#define SPECTRUM_WF_RANGE    32      //            and its span in dB, up to solid black

// SDR-like view of the band around the mesh frequency: live spectrum with decaying peak-hold, plus a waterfall.
// Sweeps run continuously in their own RTOS task, so they don't stop while the display is drawn or refreshed;
// each display frame shows the maximum seen at each point since the previous frame, so short bursts aren't lost.
// The mesh is paused while this screen owns the radio (the app is still served).
class SpectrumScreen : public UIScreen {
  UITask* _task;
  NodePrefs* _prefs;
  DisplayDriver* _display;
  SX126xSpectrumScanner _scanner;
  bool _pending_start, _need_wipe;
  uint8_t _span_idx;
  float _freq, _bw;      // the mesh channel shown; follows the node's prefs (render())
  uint32_t _last_input;  // for the idle timeout, on displays that turn off by themselves

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
  float edgeMHz(uint8_t idx) const { return _freq - spanMHz(idx) / 2; }

  // --- scan task ---

  static void scanTaskEntry(void* self) { ((SpectrumScreen *) self)->scanLoop(); }

  void applySpan(uint8_t idx) {
    int n = _points;
    float step = spanMHz(idx) / n;
    _scanner.setSweep(edgeMHz(idx) + step / 2, step, n, step * 1000.0f * spanRbwSteps(idx));
    for (int i = 0; i < SPECTRUM_POINTS; i++) _cur[i] = _fmax[i] = SPECTRUM_NO_SIGNAL;
  }

  void scanLoop() {
    uint8_t level[SPECTRUM_POINTS];
    while (_run) {
      uint8_t gen = _cfg_gen;
      if (gen != _cfg_applied) {
        applySpan(_span_req);
        _cfg_applied = gen;
      }
      int16_t err = _scanner.sweep(level);   // the whole sweep runs in the radio
      _sweep_err = err;
      if (err != RADIOLIB_ERR_NONE) {     // shown in the header; keep trying
        delay(10);
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
    for (int y = _graph_y; y < bottom; y += 3) d.fillRectNative(cx, y, 1, 1);
    int half_bw = (int)(_bw / 2000.0f / spanMHz(_span_idx) * _w);
    d.fillRectNative(cx - half_bw, bottom + 1, half_bw * 2 + 1, 2);
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
      sprintf(left, span < 1 ? "%.3f %.1fM" : "%.3f %.0fM", _freq, span);
    }
    if (d.getTextWidth(left) + gap + wr <= d.width()) {   // else only the error: it matters more
      d.setCursor(0, 0);
      d.print(left);
    }
    d.drawTextRightAlign(d.width() - 1, 0, right);
  }

  void restoreRadio() {
    _scanner.end();
    // begin() hardware-resets the SX126x, which also drops the PRAM plugin
    bool ok = radio_init();
    radio_driver.setParams(_prefs->freq, _prefs->bw, _prefs->sf, _prefs->cr);
    radio_driver.setTxPower(_prefs->tx_power_dbm);
    radio_driver.setRxBoostedGainMode(_prefs->rx_boosted_gain);
    radio_driver.onExternalReinit();
    if (!ok) _task->showAlert("Radio restore failed!", 3000);
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
  SpectrumScreen(UITask* task, NodePrefs* prefs)
      : _task(task), _prefs(prefs), _display(NULL), _scanner(spectrumRadio(radio, std::is_base_of<SX1262, RADIO_CLASS>{})), _pending_start(false), _need_wipe(false), _span_idx(1),
        _run(false), _task_running(false), _span_req(1), _cfg_gen(0), _cfg_applied(0), _wf_mem(NULL), _wf(NULL), _wf_err(NULL), _wf_err_next(NULL) { }

  bool ownsRadio() const { return _scanner.isActive(); }

  void start(DisplayDriver& d) {
    if (_wf == NULL && !allocBuffers(d)) {
      _task->showAlert("Out of memory", 1500);
      return;
    }
    _freq = _prefs->freq;
    _bw = _prefs->bw;
    _last_input = millis();
    _sweep_err = RADIOLIB_ERR_NONE;
    _frame_noise = -115;
    _floor_dbm = -125;
    _peak = _hdr_peak = -1;
    _peak_mhz = _freq;
    for (int i = 0; i < SPECTRUM_POINTS; i++) _cur[i] = _fmax[i] = SPECTRUM_NO_SIGNAL;
    clearHistory();
    _display = &d;
    _pending_start = true;   // wait for any in-flight TX to finish, see poll()
  }

  void stop() {
    _pending_start = false;
    if (_task_running && !stopScanTask()) {
      board.reboot();   // restoring the radio under the stuck task's feet could leave it in any state
    }
    if (_scanner.isActive()) restoreRadio();
    if (_display) {
      _display->wipe();   // leave no ghosts behind for the next screen
      _display->setFastRefresh(false);
    }
    freeBuffers();
  }

  void poll() override {
  #if AUTO_OFF_MILLIS > 0
    // where the display turns itself off (OLEDs), the view would hold the mesh off for good if forgotten:
    // it keeps the display on, and hands the radio back after a while without input
    if (_scanner.isActive() && millis() - _last_input > SPECTRUM_AUTO_OFF_MILLIS) {
      _task->showAlert("Mesh radio resumed", 1500);
      _task->gotoHomeScreen();   // stops the scan, see UITask::setCurrScreen()
      return;
    }
  #endif
    if (_pending_start && !_scanner.hasRadio()) {   // the board has another radio
      _pending_start = false;
      _task->showAlert("Needs an SX1262", 2000);
      _task->gotoHomeScreen();
      return;
    }
    if (!_pending_start || !radio_driver.isIdleInRecvMode()) return;   // not mid-TX, mid-RX or with a packet to read
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
    if (err != RADIOLIB_ERR_NONE || !startScanTask()) {
      char tmp[32];
      sprintf(tmp, "Scan failed: %d", err);
      restoreRadio();
      _task->showAlert(tmp, 2000);
      _task->gotoHomeScreen();
      return;
    }
    _header_at = millis();
    _display->setFastRefresh(true);
    _need_wipe = true;
  }

  int render(DisplayDriver& d) override {
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
      _peak_mhz = _freq;
      changeSpan(_span_idx, false);
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

  bool handleInput(char c) override {
    _last_input = millis();
    if (c == KEY_NEXT || c == KEY_RIGHT) {   // zoom out
      changeSpan(_span_idx + 1);
      return true;
    }
    if (c == KEY_PREV || c == KEY_LEFT) {    // zoom in
      changeSpan(_span_idx + 3);
      return true;
    }
    if (c == KEY_ENTER || c == KEY_CANCEL) {
      stop();
      _task->showAlert("Mesh radio resumed", 1000);
      _task->gotoHomeScreen();
      return true;
    }
    return false;
  }
};
#endif

void UITask::begin(DisplayDriver* display, SensorManager* sensors, NodePrefs* node_prefs) {
  _display = display;
  _sensors = sensors;
  _auto_off = millis() + AUTO_OFF_MILLIS;

#if defined(PIN_USER_BTN)
  user_btn.begin();
#endif
#if defined(PIN_USER_BTN_ANA)
  analog_btn.begin();
#endif

  _node_prefs = node_prefs;

  if (_display != NULL) {
    _display->turnOn();
  }

#ifdef PIN_BUZZER
  buzzer.begin();
  buzzer.quiet(_node_prefs->buzzer_quiet);
  buzzer.startup();
#endif

#ifdef PIN_VIBRATION
  vibration.begin();
#endif

  ui_started_at = millis();
  _alert_expiry = 0;

  splash = new SplashScreen(this);
  home = new HomeScreen(this, &rtc_clock, sensors, node_prefs);
  msg_preview = new MsgPreviewScreen(this, &rtc_clock);
#if UI_SPECTRUM_PAGE == 1
  if (_display != NULL) spectrum = new SpectrumScreen(this, node_prefs);   // none found: no page to show it on
#endif
  setCurrScreen(splash);
}

void UITask::showAlert(const char* text, int duration_millis) {
  strcpy(_alert, text);
  _alert_expiry = millis() + duration_millis;
}

void UITask::notify(UIEventType t) {
#if defined(PIN_BUZZER)
switch(t){
  case UIEventType::contactMessage:
    // gemini's pick
    buzzer.play("MsgRcv3:d=4,o=6,b=200:32e,32g,32b,16c7");
    break;
  case UIEventType::channelMessage:
    buzzer.play("kerplop:d=16,o=6,b=120:32g#,32c#");
    break;
  case UIEventType::ack:
    buzzer.play("ack:d=32,o=8,b=120:c");
    break;
  case UIEventType::roomMessage:
  case UIEventType::newContactMessage:
  case UIEventType::none:
  default:
    break;
}
#endif

#ifdef PIN_VIBRATION
  // Trigger vibration for all UI events except none
  if (t != UIEventType::none) {
    vibration.trigger();
  }
#endif
}

void UITask::onMessageRecv(mesh::Packet *pkt, const ContactInfo &from, uint8_t txt_type, uint32_t sender_timestamp, const char* text) {
  // we only want to show text messages on display, not cli data
  if (!(txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_SIGNED_PLAIN)) return;

  uint8_t path_len = pkt->isRouteFlood() ? pkt->path_len : 0xFF;
  ((MsgPreviewScreen *) msg_preview)->addPreview(path_len, from.name, text);
  setCurrScreen(msg_preview);

  if (_display != NULL) {
    if (!_display->isOn() && !hasConnection()) {
      _display->turnOn();
    }
    if (_display->isOn()) {
    _auto_off = millis() + AUTO_OFF_MILLIS;  // extend the auto-off timer
    _next_refresh = 100;  // trigger refresh
    }
  }

  if (!hasConnection()) {
    notify(UIEventType::contactMessage);
  }
}

void UITask::onChannelMessageRecv(mesh::Packet *pkt, ChannelDetails& channel_details, const char* text) {
  uint8_t path_len = pkt->isRouteFlood() ? pkt->path_len : 0xFF;
  ((MsgPreviewScreen *) msg_preview)->addPreview(path_len, channel_details.name, text);
  setCurrScreen(msg_preview);

  if (_display != NULL) {
    if (!_display->isOn() && !hasConnection()) {
      _display->turnOn();
    }
    if (_display->isOn()) {
    _auto_off = millis() + AUTO_OFF_MILLIS;  // extend the auto-off timer
    _next_refresh = 100;  // trigger refresh
    }
  }

  if (!hasConnection()) {
    notify(UIEventType::channelMessage);
  }
}

void UITask::onQueueSizeChanged(int msgcount) {
  _msgcount = msgcount;
  if (msgcount == 0) {
    gotoHomeScreen();
  }
}

void UITask::onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t* path) {
  if (!hasConnection()) {
    notify(UIEventType::newContactMessage);
  }
}

void UITask::onControlDataRecv(const mesh::Packet* packet) {
#if UI_DISCOVER_SCREEN
  if (packet->payload_len >= 14 && (packet->payload[0] & 0xF0) == CTL_TYPE_NODE_DISCOVER_RESP) {
    ((HomeScreen *) home)->handleDiscoverResponse(packet);
  }
#endif
}

void UITask::userLedHandler() {
#ifdef PIN_STATUS_LED
  int cur_time = millis();
  if (cur_time > next_led_change) {
    if (led_state == 0) {
      led_state = 1;
      if (_msgcount > 0) {
        last_led_increment = LED_ON_MSG_MILLIS;
      } else {
        last_led_increment = LED_ON_MILLIS;
      }
      next_led_change = cur_time + last_led_increment;
    } else {
      led_state = 0;
      next_led_change = cur_time + LED_CYCLE_MILLIS - last_led_increment;
    }
    digitalWrite(PIN_STATUS_LED, led_state == LED_STATE_ON);
  }
#endif
}

void UITask::setCurrScreen(UIScreen* c) {
#if UI_SPECTRUM_PAGE == 1
  if (spectrum != NULL && curr == spectrum && c != spectrum) {
    ((SpectrumScreen *) spectrum)->stop();   // always hand the radio back to the mesh
  }
#endif
  curr = c;
  _next_refresh = 100;
}

/*
  hardware-agnostic pre-shutdown activity should be done here
*/
void UITask::shutdown(bool restart){
  releaseRadio();   // powerOff() touches the radio too

  #ifdef PIN_BUZZER
  /* note: we have a choice here -
     we can do a blocking buzzer.loop() with non-deterministic consequences
     or we can set a flag and delay the shutdown for a couple of seconds
     while a non-blocking buzzer.loop() plays out in UITask::loop()
  */
  buzzer.shutdown();
  uint32_t buzzer_timer = millis(); // fail-safe shutdown
  while (buzzer.isPlaying() && (millis() - 2500) < buzzer_timer)
    buzzer.loop();

  #endif // PIN_BUZZER

  if (restart) {
    _board->reboot();
  } else {
    display.forceFullRefresh();
    display.clear();
    display.endFrame();
    // Power off board including radio, display, GPS and components
    _board->powerOff();
  }
}

void UITask::gotoSpectrumScreen() {
#if UI_SPECTRUM_PAGE == 1
  if (_display == NULL || spectrum == NULL) return;
  ((SpectrumScreen *) spectrum)->start(*_display);
  setCurrScreen(spectrum);
#endif
}

void UITask::releaseRadio() {
#if UI_SPECTRUM_PAGE == 1
  if (spectrum != NULL && curr == spectrum) gotoHomeScreen();   // stops the scan and hands the radio back to the mesh
#endif
}

bool UITask::isRadioBorrowed() const {
#if UI_SPECTRUM_PAGE == 1
  return spectrum != NULL && curr == spectrum && ((SpectrumScreen *) spectrum)->ownsRadio();
#else
  return false;
#endif
}

bool UITask::isButtonPressed() const {
#ifdef PIN_USER_BTN
  return user_btn.isPressed();
#else
  return false;
#endif
}

void UITask::loop() {
  char c = 0;

#if UI_HAS_JOYSTICK
  int ev = user_btn.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_ENTER);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_ENTER);  // REVISIT: could be mapped to different key code
  }
  ev = joystick_left.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_LEFT);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_LEFT);
  }
  ev = joystick_right.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_RIGHT);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_RIGHT);
  }
  ev = back_btn.check();
  if (ev == BUTTON_EVENT_TRIPLE_CLICK) {
    c = handleTripleClick(KEY_SELECT);
  }
#elif defined(PIN_USER_BTN)
  int ev = user_btn.check();
  #ifdef UI_HAS_NAV_INPUT
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_ENTER);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    display.turnOff();
  } else if (ev == BUTTON_EVENT_DOUBLE_CLICK) {
    c = handleDoubleClick(KEY_SELECT);
  } else if (ev == BUTTON_EVENT_TRIPLE_CLICK) {
    c = handleTripleClick(KEY_SELECT);
  }
  #else
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_NEXT);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_ENTER);
  } else if (ev == BUTTON_EVENT_DOUBLE_CLICK) {
    c = handleDoubleClick(KEY_PREV);
  } else if (ev == BUTTON_EVENT_TRIPLE_CLICK) {
    c = handleTripleClick(KEY_SELECT);
  }
  #endif  
#endif
#if defined(UI_HAS_ROTARY_INPUT)
  RotaryInputEvent rotaryEv = rotary_input.poll();
  if (c == 0 && _display != NULL && _display->isOn()) {
    if (rotaryEv == RotaryInputEvent::Next) {
      c = KEY_NEXT;
    } else if (rotaryEv == RotaryInputEvent::Prev) {
      c = KEY_PREV;
    }
  }
#endif
#if defined(PIN_USER_BTN_ANA)
  if (abs(millis() - _analogue_pin_read_millis) > 10) {
    int ev = analog_btn.check();
    if (ev == BUTTON_EVENT_CLICK) {
      c = checkDisplayOn(KEY_NEXT);
    } else if (ev == BUTTON_EVENT_LONG_PRESS) {
      c = handleLongPress(KEY_ENTER);
    } else if (ev == BUTTON_EVENT_DOUBLE_CLICK) {
      c = handleDoubleClick(KEY_PREV);
    } else if (ev == BUTTON_EVENT_TRIPLE_CLICK) {
      c = handleTripleClick(KEY_SELECT);
    }
    _analogue_pin_read_millis = millis();
  }
#endif
#if defined(BACKLIGHT_BTN)
  if (millis() > next_backlight_btn_check) {
    bool touch_state = digitalRead(PIN_BUTTON2);
#if defined(DISP_BACKLIGHT)
    digitalWrite(DISP_BACKLIGHT, !touch_state);
#elif defined(EXP_PIN_BACKLIGHT)
    expander.digitalWrite(EXP_PIN_BACKLIGHT, !touch_state);
#endif
    next_backlight_btn_check = millis() + 300;
  }
#endif

  if (c != 0 && curr) {
    curr->handleInput(c);
    _auto_off = millis() + AUTO_OFF_MILLIS;   // extend auto-off timer
    _next_refresh = 100;  // trigger refresh
  }

  userLedHandler();

#ifdef PIN_BUZZER
  if (buzzer.isPlaying())  buzzer.loop();
#endif

  if (curr) curr->poll();

  if (_display != NULL && _display->isOn()) {
    if (millis() >= _next_refresh && curr) {
      _display->startFrame();
      int delay_millis = curr->render(*_display);
      if (millis() < _alert_expiry) {  // render alert popup
        _display->setTextSize(1);
        int y = _display->height() / 3;
        int p = _display->height() / 32;
        _display->setColor(UIColor::popup_bkg);
        _display->fillRect(p, y, _display->width() - p*2, y);
        _display->setColor(UIColor::popup_txt);  // draw box border
        _display->drawRect(p, y, _display->width() - p*2, y);
        _display->drawTextCentered(_display->width() / 2, y + p*3, _alert);
        _next_refresh = _alert_expiry;   // will need refresh when alert is dismissed
      } else {
        _next_refresh = millis() + delay_millis;
      }
      _display->endFrame();
    }
#if AUTO_OFF_MILLIS > 0
#ifdef KEEP_DISPLAY_ON_USB
    // Opt-in: refresh the auto-off deadline while externally powered, so the
    // timer counts from the moment external power is removed. Off by default
    // because OLED panels burn in quickly; only enable for LCD targets or
    // where the display is replaceable.
    if (board.isExternalPowered()) {
      _auto_off = millis() + AUTO_OFF_MILLIS;
    }
#endif
    if (isRadioBorrowed()) {
      _auto_off = millis() + AUTO_OFF_MILLIS;   // a live view: the spectrum screen times out on its own
    }
    if (millis() > _auto_off) {
      _display->turnOff();
    }
#endif
  }

#ifdef PIN_VIBRATION
  vibration.loop();
#endif

#ifdef AUTO_SHUTDOWN_MILLIVOLTS
  if (millis() > next_batt_chck) {
    uint16_t milliVolts = getBattMilliVolts();
    if (milliVolts > 0 && milliVolts < AUTO_SHUTDOWN_MILLIVOLTS) {
      if(!board.isExternalPowered()) {
        releaseRadio();   // stop the spectrum view first: its scan task, and its fast low-contrast refresh
        if (_display != NULL) {
          _display->startFrame();
          _display->setTextSize(2);
          _display->setColor(UIColor::warning_txt);
          _display->drawTextCentered(_display->width() / 2, 20, "Low Battery.");
          _display->drawTextCentered(_display->width() / 2, 40, "Shutting Down!");
          _display->endFrame();
          if (_display->isEink() == false) { delay(3000); }
        }
        shutdown();
      }
    }
    next_batt_chck = millis() + 8000;
  }
#endif
}

char UITask::checkDisplayOn(char c) {
  if (_display != NULL) {
    if (!_display->isOn()) {
      _display->turnOn();   // turn display on and consume event
      c = 0;
    }
    _auto_off = millis() + AUTO_OFF_MILLIS;   // extend auto-off timer
    _next_refresh = 0;  // trigger refresh
  }
  return c;
}

char UITask::handleLongPress(char c) {
  if (millis() - ui_started_at < 8000) {   // long press in first 8 seconds since startup -> CLI/rescue
    the_mesh.enterCLIRescue();
    c = 0;   // consume event
  }
  return c;
}

char UITask::handleDoubleClick(char c) {
  MESH_DEBUG_PRINTLN("UITask: double-click triggered");
  checkDisplayOn(c);
  return c;
}

char UITask::handleTripleClick(char c) {
  MESH_DEBUG_PRINTLN("UITask: triple click triggered");
  checkDisplayOn(c);
  toggleBuzzer();
  c = 0;
  return c;
}

bool UITask::getGPSState() {
  if (_sensors != NULL) {
    int num = _sensors->getNumSettings();
    for (int i = 0; i < num; i++) {
      if (strcmp(_sensors->getSettingName(i), "gps") == 0) {
        return !strcmp(_sensors->getSettingValue(i), "1");
      }
    }
  }
  return false;
}

void UITask::toggleGPS() {
    if (_sensors != NULL) {
    // toggle GPS on/off
    int num = _sensors->getNumSettings();
    for (int i = 0; i < num; i++) {
      if (strcmp(_sensors->getSettingName(i), "gps") == 0) {
        if (strcmp(_sensors->getSettingValue(i), "1") == 0) {
          _sensors->setSettingValue("gps", "0");
          _node_prefs->gps_enabled = 0;
          notify(UIEventType::ack);
        } else {
          _sensors->setSettingValue("gps", "1");
          _node_prefs->gps_enabled = 1;
          notify(UIEventType::ack);
        }
        the_mesh.savePrefs();
        showAlert(_node_prefs->gps_enabled ? "GPS: Enabled" : "GPS: Disabled", 800);
        _next_refresh = 0;
        break;
      }
    }
  }
}

void UITask::toggleBuzzer() {
    // Toggle buzzer quiet mode
  #ifdef PIN_BUZZER
    if (buzzer.isQuiet()) {
      buzzer.quiet(false);
      notify(UIEventType::ack);
    } else {
      buzzer.quiet(true);
    }
    _node_prefs->buzzer_quiet = buzzer.isQuiet();
    the_mesh.savePrefs();
    showAlert(buzzer.isQuiet() ? "Buzzer: OFF" : "Buzzer: ON", 800);
    _next_refresh = 0;  // trigger refresh
  #endif
}
