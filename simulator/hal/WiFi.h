// arduino-esp32 WiFi (station and soft AP). The radio and the networks around the device are simulated in
// ../src/wifi_sim.cpp.
#pragma once

#include "Arduino.h"

typedef enum {
  WL_NO_SHIELD = 255,
  WL_IDLE_STATUS = 0,
  WL_NO_SSID_AVAIL = 1,
  WL_SCAN_COMPLETED = 2,
  WL_CONNECTED = 3,
  WL_CONNECT_FAILED = 4,
  WL_CONNECTION_LOST = 5,
  WL_DISCONNECTED = 6
} wl_status_t;

typedef enum {
  WIFI_OFF = 0,
  WIFI_STA = 1,
  WIFI_AP = 2,
  WIFI_AP_STA = 3
} wifi_mode_t;

class IPAddress : public Printable {
 public:
  IPAddress() : bytes{0, 0, 0, 0} {}
  IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) : bytes{a, b, c, d} {}
  String toString() const;
  uint8_t operator[](int index) const { return bytes[index & 3]; }
  bool operator==(const IPAddress &other) const { return memcmp(bytes, other.bytes, 4) == 0; }
  bool operator!=(const IPAddress &other) const { return !(*this == other); }
  size_t printTo(Print &p) const override { return p.print(toString()); }

 private:
  uint8_t bytes[4];
};

class WiFiClass {
 public:
  bool mode(wifi_mode_t mode);
  wifi_mode_t getMode();
  uint8_t *macAddress(uint8_t *mac);
  String macAddress();
  bool setAutoReconnect(bool autoReconnect);
  bool getAutoReconnect();
  wl_status_t begin();
  wl_status_t begin(const char *ssid, const char *passphrase = nullptr);
  bool disconnect(bool wifiOff = false, bool eraseAp = false);
  bool reconnect();
  wl_status_t status();
  bool isConnected() { return status() == WL_CONNECTED; }
  String SSID();
  int8_t RSSI();
  IPAddress localIP();
  IPAddress softAPIP();
  bool setHostname(const char *hostname);
  bool softAP(const char *ssid, const char *passphrase = nullptr);
  bool softAPdisconnect(bool wifiOff = false);
};

extern WiFiClass WiFi;
