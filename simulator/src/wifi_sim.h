// Simulated radio environment: the networks around the device, the ESP32 station and soft AP, and a phone
// that can join the setup hotspot and fill in the WiFiManager portal page.
#pragma once

#include <string>
#include <vector>

#include "json.h"

namespace sim {

struct Network {
  std::string ssid;
  std::string password;
  int rssi;
  bool on;
};

class WifiWorld {
 public:
  WifiWorld();
  void update(double now);

  // Station
  enum StationState { STATION_OFF, STATION_IDLE, STATION_CONNECTING, STATION_CONNECTED, STATION_NO_SSID, STATION_FAILED };
  void loadSavedCredentials();
  void saveCredentials(const std::string &ssid, const std::string &password);
  void clearCredentials();
  void beginConnect(const std::string &ssid, const std::string &password, double now);
  int status() const;  // wl_status_t

  int mode = 0;  // wifi_mode_t
  bool autoReconnect = true;
  StationState station = STATION_OFF;
  std::string savedSsid;
  std::string savedPassword;
  std::string ssid;
  std::string password;
  double attemptEndsAt = 0;
  double retryAt = -1;
  std::string address;

  // Soft AP (WiFiManager configuration portal)
  bool apActive = false;
  std::string apSsid;
  std::string apPassword;

  // Phone
  bool phoneJoined = false;
  std::string phoneMessage;
  bool submissionPending = false;
  std::string submittedSsid;
  std::string submittedPassword;
  void phoneJoin(const std::string &ssid, const std::string &password);
  void phoneSubmit(const std::string &ssid, const std::string &password);

  std::vector<Network> networks;
  Network *findNetwork(const std::string &ssid);

  Json snapshot() const;
  double now = 0;
};

WifiWorld &wifiWorld();

}  // namespace sim
