// WiFiManager (tzapu) with the configuration portal simulated: instead of a web server on the device, the
// simulator UI has a phone that joins the hotspot and fills in the portal form. The callbacks and the
// non-blocking flow (startConfigPortal() + process()) behave like the real library.
#pragma once

#include <functional>
#include <vector>

#include "WiFi.h"

class WiFiManager {
 public:
  WiFiManager();

  void setDebugOutput(bool debug) { debugOutput = debug; }
  void setConfigPortalBlocking(bool blocking) { portalBlocking = blocking; }
  void setTitle(String title) { this->title = title; }
  void setHostname(const char *hostname) { this->hostname = hostname; }
  void setHostname(String hostname) { this->hostname = hostname; }
  void setMenu(const char *menu[], uint8_t size);
  void setMenu(std::vector<const char *> &menu);
  void setConnectTimeout(unsigned long seconds) { connectTimeoutSeconds = seconds; }
  void setConfigPortalTimeout(unsigned long seconds) { portalTimeoutSeconds = seconds; }
  void setPreSaveConfigCallback(std::function<void()> callback) { preSaveCallback = callback; }
  void setSaveConfigCallback(std::function<void()> callback) { saveCallback = callback; }
  void setAPCallback(std::function<void(WiFiManager *)> callback) { apCallback = callback; }

  bool getWiFiIsSaved();
  String getWiFiSSID(bool persistent = true);
  String getWiFiPass(bool persistent = true);
  bool autoConnect(const char *apName = nullptr, const char *apPassword = nullptr);
  bool startConfigPortal(const char *apName = nullptr, const char *apPassword = nullptr);
  bool getConfigPortalActive() { return portalActive; }
  void stopConfigPortal();
  void resetSettings();
  bool process();
  String getConfigPortalSSID() { return apName; }

  // Simulator side
  String title;
  String hostname;
  std::vector<std::string> menu;
  bool debugOutput = true;
  bool portalBlocking = true;
  bool portalActive = false;
  unsigned long connectTimeoutSeconds = 0;
  unsigned long portalTimeoutSeconds = 0;
  String apName;
  String apPassword;

 private:
  void debug(const String &text);

  std::function<void()> preSaveCallback;
  std::function<void()> saveCallback;
  std::function<void(WiFiManager *)> apCallback;
};
