#include "wifi_sim.h"

#include "WiFi.h"
#include "WiFiManager.h"
#include "sim_api.h"

namespace sim {

namespace {
// The ESP32 keeps the station configuration in this NVS namespace
const char *wifiNvsSpace = "nvs.net80211";
const double connectSeconds = 2.5;
const double retrySeconds = 5;
}  // namespace

WifiWorld &wifiWorld() {
  static WifiWorld world;
  return world;
}

WifiWorld::WifiWorld() {
  networks = {
    {"HomeNet", "hydroponics", -58, true},
    {"Greenhouse-IoT", "lettuce123", -71, true},
    {"Neighbour 5G", "do-not-know", -84, true},
  };
}

Network *WifiWorld::findNetwork(const std::string &name) {
  for (auto &network : networks) {
    if (network.ssid == name) return &network;
  }
  return nullptr;
}

void WifiWorld::loadSavedCredentials() {
  simhal::NvsType type;
  std::vector<uint8_t> data;
  savedSsid = simhal::nvsGet(wifiNvsSpace, "sta.ssid", type, data) ? std::string(data.begin(), data.end()) : "";
  savedPassword = simhal::nvsGet(wifiNvsSpace, "sta.pswd", type, data) ? std::string(data.begin(), data.end()) : "";
}

void WifiWorld::saveCredentials(const std::string &newSsid, const std::string &newPassword) {
  savedSsid = newSsid;
  savedPassword = newPassword;
  simhal::nvsSet(wifiNvsSpace, "sta.ssid", simhal::NVS_STR, std::vector<uint8_t>(newSsid.begin(), newSsid.end()));
  simhal::nvsSet(wifiNvsSpace, "sta.pswd", simhal::NVS_STR,
                 std::vector<uint8_t>(newPassword.begin(), newPassword.end()));
}

void WifiWorld::clearCredentials() {
  savedSsid.clear();
  savedPassword.clear();
  simhal::nvsRemove(wifiNvsSpace, "sta.ssid");
  simhal::nvsRemove(wifiNvsSpace, "sta.pswd");
}

void WifiWorld::beginConnect(const std::string &newSsid, const std::string &newPassword, double time) {
  ssid = newSsid;
  password = newPassword;
  station = STATION_CONNECTING;
  attemptEndsAt = time + connectSeconds;
  retryAt = -1;
  address.clear();
}

void WifiWorld::update(double time) {
  now = time;
  if (station == STATION_CONNECTING && now >= attemptEndsAt) {
    Network *network = findNetwork(ssid);
    if (!network || !network->on) {
      station = STATION_NO_SSID;
    } else if (network->password != password) {
      station = STATION_FAILED;
    } else {
      station = STATION_CONNECTED;
      address = "192.168.1.57";
    }
    if (station != STATION_CONNECTED && autoReconnect) retryAt = now + retrySeconds;
  }
  if (station == STATION_CONNECTED) {
    Network *network = findNetwork(ssid);
    if (!network || !network->on || network->password != password) {
      station = STATION_NO_SSID;
      address.clear();
      if (autoReconnect) retryAt = now + retrySeconds;
    }
  }
  if (retryAt >= 0 && now >= retryAt && station != STATION_CONNECTED && station != STATION_CONNECTING) {
    beginConnect(ssid, password, now);
  }
}

int WifiWorld::status() const {
  switch (station) {
    case STATION_CONNECTED:
      return WL_CONNECTED;
    case STATION_NO_SSID:
      return WL_NO_SSID_AVAIL;
    case STATION_FAILED:
      return WL_CONNECT_FAILED;
    case STATION_CONNECTING:
      return WL_DISCONNECTED;
    case STATION_IDLE:
      return WL_IDLE_STATUS;
    default:
      return WL_DISCONNECTED;
  }
}

void WifiWorld::phoneJoin(const std::string &joinSsid, const std::string &joinPassword) {
  if (!apActive || joinSsid != apSsid) {
    phoneMessage = "Network " + joinSsid + " not found";
    phoneJoined = false;
  } else if (joinPassword != apPassword) {
    phoneMessage = "Incorrect password for " + joinSsid;
    phoneJoined = false;
  } else {
    phoneJoined = true;
    phoneMessage = "Joined " + joinSsid + ". The setup page opened (captive portal).";
  }
}

void WifiWorld::phoneSubmit(const std::string &newSsid, const std::string &newPassword) {
  if (!phoneJoined || !apActive) {
    phoneMessage = "Not connected to the setup hotspot";
    return;
  }
  submissionPending = true;
  submittedSsid = newSsid;
  submittedPassword = newPassword;
  phoneMessage = "Saving... the device is connecting to " + newSsid;
}

Json WifiWorld::snapshot() const {
  static const char *stationNames[] = {"off", "idle", "connecting", "connected", "network not found", "wrong password"};
  Json json = Json::object();
  json.set("mode", mode);
  json.set("station", stationNames[station]);
  json.set("ssid", ssid);
  json.set("savedSsid", savedSsid);
  json.set("address", address);
  json.set("apActive", apActive);
  json.set("apSsid", apSsid);
  json.set("phoneJoined", phoneJoined);
  json.set("phoneMessage", phoneMessage);
  Json list = Json::array();
  for (const auto &network : networks) {
    list.push(Json::object().set("ssid", network.ssid).set("password", network.password).set("rssi", network.rssi).set("on", network.on));
  }
  json.set("networks", list);
  return json;
}

}  // namespace sim

using sim::wifiWorld;

// ARDUINO WIFI API

WiFiClass WiFi;

static double nowSeconds() { return simhal::micros64() / 1e6; }

String IPAddress::toString() const {
  char text[16];
  snprintf(text, sizeof(text), "%u.%u.%u.%u", bytes[0], bytes[1], bytes[2], bytes[3]);
  return String(text);
}

bool WiFiClass::mode(wifi_mode_t mode) {
  simhal::spend(50000);  // Starting the radio takes a while
  auto &world = wifiWorld();
  world.mode = mode;
  if ((mode & WIFI_STA) && world.station == sim::WifiWorld::STATION_OFF) world.station = sim::WifiWorld::STATION_IDLE;
  if (!(mode & WIFI_STA)) world.station = sim::WifiWorld::STATION_OFF;
  return true;
}

wifi_mode_t WiFiClass::getMode() { return (wifi_mode_t)wifiWorld().mode; }

uint8_t *WiFiClass::macAddress(uint8_t *mac) {
  static const uint8_t address[6] = {0x24, 0x6F, 0x28, 0xA1, 0x7C, 0x3E};
  memcpy(mac, address, 6);
  return mac;
}

String WiFiClass::macAddress() {
  uint8_t mac[6];
  macAddress(mac);
  char text[18];
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(text);
}

bool WiFiClass::setAutoReconnect(bool autoReconnect) {
  wifiWorld().autoReconnect = autoReconnect;
  return true;
}

bool WiFiClass::getAutoReconnect() { return wifiWorld().autoReconnect; }

wl_status_t WiFiClass::begin() {
  auto &world = wifiWorld();
  world.loadSavedCredentials();
  if (world.savedSsid.empty()) return WL_CONNECT_FAILED;
  if (!(world.mode & WIFI_STA)) mode(WIFI_STA);
  world.beginConnect(world.savedSsid, world.savedPassword, nowSeconds());
  return (wl_status_t)world.status();
}

wl_status_t WiFiClass::begin(const char *ssid, const char *passphrase) {
  auto &world = wifiWorld();
  if (!(world.mode & WIFI_STA)) mode((wifi_mode_t)(world.mode | WIFI_STA));
  // WiFi.persistent(true) is the default: the credentials are written to NVS
  world.saveCredentials(ssid ? ssid : "", passphrase ? passphrase : "");
  world.beginConnect(world.savedSsid, world.savedPassword, nowSeconds());
  return (wl_status_t)world.status();
}

bool WiFiClass::disconnect(bool wifiOff, bool eraseAp) {
  auto &world = wifiWorld();
  world.station = sim::WifiWorld::STATION_IDLE;
  world.retryAt = -1;
  world.address.clear();
  if (eraseAp) world.clearCredentials();
  if (wifiOff) mode(WIFI_OFF);
  return true;
}

bool WiFiClass::reconnect() {
  auto &world = wifiWorld();
  world.beginConnect(world.ssid, world.password, nowSeconds());
  return true;
}

wl_status_t WiFiClass::status() {
  simhal::spend(5);
  auto &world = wifiWorld();
  world.update(nowSeconds());
  return (wl_status_t)world.status();
}

String WiFiClass::SSID() {
  auto &world = wifiWorld();
  return world.station == sim::WifiWorld::STATION_CONNECTED ? String(world.ssid) : String();
}

int8_t WiFiClass::RSSI() {
  auto &world = wifiWorld();
  sim::Network *network = world.findNetwork(world.ssid);
  return world.station == sim::WifiWorld::STATION_CONNECTED && network ? network->rssi : 0;
}

IPAddress WiFiClass::localIP() {
  auto &world = wifiWorld();
  return world.station == sim::WifiWorld::STATION_CONNECTED ? IPAddress(192, 168, 1, 57) : IPAddress();
}

IPAddress WiFiClass::softAPIP() { return wifiWorld().apActive ? IPAddress(192, 168, 4, 1) : IPAddress(); }

bool WiFiClass::setHostname(const char *) { return true; }

bool WiFiClass::softAP(const char *ssid, const char *passphrase) {
  auto &world = wifiWorld();
  mode((wifi_mode_t)(world.mode | WIFI_AP));
  world.apActive = true;
  world.apSsid = ssid ? ssid : "";
  world.apPassword = passphrase ? passphrase : "";
  return true;
}

bool WiFiClass::softAPdisconnect(bool) {
  auto &world = wifiWorld();
  world.apActive = false;
  world.phoneJoined = false;
  if (world.mode & WIFI_AP) world.mode &= ~WIFI_AP;
  return true;
}

// WIFIMANAGER

WiFiManager::WiFiManager() {}

void WiFiManager::debug(const String &text) {
  if (!debugOutput) return;
  Serial.print("*wm:");
  Serial.println(text);
}

void WiFiManager::setMenu(const char *items[], uint8_t size) {
  menu.clear();
  for (uint8_t i = 0; i < size; ++i) menu.push_back(items[i]);
}

void WiFiManager::setMenu(std::vector<const char *> &items) {
  menu.clear();
  for (const char *item : items) menu.push_back(item);
}

bool WiFiManager::getWiFiIsSaved() {
  wifiWorld().loadSavedCredentials();
  return !wifiWorld().savedSsid.empty();
}

String WiFiManager::getWiFiSSID(bool) {
  wifiWorld().loadSavedCredentials();
  return String(wifiWorld().savedSsid);
}

String WiFiManager::getWiFiPass(bool) {
  wifiWorld().loadSavedCredentials();
  return String(wifiWorld().savedPassword);
}

bool WiFiManager::autoConnect(const char *name, const char *password) {
  if (getWiFiIsSaved()) {
    WiFi.begin();
    const unsigned long startedAt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startedAt < (connectTimeoutSeconds ? connectTimeoutSeconds : 10) * 1000UL) {
      delay(100);
    }
    if (WiFi.status() == WL_CONNECTED) return true;
  }
  startConfigPortal(name, password);
  return false;
}

bool WiFiManager::startConfigPortal(const char *name, const char *password) {
  apName = name ? name : "ESP32";
  apPassword = password ? password : "";
  debug("Starting Config Portal");
  WiFi.softAP(apName.c_str(), apPassword.c_str());
  debug(String("StartAP with SSID:  ") + apName);
  debug(String("AP IP address: ") + WiFi.softAPIP().toString());
  debug("Starting Web Portal");
  portalActive = true;
  if (apCallback) apCallback(this);
  if (!portalBlocking) {
    debug("Config Portal Running, non blocking (processing)");
    return false;
  }
  // Blocking portal: process until configured
  while (portalActive) {
    if (process()) return true;
    delay(10);
  }
  return false;
}

void WiFiManager::stopConfigPortal() {
  if (!portalActive) return;
  debug("Stopping Config Portal");
  portalActive = false;
  WiFi.softAPdisconnect(false);
}

void WiFiManager::resetSettings() {
  debug("resetSettings");
  WiFi.disconnect(false, true);
  debug("SETTINGS ERASED");
}

bool WiFiManager::process() {
  simhal::spend(20);
  auto &world = wifiWorld();
  if (!portalActive || !world.submissionPending) return false;
  world.submissionPending = false;

  debug("processConfigPortal");
  if (preSaveCallback) preSaveCallback();
  debug(String("Connecting to NEW AP: ") + world.submittedSsid.c_str());
  WiFi.begin(world.submittedSsid.c_str(), world.submittedPassword.c_str());
  // Blocks until connected or the connect timeout (like waitForConnectResult())
  const unsigned long timeout = (connectTimeoutSeconds ? connectTimeoutSeconds : 10) * 1000UL;
  const unsigned long startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startedAt < timeout) {
    delay(100);
  }
  if (WiFi.status() == WL_CONNECTED) {
    debug("Connect to new AP [SUCCESS]");
    debug(String("Got IP Address: ") + WiFi.localIP().toString());
    world.phoneMessage = "Credentials saved. The device connected to " + world.submittedSsid + ".";
    if (saveCallback) saveCallback();
    stopConfigPortal();
    return true;
  }
  debug("[ERROR] Connect to new AP Failed");
  world.phoneMessage = "Could not connect to " + world.submittedSsid + ". The setup page is still open.";
  world.retryAt = -1;
  world.station = sim::WifiWorld::STATION_IDLE;
  return false;
}
