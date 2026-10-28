#include "sim.h"

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <random>
#include <sstream>
#include <thread>

#include "sim_api.h"
#include "wifi_sim.h"

void setup();
void loop();

namespace sim {

namespace {

int64_t realNanoseconds() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

const uint64_t physicsStepMicroseconds = 10000;
const uint64_t loopMicroseconds = 100;  // Time of one loop() iteration besides the API calls it makes
const int64_t publishIntervalNanoseconds = 40000000;
const size_t serialHistoryLimit = 256 * 1024;

bool readFile(const std::string &path, std::string &contents) {
  std::ifstream file(path, std::ios::binary);
  if (!file) return false;
  std::stringstream buffer;
  buffer << file.rdbuf();
  contents = buffer.str();
  return true;
}

bool writeFileAtomically(const std::string &path, const std::string &contents) {
  const std::string temporary = path + ".tmp";
  {
    std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
    if (!file) return false;
    file << contents;
    if (!file) return false;
  }
  return rename(temporary.c_str(), path.c_str()) == 0;
}

std::string toHex(const std::vector<uint8_t> &data) {
  static const char digits[] = "0123456789abcdef";
  std::string hex;
  for (uint8_t byte : data) {
    hex += digits[byte >> 4];
    hex += digits[byte & 15];
  }
  return hex;
}

std::vector<uint8_t> fromHex(const std::string &hex) {
  std::vector<uint8_t> data;
  for (size_t i = 0; i + 1 < hex.size(); i += 2) data.push_back((uint8_t)strtoul(hex.substr(i, 2).c_str(), nullptr, 16));
  return data;
}

}  // namespace

Simulator &simulator() {
  static Simulator instance;
  return instance;
}

Simulator::Simulator() : world(params), board(params, world) {
  // Display memory holds random data at power up
  std::mt19937 random(7);
  for (auto &pixel : panel) pixel = random() & 0xFFFF;
}

void Simulator::load() {
  mkdir(stateDirectory.c_str(), 0755);
  if (resetState) {
    remove((stateDirectory + "/nvs.json").c_str());
    remove((stateDirectory + "/world.json").c_str());
  }
  loadNvs();
  std::string text;
  Json saved;
  if (readFile(stateDirectory + "/world.json", text) && Json::parse(text, saved)) {
    params.load(saved["params"]);
    world.fromJson(saved["world"]);
  } else {
    world.reset(0);
  }
}

void Simulator::runFirmware() {
  board.powerOn();
  setup();
  for (;;) {
    loop();
    spend(loopMicroseconds);
  }
}

// TIME

void Simulator::spend(uint64_t microseconds) {
  const uint64_t target = clock + microseconds;
  if (inService) {
    // Interrupt handlers and events run inside service(); they only advance the clock
    clock = target;
    board.now = clock * 1e-6;
    return;
  }
  while (nextService <= target) {
    if (nextService > clock) clock = nextService;
    service();
  }
  clock = target;
  board.now = clock * 1e-6;
}

void Simulator::schedule(uint64_t at, std::function<void()> action) {
  events.push(Event{at < clock ? clock : at, eventSequence++, action});
  if (events.top().at < nextService) nextService = events.top().at;
}

void Simulator::service() {
  inService = true;
  board.now = clock * 1e-6;
  while (!events.empty() && events.top().at <= clock) {
    Event event = events.top();
    events.pop();
    event.action();
  }
  while (clock >= nextPhysics) {
    stepPhysics();
    nextPhysics += physicsStepMicroseconds;
  }
  if (clock >= nextPace) {
    while (nextPace <= clock) nextPace += 1000;
    if (onMillisecond) onMillisecond();
    pace();
  }
  inService = false;
  nextService = std::min(nextPhysics, nextPace);
  if (!events.empty()) nextService = std::min(nextService, events.top().at);
}

void Simulator::stepPhysics() {
  world.step(physicsStepMicroseconds / 1e6, board.actuators());
  board.update(clock * 1e-6);
  wifiWorld().update(clock * 1e-6);
}

void Simulator::post(std::function<void()> action) {
  {
    std::lock_guard<std::mutex> lock(inboxMutex);
    inbox.push_back(std::move(action));
    inboxPending = true;
  }
  inboxCondition.notify_all();
}

void Simulator::drainInbox() {
  std::vector<std::function<void()>> actions;
  {
    std::lock_guard<std::mutex> lock(inboxMutex);
    actions.swap(inbox);
    inboxPending = false;
  }
  for (auto &action : actions) action();
}

void Simulator::setSpeed(double newSpeed) { speed = newSpeed < 0 ? 0 : newSpeed; }

void Simulator::setPaused(bool pause) {
  paused = pause;
  inboxCondition.notify_all();
}

void Simulator::stepFor(double stepSeconds) {
  stepRequest = stepSeconds;
  inboxCondition.notify_all();
}

void Simulator::requestQuit(int exitCode) {
  quitCode = exitCode;
  inboxCondition.notify_all();
}

void Simulator::requestRestart() {
  restartRequested = true;
  inboxCondition.notify_all();
}

void Simulator::pace() {
  if (quitCode >= 0) quitNow(quitCode);
  if (restartRequested) restartProcess();
  if (inboxPending) drainInbox();

  const double step = stepRequest.exchange(0);
  if (step > 0) {
    stepUntil = clock + (uint64_t)(step * 1e6);
    stepping = true;
  }
  if (stepping && clock >= stepUntil) {
    stepping = false;
    publish();
  }

  int64_t now = realNanoseconds();
  while (paused && !stepping) {
    if (!headless && now - lastPublishReal >= publishIntervalNanoseconds) {
      lastPublishReal = now;
      publish();
    }
    {
      std::unique_lock<std::mutex> lock(inboxMutex);
      inboxCondition.wait_for(lock, std::chrono::milliseconds(30), [&] {
        return inboxPending || !paused || stepRequest > 0 || quitCode >= 0 || restartRequested;
      });
    }
    if (quitCode >= 0) quitNow(quitCode);
    if (restartRequested) restartProcess();
    if (inboxPending) drainInbox();
    const double request = stepRequest.exchange(0);
    if (request > 0) {
      stepUntil = clock + (uint64_t)(request * 1e6);
      stepping = true;
    }
    now = realNanoseconds();
    anchoredSpeed = -1;  // Re-anchor the pacing after the pause
  }

  const double currentSpeed = stepping ? 0 : speed.load();
  if (currentSpeed != anchoredSpeed) {
    anchorReal = now;
    anchorVirtual = clock;
    anchoredSpeed = currentSpeed;
  }
  if (currentSpeed > 0) {
    const int64_t target = anchorReal + (int64_t)((clock - anchorVirtual) * 1000.0 / currentSpeed);
    if (target > now + 1000000) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(target - now));
      now = realNanoseconds();
    } else if (now - target > 200000000) {
      // The host cannot keep up; continue from here instead of trying to catch up
      anchorReal = now;
      anchorVirtual = clock;
    }
  }

  if (now - speedSampleReal >= 500000000) {
    if (speedSampleReal > 0) {
      actualSpeed = (clock - speedSampleVirtual) * 1000.0 / (double)(now - speedSampleReal);
    }
    speedSampleReal = now;
    speedSampleVirtual = clock;
  }
  if (!headless && now - lastPublishReal >= publishIntervalNanoseconds) {
    lastPublishReal = now;
    publish();
  }
  if (now - lastSaveReal >= 3000000000LL) {
    lastSaveReal = now;
    if (nvsDirty) saveNvs();
    if (!headless) saveWorld();
  }
}

void Simulator::quitNow(int exitCode) {
  if (nvsDirty) saveNvs();
  if (!headless) saveWorld();
  fflush(stdout);
  _exit(exitCode);
}

void Simulator::restartProcess() {
  if (nvsDirty) saveNvs();
  saveWorld();
  fflush(stdout);
  std::vector<char *> argv;
  for (auto &argument : arguments) argv.push_back(const_cast<char *>(argument.c_str()));
  argv.push_back(nullptr);
  execv(argv[0], argv.data());
  perror("Could not restart the simulator");
  _exit(1);
}

// INPUTS

void Simulator::rotateEncoder(int detents) {
  // Contacts (A closed, B closed) through one detent of the EC12E. A closes first when turning clockwise.
  static const bool phases[4][2] = {{false, false}, {true, false}, {true, true}, {false, true}};
  const int direction = detents > 0 ? 1 : -1;
  uint64_t at = std::max(clock, encoderBusyUntil);
  for (int i = 0; i < abs(detents); ++i) {
    for (int k = 1; k <= 4; ++k) {
      encoderPhase = (encoderPhase + direction) & 3;
      const int phase = encoderPhase;
      schedule(at + k * 2500, [this, phase] { board.setEncoderContacts(phases[phase][0], phases[phase][1]); });
    }
    at += 4 * 2500 + 30000;
  }
  encoderBusyUntil = at;
}

void Simulator::pressButton(Button button) {
  board.setButton(button, true);
  buttonPressedAt[button] = clock;
}

void Simulator::releaseButton(Button button) {
  // A press lasts at least 80 ms, however briefly it was clicked
  const uint64_t at = std::max(clock, buttonPressedAt[button] + 80000);
  schedule(at, [this, button] { board.setButton(button, false); });
}

void Simulator::clickButton(Button button, double holdSeconds) {
  pressButton(button);
  scheduleIn(holdSeconds, [this, button] { board.setButton(button, false); });
}

void Simulator::serialInput(const std::string &text) {
  for (char c : text) serialRx.push_back((uint8_t)c);
  if (text.empty() || text.back() != '\n') serialRx.push_back('\n');
}

// SERIAL

void Simulator::serialWrite(const uint8_t *data, size_t length) {
  // 115200 baud with a 128 byte transmit FIFO: writing blocks while the FIFO is full
  const double characterMicroseconds = 1e6 * 10 / 115200;
  if (uartBusyUntil < clock) uartBusyUntil = clock;
  uartBusyUntil += length * characterMicroseconds;
  const double backlog = uartBusyUntil - clock - 128 * characterMicroseconds;

  const std::string text((const char *)data, length);
  {
    std::lock_guard<std::mutex> lock(historyMutex);
    history += text;
    if (history.size() > serialHistoryLimit) {
      const size_t removed = history.size() - serialHistoryLimit * 3 / 4;
      history.erase(0, removed);
      historyBase += removed;
    }
  }
  for (char c : text) {
    if (c == '\n') {
      if (!currentLine.empty() && currentLine.back() == '\r') currentLine.pop_back();
      serialLines.push_back(currentLine);
      if (serialLines.size() > 5000) serialLines.erase(serialLines.begin(), serialLines.begin() + 1000);
      currentLine.clear();
    } else {
      currentLine += c;
    }
  }
  if (echoSerial) {
    fwrite(data, 1, length, stdout);
    fflush(stdout);
  }
  if (backlog > 0) spend((uint64_t)backlog);
}

std::string Simulator::serialSince(uint64_t &offset) {
  std::lock_guard<std::mutex> lock(historyMutex);
  if (offset < historyBase) offset = historyBase;
  const uint64_t end = historyBase + history.size();
  std::string text = offset < end ? history.substr(offset - historyBase) : std::string();
  offset = end;
  return text;
}

int Simulator::serialRead() {
  if (serialRx.empty()) return -1;
  const int c = serialRx.front();
  serialRx.pop_front();
  return c;
}

// NVS

bool Simulator::nvsGet(const std::string &space, const std::string &key, NvsEntry &entry) const {
  auto s = nvs.find(space);
  if (s == nvs.end()) return false;
  auto k = s->second.find(key);
  if (k == s->second.end()) return false;
  entry = k->second;
  return true;
}

void Simulator::nvsSet(const std::string &space, const std::string &key, const NvsEntry &entry) {
  nvs[space][key] = entry;
  nvsDirty = true;
}

bool Simulator::nvsRemove(const std::string &space, const std::string &key) {
  auto s = nvs.find(space);
  if (s == nvs.end()) return false;
  nvsDirty = true;
  return s->second.erase(key) > 0;
}

void Simulator::nvsClear(const std::string &space) {
  nvs.erase(space);
  nvsDirty = true;
}

void Simulator::eraseFlash() {
  nvs.clear();
  saveNvs();
}

void Simulator::loadNvs() {
  std::string text;
  Json json;
  if (!readFile(stateDirectory + "/nvs.json", text) || !Json::parse(text, json)) return;
  for (const auto &space : json.objectMembers()) {
    for (const auto &entry : space.second.objectMembers()) {
      nvs[space.first][entry.first] = NvsEntry{(uint8_t)entry.second["type"].asNumber(), fromHex(entry.second["hex"].asString())};
    }
  }
}

void Simulator::saveNvs() {
  Json json = Json::object();
  for (const auto &space : nvs) {
    Json entries = Json::object();
    for (const auto &entry : space.second) {
      entries.set(entry.first, Json::object().set("type", entry.second.type).set("hex", toHex(entry.second.data)));
    }
    json.set(space.first, entries);
  }
  writeFileAtomically(stateDirectory + "/nvs.json", json.dump());
  nvsDirty = false;
}

void Simulator::saveWorld() {
  Json json = Json::object();
  json.set("params", params.values());
  json.set("world", world.toJson());
  writeFileAtomically(stateDirectory + "/world.json", json.dump());
}

void Simulator::resetWorld(int preset) { world.reset(preset); }

// STATE FOR THE UI

Json Simulator::snapshot() {
  Json json = Json::object();
  json.set("type", "state");
  json.set("time", seconds());
  json.set("speed", speed.load());
  json.set("actualSpeed", actualSpeed);
  json.set("paused", paused.load() && !stepping);
  json.set("firmware", firmwareSnapshot());
  json.set("board", board.snapshot());
  json.set("world", world.snapshot());
  json.set("wifi", wifiWorld().snapshot());
  json.set("params", params.values());
  return json;
}

Json Simulator::hello() {
  Json json = Json::object();
  json.set("type", "hello");
  json.set("params", params.definitions());
  return json;
}

std::string Simulator::helloText() {
  // The parameter definitions do not change while running
  std::lock_guard<std::mutex> lock(helloMutex);
  if (helloCache.empty()) helloCache = hello().dump();
  return helloCache;
}

void Simulator::publish() {
  std::string state = snapshot().dump();
  {
    std::lock_guard<std::mutex> lock(publishMutex);
    published.state = std::move(state);
    published.sequence++;
    if (panelDirty) {
      published.panel.assign(panel, panel + 135 * 240);
      published.panelSequence++;
      panelDirty = false;
    }
    published.backlight = backlight;
  }
  publishCondition.notify_all();
  if (onPublish) onPublish();
}

}  // namespace sim

// SIMHAL (the Arduino API side)

using sim::simulator;

namespace simhal {

uint64_t micros64() { return simulator().micros(); }
void spend(uint64_t microseconds) { simulator().spend(microseconds); }
void pinMode(uint8_t pin, uint8_t mode) { simulator().board.pinMode(pin, mode); }
void digitalWrite(uint8_t pin, uint8_t level) { simulator().board.digitalWrite(pin, level); }
int digitalRead(uint8_t pin) { return simulator().board.digitalRead(pin); }
uint32_t analogReadMilliVolts(uint8_t pin) { return simulator().board.analogMilliVolts(pin); }
int analogReadRaw(uint8_t pin) { return simulator().board.analogRaw(pin); }
void attachInterrupt(uint8_t pin, void (*handler)(), int mode) { simulator().board.attachInterrupt(pin, handler, mode); }
void detachInterrupt(uint8_t pin) { simulator().board.detachInterrupt(pin); }

uint32_t random32() {
  static std::mt19937 random(2024);
  return random();
}

void serialWrite(const uint8_t *data, size_t length) { simulator().serialWrite(data, length); }
int serialAvailable() { return simulator().serialAvailable(); }
int serialRead() { return simulator().serialRead(); }
int serialPeek() { return simulator().serialPeek(); }

bool nvsGet(const std::string &space, const std::string &key, NvsType &type, std::vector<uint8_t> &data) {
  sim::NvsEntry entry;
  if (!simulator().nvsGet(space, key, entry)) return false;
  type = (NvsType)entry.type;
  data = entry.data;
  return true;
}

void nvsSet(const std::string &space, const std::string &key, NvsType type, const std::vector<uint8_t> &data) {
  simulator().nvsSet(space, key, sim::NvsEntry{(uint8_t)type, data});
}

bool nvsRemove(const std::string &space, const std::string &key) { return simulator().nvsRemove(space, key); }
void nvsClear(const std::string &space) { simulator().nvsClear(space); }

uint16_t *panelPixels() { return simulator().panel; }
void panelChanged() { simulator().panelDirty = true; }
void panelBacklight(bool on) { simulator().backlight = on; }

}  // namespace simhal
