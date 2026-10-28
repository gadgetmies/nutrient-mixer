// Simulator core: runs the firmware on a virtual clock and connects it to the simulated board and world.
//
// The firmware runs on its own thread. Virtual time only advances when the firmware spends time (every Arduino
// API call costs a few microseconds, delay() its argument, display transfers their SPI time). Everything else -
// the physics, input events from the UI, interrupts - happens on the firmware thread at defined virtual times,
// so a run is the same at any simulation speed. The speed only decides how virtual time is paced against real
// time (or not at all, as fast as possible).
#pragma once

#include <stdint.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "board.h"
#include "json.h"
#include "params.h"
#include "world.h"

namespace sim {

struct NvsEntry {
  uint8_t type;
  std::vector<uint8_t> data;
};

struct Published {
  uint64_t sequence = 0;
  std::string state;  // JSON
  uint64_t panelSequence = 0;
  std::vector<uint16_t> panel;
  bool backlight = false;
};

class Simulator {
 public:
  Simulator();

  // Startup
  std::string stateDirectory = "state";
  bool resetState = false;
  bool echoSerial = true;
  bool headless = false;
  std::vector<std::string> arguments;  // For restarting the process (device reset)
  void load();
  [[noreturn]] void runFirmware();

  // Virtual time (firmware thread)
  uint64_t micros() const { return clock; }
  double seconds() const { return clock / 1e6; }
  void spend(uint64_t microseconds);
  void schedule(uint64_t at, std::function<void()> action);
  void scheduleIn(double seconds, std::function<void()> action) { schedule(clock + (uint64_t)(seconds * 1e6), action); }

  // Called from any thread: runs the action on the firmware thread
  void post(std::function<void()> action);

  // Pacing (any thread)
  void setSpeed(double speed);  // 0 = as fast as possible
  void setPaused(bool paused);
  void stepFor(double seconds);
  void requestQuit(int exitCode);
  [[noreturn]] void quitNow(int exitCode);
  void requestRestart();

  // Inputs (firmware thread)
  void rotateEncoder(int detents);
  void pressButton(Button button);
  void releaseButton(Button button);
  void clickButton(Button button, double holdSeconds = 0.12);
  void serialInput(const std::string &text);

  // Serial port (firmware thread)
  void serialWrite(const uint8_t *data, size_t length);
  int serialAvailable() const { return (int)serialRx.size(); }
  int serialRead();
  int serialPeek() const { return serialRx.empty() ? -1 : serialRx.front(); }

  // NVS (firmware thread)
  bool nvsGet(const std::string &space, const std::string &key, NvsEntry &entry) const;
  void nvsSet(const std::string &space, const std::string &key, const NvsEntry &entry);
  bool nvsRemove(const std::string &space, const std::string &key);
  void nvsClear(const std::string &space);
  void eraseFlash();

  // Display
  uint16_t panel[135 * 240];
  bool panelDirty = true;
  bool backlight = false;

  // State for the UI
  Json snapshot();
  Json hello();  // Sent to new UI connections: parameter definitions
  std::string helloText();  // hello() serialized; any thread
  std::function<void()> onPublish;  // Called on the firmware thread about 25 times per (real) second
  std::function<void()> onMillisecond;  // Called on the firmware thread every virtual millisecond
  std::mutex publishMutex;
  std::condition_variable publishCondition;
  Published published;
  void publish();
  // Serial output from the given absolute offset (updated to the end); any thread
  std::string serialSince(uint64_t &offset);
  std::vector<std::string> serialLines;  // Complete lines received since start (for scenario checks)

  Params params;
  World world;
  Board board;

  // World state (tanks, bottles, parameters) is saved so that it survives restarts like the real world
  void saveWorld();
  void resetWorld(int preset);

 private:
  void service();
  void pace();
  void drainInbox();
  void stepPhysics();
  void loadNvs();
  void saveNvs();
  [[noreturn]] void restartProcess();

  uint64_t clock = 0;
  uint64_t nextService = 0;
  uint64_t nextPhysics = 0;
  uint64_t nextPace = 0;
  bool inService = false;

  struct Event {
    uint64_t at;
    uint64_t sequence;
    std::function<void()> action;
    bool operator<(const Event &other) const {
      return at != other.at ? at > other.at : sequence > other.sequence;
    }
  };
  std::priority_queue<Event> events;
  uint64_t eventSequence = 0;

  std::mutex inboxMutex;
  std::condition_variable inboxCondition;
  std::vector<std::function<void()>> inbox;
  std::atomic<bool> inboxPending{false};

  // Pacing
  std::atomic<double> speed{1};
  std::atomic<bool> paused{false};
  std::atomic<double> stepRequest{0};
  uint64_t stepUntil = 0;
  bool stepping = false;
  int64_t anchorReal = 0;  // Nanoseconds (steady clock)
  uint64_t anchorVirtual = 0;
  double anchoredSpeed = -1;
  int64_t lastPublishReal = 0;
  int64_t speedSampleReal = 0;
  uint64_t speedSampleVirtual = 0;
  double actualSpeed = 0;
  int64_t lastSaveReal = 0;
  std::atomic<int> quitCode{-1};
  std::atomic<bool> restartRequested{false};

  // Inputs
  int encoderPhase = 0;
  uint64_t encoderBusyUntil = 0;
  uint64_t buttonPressedAt[BUTTON_COUNT] = {0, 0, 0, 0};

  // Serial
  std::deque<uint8_t> serialRx;
  std::string currentLine;
  std::mutex historyMutex;
  std::string history;
  uint64_t historyBase = 0;  // Absolute offset of history[0]
  double uartBusyUntil = 0;  // Virtual microseconds when the UART has sent everything written so far

  std::mutex helloMutex;
  std::string helloCache;

  // NVS
  std::map<std::string, std::map<std::string, NvsEntry>> nvs;
  bool nvsDirty = false;
};

Simulator &simulator();

// Implemented next to the firmware (firmware_unit.cpp): the firmware's own state for the UI
Json firmwareSnapshot();

}  // namespace sim
