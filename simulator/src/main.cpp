// Nutrient mixer simulator: runs the ESP32 firmware (../nutrient_mixer.ino) against a simulated board and
// world, and serves a UI that shows the T-Display, the electronics and the flow of water and nutrients.
#include <libgen.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <thread>

#include "actions.h"
#include "scenario.h"
#include "server.h"
#include "sim.h"

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

namespace {

void printUsage(const char *program) {
  printf(
      "Usage: %s [options]\n"
      "\n"
      "Runs the nutrient mixer firmware in a simulated device and serves the UI at http://localhost:<port>.\n"
      "Type serial commands into the terminal (or the UI console); '?' lists them.\n"
      "\n"
      "Options:\n"
      "  --port <port>       UI port (default 8737)\n"
      "  --open              Open the UI in the browser\n"
      "  --speed <x>         Simulation speed: 1 = real time, 0 = as fast as possible (default 1)\n"
      "  --paused            Start paused\n"
      "  --state <dir>       Directory for the device flash (NVS) and the world state (default: state next to\n"
      "                      the simulator directory)\n"
      "  --reset             Start with erased flash and a fresh world\n"
      "  --quiet             Do not echo the serial output to the terminal\n"
      "  --headless          Run without the UI (use with --duration or --script)\n"
      "  --duration <s>      Headless: stop after this many seconds of device time\n"
      "  --script <file>     Headless: run a scenario script (see scenarios/README.md); exit code 1 if it fails\n"
      "  --web <dir>         Directory of the UI files\n"
      "  --eagle <dir>       Directory of the EAGLE board files\n",
      program);
}

std::string executablePath(const char *argv0) {
  char path[PATH_MAX];
#ifdef __APPLE__
  uint32_t size = sizeof(path);
  if (_NSGetExecutablePath(path, &size) == 0) {
    char resolved[PATH_MAX];
    if (realpath(path, resolved)) return resolved;
  }
#else
  const ssize_t n = readlink("/proc/self/exe", path, sizeof(path) - 1);
  if (n > 0) {
    path[n] = 0;
    return path;
  }
#endif
  char resolved[PATH_MAX];
  return realpath(argv0, resolved) ? resolved : argv0;
}

std::string directoryOf(const std::string &path) {
  std::string copy = path;
  return dirname(&copy[0]);
}

void openBrowser(const std::string &url) {
#ifdef __APPLE__
  const std::string command = "open '" + url + "' >/dev/null 2>&1 &";
#else
  const std::string command = "xdg-open '" + url + "' >/dev/null 2>&1 &";
#endif
  if (system(command.c_str()) != 0) printf("Open %s in your browser\n", url.c_str());
}

void onSignal(int) { sim::simulator().requestQuit(0); }

}  // namespace

int main(int argc, char **argv) {
  sim::Simulator &s = sim::simulator();
  const std::string executable = executablePath(argv[0]);
  // build/nutrient-mixer-sim -> simulator/
  const std::string simulatorDirectory = directoryOf(directoryOf(executable));

  int port = 8737;
  bool open = false;
  bool restarted = false;
  double speed = 1;
  bool paused = false;
  double duration = -1;
  std::string script;
  std::string webDirectory = simulatorDirectory + "/web";
  std::string eagleDirectory = simulatorDirectory + "/../eagle";
  s.stateDirectory = simulatorDirectory + "/state";
  bool stateGiven = false;

  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) {
        fprintf(stderr, "Missing value for %s\n", option.c_str());
        exit(2);
      }
      return argv[++i];
    };
    if (option == "--port") {
      port = atoi(value().c_str());
    } else if (option == "--open") {
      open = true;
    } else if (option == "--restarted") {
      restarted = true;
    } else if (option == "--speed") {
      speed = atof(value().c_str());
    } else if (option == "--paused") {
      paused = true;
    } else if (option == "--state") {
      s.stateDirectory = value();
      stateGiven = true;
    } else if (option == "--reset") {
      s.resetState = true;
    } else if (option == "--quiet") {
      s.echoSerial = false;
    } else if (option == "--headless") {
      s.headless = true;
    } else if (option == "--duration") {
      duration = atof(value().c_str());
    } else if (option == "--script") {
      script = value();
      s.headless = true;
    } else if (option == "--web") {
      webDirectory = value();
    } else if (option == "--eagle") {
      eagleDirectory = value();
    } else if (option == "--help" || option == "-h") {
      printUsage(argv[0]);
      return 0;
    } else {
      fprintf(stderr, "Unknown option %s\n\n", option.c_str());
      printUsage(argv[0]);
      return 2;
    }
  }

  // Restarting the device restarts the process with the same options (but without reopening the browser and
  // without erasing the flash again)
  s.arguments.push_back(executable);
  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    if (option != "--reset" && option != "--paused") s.arguments.push_back(option);
  }
  if (!restarted) s.arguments.push_back("--restarted");

  if (s.headless) {
    // Scenarios start from a fresh device and world unless told otherwise, and never touch the UI's state
    if (!stateGiven) {
      char directory[] = "/tmp/nutrient-mixer-sim-XXXXXX";
      if (!mkdtemp(directory)) {
        perror("mkdtemp");
        return 1;
      }
      s.stateDirectory = directory;
    }
    s.load();
    s.setSpeed(0);
    static sim::Scenario scenario;
    if (!script.empty()) {
      std::string error;
      if (!scenario.load(script, error)) {
        fprintf(stderr, "%s\n", error.c_str());
        return 2;
      }
      scenario.start();
    } else if (duration > 0) {
      s.onMillisecond = [&s, duration] {
        if (s.seconds() >= duration) s.quitNow(0);
      };
    } else {
      fprintf(stderr, "--headless needs --duration or --script\n");
      return 2;
    }
    s.runFirmware();
  }

  s.load();
  s.setSpeed(speed);
  s.setPaused(paused);

  static sim::Server server;
  server.onMessage = sim::handleUiMessage;
  if (!server.start(port, {{"/eagle/", eagleDirectory}, {"/", webDirectory}})) {
    fprintf(stderr, "Could not listen on port %d (is the simulator already running? Use --port)\n", port);
    return 1;
  }
  std::thread([] { server.run(); }).detach();

  const std::string url = "http://localhost:" + std::to_string(port) + "/";
  if (!restarted) {
    printf("Nutrient mixer simulator: %s\nType serial commands here or in the UI ('?' lists them). Ctrl-C quits.\n\n",
           url.c_str());
    if (open) openBrowser(url);
  } else {
    printf("\n--- device restarted ---\n\n");
  }
  fflush(stdout);

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  // Lines typed into the terminal go to the device's serial port
  std::thread([&s] {
    std::string line;
    while (std::getline(std::cin, line)) {
      s.post([&s, line] { s.serialInput(line); });
    }
  }).detach();

  s.runFirmware();
}
