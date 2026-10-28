// Headless scenario scripts: drive the simulated device with a list of commands and check the outcome.
// See scenarios/README.md for the commands.
#pragma once

#include <stdint.h>

#include <string>
#include <vector>

namespace sim {

class Scenario {
 public:
  bool load(const std::string &path, std::string &error);
  void start();  // Installs the runner on the simulator (runs on the firmware thread)

 private:
  struct Command {
    int line;
    std::vector<std::string> words;
    std::string rest;  // Text after the first word
  };
  void tick();
  bool runCommand(const Command &command);  // Returns false while the command is still waiting
  bool evaluate(const std::vector<std::string> &words, size_t start, std::string &description);
  [[noreturn]] void fail(const Command &command, const std::string &message);

  std::string path;
  std::vector<Command> commands;
  size_t current = 0;
  double commandStartedAt = -1;
  double lastCheckAt = -1;
  size_t serialLinesSeen = 0;
};

bool writePng(const std::string &path, int width, int height, const std::vector<uint32_t> &rgb);

}  // namespace sim
