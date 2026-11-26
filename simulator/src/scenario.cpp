#include "scenario.h"

#include <stdio.h>
#include <stdlib.h>

#include <fstream>
#include <sstream>

#include "actions.h"
#include "sim.h"

namespace sim {

namespace {

std::vector<std::string> splitWords(const std::string &line) {
  std::vector<std::string> words;
  std::istringstream stream(line);
  std::string word;
  while (stream >> word) words.push_back(word);
  return words;
}

std::string restAfterFirstWord(const std::string &line) {
  const size_t start = line.find_first_not_of(" \t");
  const size_t space = line.find_first_of(" \t", start);
  if (space == std::string::npos) return "";
  const size_t rest = line.find_first_not_of(" \t", space);
  return rest == std::string::npos ? "" : line.substr(rest);
}

bool toNumber(const std::string &text, double &value) {
  char *end;
  value = strtod(text.c_str(), &end);
  return !text.empty() && *end == 0;
}

// Display contents as RGB, upright as mounted (the firmware uses rotation 3)
std::vector<uint32_t> screenImage() {
  std::vector<uint32_t> image(240 * 135);
  const uint16_t *panel = simulator().panel;
  const bool backlight = simulator().backlight;
  for (int y = 0; y < 135; ++y) {
    for (int x = 0; x < 240; ++x) {
      const uint16_t c = panel[(239 - x) * 135 + y];
      uint32_t r = (c >> 11) << 3, g = ((c >> 5) & 63) << 2, b = (c & 31) << 3;
      if (!backlight) r = g = b = 0;
      image[y * 240 + x] = (r << 16) | (g << 8) | b;
    }
  }
  return image;
}

}  // namespace

bool Scenario::load(const std::string &file, std::string &error) {
  path = file;
  std::ifstream input(file);
  if (!input) {
    error = "Cannot open " + file;
    return false;
  }
  std::string line;
  int number = 0;
  while (std::getline(input, line)) {
    ++number;
    std::vector<std::string> words = splitWords(line);
    // A '#' starts a comment, except in text sent over serial
    if (words.empty() || words[0] != "serial") {
      line = line.substr(0, line.find('#'));
      words = splitWords(line);
    }
    if (words.empty()) continue;
    commands.push_back(Command{number, words, restAfterFirstWord(line)});
  }
  return true;
}

void Scenario::start() {
  simulator().onMillisecond = [this] { tick(); };
}

void Scenario::fail(const Command &command, const std::string &message) {
  fprintf(stdout, "\n[scenario] %s:%d: FAILED: %s\n", path.c_str(), command.line, message.c_str());
  simulator().quitNow(1);
}

void Scenario::tick() {
  Simulator &s = simulator();
  while (current < commands.size()) {
    const Command &command = commands[current];
    if (commandStartedAt < 0) commandStartedAt = s.seconds();
    if (!runCommand(command)) return;
    ++current;
    commandStartedAt = -1;
    lastCheckAt = -1;
  }
  fprintf(stdout, "\n[scenario] %s: passed (%.1f s of device time)\n", path.c_str(), s.seconds());
  s.quitNow(0);
}

// <path> <operator> <value>: compares a value of the simulator state (see the UI state message)
bool Scenario::evaluate(const std::vector<std::string> &words, size_t start, std::string &description) {
  if (words.size() < start + 3) {
    description = "expected: <path> <operator> <value>";
    return false;
  }
  const Json state = simulator().snapshot();
  const Json &value = state.path(words[start]);
  const std::string &op = words[start + 1];
  std::string expected = words[start + 2];
  for (size_t i = start + 3; i < words.size() && words[i] != "timeout"; ++i) expected += " " + words[i];
  description = words[start] + " = " + value.toText() + ", expected " + op + " " + expected;

  // Numbers compare as numbers, also when the firmware shows them as text (e.g. firmware.ph)
  double expectedNumber, number = value.asNumber();
  const bool numeric = value.isNumber() || toNumber(value.asString(), number);
  if (numeric && toNumber(expected, expectedNumber)) {
    if (op == "==") return number == expectedNumber;
    if (op == "!=") return number != expectedNumber;
    if (op == "<") return number < expectedNumber;
    if (op == "<=") return number <= expectedNumber;
    if (op == ">") return number > expectedNumber;
    if (op == ">=") return number >= expectedNumber;
  }
  const std::string text = value.toText();
  if (op == "==") return text == expected;
  if (op == "!=") return text != expected;
  if (op == "contains") return text.find(expected) != std::string::npos;
  return false;
}

bool Scenario::runCommand(const Command &command) {
  Simulator &s = simulator();
  const std::vector<std::string> &w = command.words;
  const std::string &name = w[0];
  const double elapsed = s.seconds() - commandStartedAt;

  auto argument = [&](size_t index) -> std::string {
    if (index >= w.size()) fail(command, "missing argument for " + name);
    return w[index];
  };
  auto number = [&](size_t index) -> double {
    double value;
    if (!toNumber(argument(index), value)) fail(command, "not a number: " + w[index]);
    return value;
  };
  auto timeoutOf = [&]() -> double {
    for (size_t i = 0; i + 1 < w.size(); ++i) {
      if (w[i] == "timeout") return number(i + 1);
    }
    return 3600;
  };
  auto act = [&](const Json &message) {
    const std::string error = performAction(message);
    if (!error.empty()) fail(command, error);
  };

  if (name == "wait") return elapsed >= number(1);
  if (name == "serial") {
    s.serialInput(command.rest);
    return true;
  }
  if (name == "until" || name == "expect") {
    // Checking every 50 ms of device time is enough and keeps the runs fast
    if (name == "until" && lastCheckAt >= 0 && s.seconds() - lastCheckAt < 0.05) return false;
    lastCheckAt = s.seconds();
    std::string description;
    if (evaluate(w, 1, description)) {
      if (name == "expect") fprintf(stdout, "\n[scenario] ok: %s\n", description.c_str());
      return true;
    }
    if (name == "expect") fail(command, description);
    if (elapsed > timeoutOf()) fail(command, "timed out: " + description);
    return false;
  }
  if (name == "until-serial" || name == "expect-serial") {
    std::string text;
    for (size_t i = 1; i < w.size() && w[i] != "timeout"; ++i) text += (i > 1 ? " " : "") + w[i];
    for (size_t i = serialLinesSeen; i < s.serialLines.size(); ++i) {
      if (s.serialLines[i].find(text) != std::string::npos) {
        serialLinesSeen = i + 1;
        return true;
      }
    }
    if (name == "expect-serial") fail(command, "no serial output containing '" + text + "'");
    if (elapsed > timeoutOf()) fail(command, "timed out waiting for serial output '" + text + "'");
    return false;
  }
  if (name == "click" || name == "press" || name == "release") {
    Json message = Json::object().set("type", "button").set("button", argument(1)).set("action", name);
    if (name == "click" && w.size() > 2) message.set("hold", number(2));
    act(message);
    return true;
  }
  if (name == "rotate") {
    act(Json::object().set("type", "encoder").set("detents", number(1)));
    return true;
  }
  if (name == "set") {
    act(Json::object().set("type", "param").set("key", argument(1)).set("value", argument(2)));
    return true;
  }
  if (name == "world") {
    const std::string action = argument(1);
    Json message = Json::object().set("type", "world").set("action", action);
    if (action == "reset") message.set("preset", w.size() > 2 ? number(2) : 0);
    if (action == "refill") message.set("bottle", number(2));
    if (action == "volume") message.set("tank", argument(2)).set("litres", number(3));
    act(message);
    return true;
  }
  if (name == "wifi") {
    const std::string action = argument(1);
    Json message = Json::object().set("type", "wifi");
    if (action == "network") {
      message.set("action", "network").set("ssid", argument(2)).set("on", argument(3) == "on");
    } else if (action == "join") {
      message.set("action", "phoneJoin").set("ssid", argument(2)).set("password", argument(3));
    } else if (action == "scan") {
      message.set("action", "phoneScan");
    } else if (action == "submit") {
      message.set("action", "phoneSubmit").set("ssid", argument(2)).set("password", w.size() > 3 ? w[3] : "");
    } else {
      fail(command, "unknown wifi command " + action);
    }
    act(message);
    return true;
  }
  if (name == "print") {
    const Json state = s.snapshot();
    fprintf(stdout, "\n[scenario] t=%.1f s", s.seconds());
    for (size_t i = 1; i < w.size(); ++i) fprintf(stdout, " %s=%s", w[i].c_str(), state.path(w[i]).toText().c_str());
    fprintf(stdout, "\n");
    return true;
  }
  if (name == "screenshot") {
    if (!writePng(argument(1), 240, 135, screenImage())) fail(command, "could not write " + w[1]);
    fprintf(stdout, "\n[scenario] screenshot saved to %s\n", w[1].c_str());
    return true;
  }
  fail(command, "unknown command " + name);
}

// PNG with uncompressed (stored) deflate blocks

static uint32_t crc32(const uint8_t *data, size_t length, uint32_t crc = 0) {
  static uint32_t table[256];
  static bool ready = false;
  if (!ready) {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = c & 1 ? 0xEDB88320 ^ (c >> 1) : c >> 1;
      table[i] = c;
    }
    ready = true;
  }
  crc = ~crc;
  for (size_t i = 0; i < length; ++i) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

bool writePng(const std::string &file, int width, int height, const std::vector<uint32_t> &rgb) {
  std::string raw;
  for (int y = 0; y < height; ++y) {
    raw += (char)0;  // No filter
    for (int x = 0; x < width; ++x) {
      const uint32_t c = rgb[y * width + x];
      raw += (char)(c >> 16);
      raw += (char)(c >> 8);
      raw += (char)c;
    }
  }
  std::string zlib = "\x78\x01";
  for (size_t i = 0; i < raw.size(); i += 65535) {
    const size_t n = std::min<size_t>(65535, raw.size() - i);
    zlib += (char)(i + n >= raw.size() ? 1 : 0);
    zlib += (char)(n & 0xFF);
    zlib += (char)(n >> 8);
    zlib += (char)(~n & 0xFF);
    zlib += (char)((~n >> 8) & 0xFF);
    zlib += raw.substr(i, n);
  }
  uint32_t a = 1, b = 0;
  for (unsigned char c : raw) {
    a = (a + c) % 65521;
    b = (b + a) % 65521;
  }
  const uint32_t adler = (b << 16) | a;
  for (int i = 3; i >= 0; --i) zlib += (char)((adler >> (i * 8)) & 0xFF);

  auto be32 = [](uint32_t v) {
    std::string s;
    for (int i = 3; i >= 0; --i) s += (char)((v >> (i * 8)) & 0xFF);
    return s;
  };
  auto chunk = [&](const std::string &type, const std::string &data) {
    const std::string body = type + data;
    return be32(data.size()) + body + be32(crc32((const uint8_t *)body.data(), body.size()));
  };
  std::string header = be32(width) + be32(height) + std::string("\x08\x02\x00\x00\x00", 5);
  std::string png = std::string("\x89PNG\r\n\x1a\n", 8) + chunk("IHDR", header) + chunk("IDAT", zlib) + chunk("IEND", "");
  std::ofstream out(file, std::ios::binary);
  out << png;
  return (bool)out;
}

}  // namespace sim
