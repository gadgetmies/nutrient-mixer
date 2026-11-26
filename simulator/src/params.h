// Tunable parameters of the simulated world (tank sizes, pump flows, probe errors, faults...). The UI builds its
// parameter panel from these definitions, scenarios set them by key, and they are saved with the world state.
#pragma once

#include <string>
#include <vector>

#include "json.h"

struct Param {
  std::string key;
  std::string group;
  std::string label;
  std::string unit;
  double value;
  double defaultValue;
  double min;
  double max;
  double step;
  std::vector<std::string> options;  // For choices: value is the index of the selected option
  std::string help;
};

class Params {
 public:
  Params();
  double get(const std::string &key) const;
  int choice(const std::string &key) const { return (int)get(key); }
  bool set(const std::string &key, double value);
  // Accepts a number or, for choices, an option name
  bool setText(const std::string &key, const std::string &value, std::string *error = nullptr);
  void resetToDefaults();
  Param *find(const std::string &key);
  const std::vector<Param> &all() const { return params; }

  Json definitions() const;
  Json values() const;
  void load(const Json &values);

 private:
  void add(const Param &param);
  std::vector<Param> params;
};
