#include "params.h"

#include <stdlib.h>

#include <algorithm>

namespace {

Param makeNumber(const char *group, const char *key, const char *label, const char *unit, double value, double min,
             double max, double step, const char *help) {
  return Param{key, group, label, unit, value, value, min, max, step, {}, help};
}

Param makeChoice(const char *group, const char *key, const char *label, std::vector<std::string> options, int value,
             const char *help) {
  return Param{key, group, label, "", (double)value, (double)value, 0, (double)options.size() - 1, 1, options, help};
}

}  // namespace

Params::Params() {
  // TANKS
  add(makeNumber("Tanks", "mixer.capacity", "Mixer reservoir capacity", "L", 10, 1, 100, 0.5,
             "Volume at which the mixer reservoir overflows"));
  add(makeNumber("Tanks", "plant.capacity", "Plant reservoir capacity", "L", 50, 5, 500, 1,
             "Volume at which the hydroponics plant reservoir overflows"));
  add(makeNumber("Tanks", "mixer.float", "Mixer float switch height", "%", 90, 5, 100, 1,
             "Level (percent of capacity) at which the mixer float switch triggers. It blocks the input pump."));
  add(makeNumber("Tanks", "plant.float", "Plant float switch height", "%", 90, 5, 100, 1,
             "Level (percent of capacity) at which the plant reservoir float switch triggers. It blocks the output pump."));
  add(makeNumber("Tanks", "mixer.pumpIntake", "Submersible pump intake", "L", 0.4, 0, 5, 0.1,
             "Below this volume the submersible pump in the mixer runs dry"));
  add(makeNumber("Tanks", "plant.hoseIntake", "Self-priming pump hose end", "L", 1.5, 0, 20, 0.5,
             "Below this volume the intake hose in the plant reservoir sucks air"));
  add(makeNumber("Tanks", "temperature", "Water temperature", "°C", 22, 5, 40, 0.5,
             "Temperature of the solution. The firmware assumes 25 °C (no temperature sensor yet)."));
  add(makeNumber("Tanks", "plant.uptake", "Plant water uptake", "L/h", 0, 0, 5, 0.05,
             "Water taken up by the plants and evaporated from the plant reservoir (concentrates the solution)"));

  // PUMPS
  add(makeNumber("Pumps", "output.flow", "Submersible pump flow", "L/min", 3.0, 0.1, 20, 0.1,
             "Output pump (relay 1, GPIO27): mixer reservoir to plant reservoir"));
  add(makeNumber("Pumps", "input.flow", "Self-priming pump flow", "L/min", 3.5, 0.1, 20, 0.1,
             "Input pump (relay 2, GPIO26): plant reservoir to mixer reservoir"));
  add(makeNumber("Pumps", "input.prime", "Self-priming time", "s", 2.5, 0, 30, 0.5,
             "Time the self-priming pump needs to draw water up the intake hose after starting"));
  add(makeNumber("Pumps", "peristaltic.flow", "Peristaltic pump flow", "mL/min", 100, 1, 1000, 1,
             "Flow of each peristaltic dosing pump at 12 V"));
  add(makeChoice("Pumps", "peristaltic.mapping", "Dosing tube wiring",
             {"Firmware (acid=P1, A=P4, B=P3)", "Board labels (A=P1, B=P2, acid=P3)"}, 0,
             "Which bottle each peristaltic pump (L298N channel) is connected to. The firmware and the board "
             "silkscreen disagree (TODO in the firmware), so both can be tried."));
  add(makeChoice("Pumps", "relay.activeLevel", "Relay board trigger", {"Active high", "Active low"}, 0,
             "Input level that energizes the DR21A01 relay boards. The firmware drives them active high "
             "(relayOnLevel)."));

  // SOLUTIONS
  add(makeNumber("Solutions", "acid.strength", "pH Down strength", "mmol/mL", 3.2, 0.01, 15, 0.1,
             "Phosphoric acid in the pH Down concentrate (25 % H3PO4 is about 3.2 mmol/mL)"));
  add(makeNumber("Solutions", "nutrientA.tds", "Nutrient A concentrate", "mg/mL", 250, 1, 1000, 5,
             "Dissolved solids per mL of part A (calcium and potassium nitrates, chelated iron)"));
  add(makeNumber("Solutions", "nutrientB.tds", "Nutrient B concentrate", "mg/mL", 250, 1, 1000, 5,
             "Dissolved solids per mL of part B (phosphates and sulphates)"));
  add(makeNumber("Solutions", "nutrientB.phosphate", "Nutrient B phosphate", "mmol/mL", 0.75, 0, 5, 0.05,
             "Monopotassium phosphate per mL of part B. It buffers the pH around 6-7."));
  add(makeNumber("Solutions", "water.tds", "Top-up water TDS", "ppm", 150, 0, 1000, 5, "Dissolved solids in tap water"));
  add(makeNumber("Solutions", "water.alkalinity", "Top-up water alkalinity", "mmol/L", 1.5, 0, 10, 0.1,
             "Bicarbonate alkalinity of tap water. Higher alkalinity needs more acid to lower the pH."));

  // PROBES
  add(makeChoice("Probes", "ph.location", "pH probe is in",
             {"Mixer reservoir", "Plant reservoir", "pH 4.00 buffer", "pH 7.00 buffer", "pH 10.00 buffer", "Air (dry)"},
             0, "Move the probe into a buffer solution to try the calibration commands (k ph 4.0 / k ph 7.0)"));
  add(makeChoice("Probes", "tds.location", "TDS probe is in",
             {"Mixer reservoir", "Plant reservoir", "707 ppm standard", "Distilled water", "Air (dry)"}, 0,
             "Move the probe into a standard solution to try the calibration commands (k tds 707 / k tds 0)"));
  add(makeNumber("Probes", "ph.offset", "pH board offset error", "mV", 0, -500, 500, 5,
             "Offset of the PH-4502C output from the default calibration (turn of the offset potentiometer)"));
  add(makeNumber("Probes", "ph.slope", "pH electrode slope", "%", 100, 50, 110, 1,
             "Electrode efficiency. Aged electrodes have a lower slope."));
  add(makeNumber("Probes", "tds.gain", "TDS board gain error", "%", 0, -50, 50, 1,
             "Gain error of the TDS meter compared with the default calibration"));
  add(makeNumber("Probes", "noise", "Analog noise", "mV", 4, 0, 100, 1, "Random noise at the sensor outputs"));
  add(makeNumber("Probes", "motorNoise", "Motor interference", "mV", 400, 0, 2000, 10,
             "Amplitude of the spikes that running pumps induce in the sensor readings"));
  add(makeNumber("Probes", "mixing.still", "Probe response (still)", "s", 25, 0.5, 600, 0.5,
             "How fast the solution at the probe follows the reservoir when nothing moves the water"));
  add(makeNumber("Probes", "mixing.flowing", "Probe response (flowing)", "s", 4, 0.1, 120, 0.1,
             "How fast the solution at the probe follows the reservoir while pumps move the water"));

  // FAULTS
  add(makeChoice("Faults", "fault.mixerFloat", "Mixer float switch", {"Normal", "Stuck open", "Stuck closed"}, 0,
             "Stuck open never blocks the input pump; stuck closed always does"));
  add(makeChoice("Faults", "fault.plantFloat", "Plant float switch", {"Normal", "Stuck open", "Stuck closed"}, 0,
             "Stuck open never blocks the output pump; stuck closed always does"));
  add(makeChoice("Faults", "fault.phProbe", "pH probe", {"Connected", "Disconnected"}, 0,
             "A disconnected BNC cable gives a floating, noisy signal"));
  add(makeChoice("Faults", "fault.tdsProbe", "TDS probe", {"Connected", "Disconnected"}, 0,
             "A disconnected probe reads as zero conductivity"));
  add(makeChoice("Faults", "fault.inputPump", "Self-priming pump", {"Normal", "Lost prime", "Jammed"}, 0,
             "Lost prime: it runs but moves no water. Jammed: the motor does not turn."));
  add(makeChoice("Faults", "fault.outputPump", "Submersible pump", {"Normal", "Jammed"}, 0, "Jammed: no flow"));
  add(makeChoice("Faults", "fault.pumpSupply", "12 V pump supply", {"On", "Off"}, 0,
             "Without 12 V the relays click but no pump runs"));
}

void Params::add(const Param &param) { params.push_back(param); }

Param *Params::find(const std::string &key) {
  for (auto &param : params) {
    if (param.key == key) return &param;
  }
  return nullptr;
}

double Params::get(const std::string &key) const {
  for (const auto &param : params) {
    if (param.key == key) return param.value;
  }
  return 0;
}

bool Params::set(const std::string &key, double value) {
  Param *param = find(key);
  if (!param) return false;
  param->value = std::min(param->max, std::max(param->min, value));
  if (!param->options.empty()) param->value = (int)(param->value + 0.5);
  return true;
}

bool Params::setText(const std::string &key, const std::string &value, std::string *error) {
  Param *param = find(key);
  if (!param) {
    if (error) *error = "unknown parameter " + key;
    return false;
  }
  for (size_t i = 0; i < param->options.size(); ++i) {
    if (param->options[i] == value) return set(key, (double)i);
  }
  char *end;
  const double number = strtod(value.c_str(), &end);
  if (value.empty() || *end) {
    if (error) *error = "invalid value '" + value + "' for " + key;
    return false;
  }
  return set(key, number);
}

void Params::resetToDefaults() {
  for (auto &param : params) param.value = param.defaultValue;
}

Json Params::definitions() const {
  Json list = Json::array();
  for (const auto &param : params) {
    Json p = Json::object();
    p.set("key", param.key);
    p.set("group", param.group);
    p.set("label", param.label);
    p.set("unit", param.unit);
    p.set("default", param.defaultValue);
    p.set("min", param.min);
    p.set("max", param.max);
    p.set("step", param.step);
    p.set("help", param.help);
    if (!param.options.empty()) {
      Json options = Json::array();
      for (const auto &option : param.options) options.push(option);
      p.set("options", options);
    }
    list.push(p);
  }
  return list;
}

Json Params::values() const {
  Json values = Json::object();
  for (const auto &param : params) values.set(param.key, param.value);
  return values;
}

void Params::load(const Json &values) {
  for (const auto &member : values.objectMembers()) {
    if (member.second.isNumber()) set(member.first, member.second.asNumber());
  }
}
