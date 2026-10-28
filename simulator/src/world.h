// The physical world around the electronics: reservoirs, pumps, dosing bottles and the chemistry of the
// nutrient solution.
//
// A solution is described by extensive quantities, so that mixing two solutions is just adding them:
// - volume
// - dissolved solids (TDS)
// - strong ion difference (strong cations minus strong anions, i.e. alkalinity that is not carbonate/phosphate)
// - total inorganic carbon (CO2 + bicarbonate + carbonate) and total phosphate
// The pH follows from the charge balance of the carbonate and phosphate systems. Acid (phosphoric acid pH Down)
// adds phosphate without cations, nutrient B adds monopotassium phosphate (a buffer around pH 6-7), and CO2
// slowly escapes to the air, so the pH drifts up after acid has converted bicarbonate to CO2.
#pragma once

#include <string>
#include <vector>

#include "json.h"
#include "params.h"

namespace sim {

struct Solution {
  double litres = 0;
  double tdsMg = 0;
  double sidMmol = 0;
  double carbonMmol = 0;
  double phosphateMmol = 0;

  double tdsPpm() const { return litres > 1e-9 ? tdsMg / litres : 0; }
  double ph() const;
  void add(const Solution &other);
  // Removes the given volume (same composition) and returns it
  Solution take(double volume);
  void scale(double factor);

  static Solution water(double litres, double tdsPpm, double alkalinity, double ph);
  Json toJson() const;
  static Solution fromJson(const Json &json);
};

// Dissolved CO2 (mmol/L) in a solution
double dissolvedCo2(const Solution &solution);

enum ProbeLocation {
  IN_MIXER,
  IN_PLANT,
  IN_REFERENCE,  // A calibration solution or air, see ProbeState
};

struct Tank {
  std::string name;
  Solution solution;
  double capacity = 10;
  double spilled = 0;           // Litres that have overflowed
  double inflow = 0;            // L/min, for showing the flows and stirring
  double outflow = 0;
  double ripple = 0;            // Wave height on the surface (fraction of capacity) while water flows in
  bool floatClosed = false;     // Float switch contact (closed = triggered)
  double floatHeight = 0.9;     // Fraction of capacity

  double level() const { return capacity > 0 ? solution.litres / capacity : 0; }
};

struct Bottle {
  std::string name;
  std::string contents;  // "acid", "nutrientA" or "nutrientB"
  double millilitres = 800;
  double capacity = 1000;
};

// Solution at a probe. It follows the reservoir with a delay that is shorter while pumps stir the water.
struct ProbeState {
  int location = 0;       // Index of the location option (see params)
  bool wet = true;        // False when the probe is in the air or in an empty reservoir
  double ph = 7;
  double tdsPpm = 0;
  double temperature = 22;
};

struct PumpState {
  std::string name;
  bool motorOn = false;      // Voltage at the motor
  int direction = 0;         // -1, 0 or 1 (peristaltic pumps can run backwards)
  double flow = 0;           // Actual flow, L/min (negative backwards)
  bool dry = false;          // Running without water
  double primed = 0;         // Self-priming pump: seconds of priming done
  double runSeconds = 0;     // Total motor run time
  double litres = 0;         // Total volume moved
};

// Electrical inputs from the boards
struct Actuators {
  bool outputPumpRelay = false;  // Relay 1 contact closed
  bool inputPumpRelay = false;   // Relay 2 contact closed
  int peristaltic[4] = {0, 0, 0, 0};  // L298N outputs: -1 backwards, 0 stopped, 1 forwards (PPUMP1..4)
};

class World {
 public:
  explicit World(Params &params);
  void reset(int preset);
  void step(double seconds, const Actuators &actuators);

  // Returns the bottle that the peristaltic pump (0..3) is connected to, or -1 for the spare channel
  int bottleOfPump(int pump) const;
  void refillBottle(int bottle);
  void setTankVolume(Tank &tank, double litres);
  void topUpWithWater(Tank &tank, double litres);
  void dose(Tank &tank, int bottle, double millilitres);

  Json toJson() const;
  void fromJson(const Json &json);
  Json snapshot() const;

  Params &params;
  Tank mixer;
  Tank plant;
  std::vector<Bottle> bottles;
  PumpState outputPump;
  PumpState inputPump;
  PumpState peristaltic[4];
  ProbeState phProbe;
  ProbeState tdsProbe;
  double seconds = 0;  // Time since the world was created (continues across device restarts)
  std::vector<std::string> warnings;

 private:
  Solution concentrate(const std::string &contents, double millilitres) const;
  void transfer(Tank &from, Tank &to, double litres);
  void updateFloat(Tank &tank, double floatParamPercent, int fault);
  void updateProbe(ProbeState &probe, bool phProbe, double seconds);
  void warnOnce(const std::string &warning);
  double rippleClock = 0;
};

}  // namespace sim
