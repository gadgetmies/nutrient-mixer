#include "world.h"

#include <math.h>

#include <algorithm>

namespace sim {

namespace {

// Equilibrium constants at 25 degrees C
const double Kw = 1e-14;
const double carbonicK1 = 4.47e-7;   // pKa 6.35
const double carbonicK2 = 4.69e-11;  // pKa 10.33
const double phosphoricK1 = 7.08e-3;  // pKa 2.15
const double phosphoricK2 = 6.31e-8;  // pKa 7.20
const double phosphoricK3 = 4.47e-13;  // pKa 12.35
// Dissolved CO2 in equilibrium with air (420 ppm)
const double co2Equilibrium = 0.0136;  // mmol/L

// Height of the probe tips and the volume below which a reservoir counts as empty for them
const double mixerProbeDepth = 0.5;  // L
const double plantProbeDepth = 2.0;  // L

double clampValue(double value, double low, double high) { return std::max(low, std::min(high, value)); }

// Average negative charge per carbonate and phosphate at the given hydrogen ion concentration (mol/L)
double carbonateCharge(double h) {
  return (carbonicK1 * h + 2 * carbonicK1 * carbonicK2) / (h * h + carbonicK1 * h + carbonicK1 * carbonicK2);
}

double phosphateCharge(double h) {
  const double a1 = phosphoricK1, a12 = phosphoricK1 * phosphoricK2, a123 = a12 * phosphoricK3;
  return (a1 * h * h + 2 * a12 * h + 3 * a123) / (h * h * h + a1 * h * h + a12 * h + a123);
}

}  // namespace

// SOLUTION

double Solution::ph() const {
  if (litres < 1e-6) return NAN;
  const double sid = sidMmol / litres / 1000;
  const double carbon = carbonMmol / litres / 1000;
  const double phosphate = phosphateMmol / litres / 1000;
  // The charge balance decreases monotonically with the pH; bisect for its root
  double low = 0, high = 14;
  for (int i = 0; i < 48; ++i) {
    const double middle = (low + high) / 2;
    const double h = pow(10, -middle);
    const double charge = sid + h - Kw / h - carbon * carbonateCharge(h) - phosphate * phosphateCharge(h);
    if (charge > 0) {
      low = middle;
    } else {
      high = middle;
    }
  }
  return (low + high) / 2;
}

double dissolvedCo2(const Solution &solution) {
  if (solution.litres < 1e-6) return 0;
  const double h = pow(10, -solution.ph());
  const double fraction = h * h / (h * h + carbonicK1 * h + carbonicK1 * carbonicK2);
  return solution.carbonMmol / solution.litres * fraction;
}

void Solution::add(const Solution &other) {
  litres += other.litres;
  tdsMg += other.tdsMg;
  sidMmol += other.sidMmol;
  carbonMmol += other.carbonMmol;
  phosphateMmol += other.phosphateMmol;
}

Solution Solution::take(double volume) {
  Solution part;
  if (litres <= 1e-12 || volume <= 0) return part;
  const double fraction = std::min(1.0, volume / litres);
  part = *this;
  part.scale(fraction);
  scale(1 - fraction);
  return part;
}

void Solution::scale(double factor) {
  litres *= factor;
  tdsMg *= factor;
  sidMmol *= factor;
  carbonMmol *= factor;
  phosphateMmol *= factor;
}

Solution Solution::water(double litres, double tdsPpm, double alkalinity, double ph) {
  Solution water;
  water.litres = litres;
  water.tdsMg = tdsPpm * litres;
  const double h = pow(10, -ph);
  // Alkalinity (mmol/L) = carbonate charge + OH - H; strong ions balance it
  const double carbon = (alkalinity / 1000 - Kw / h + h) / carbonateCharge(h);
  water.carbonMmol = std::max(0.0, carbon * 1000 * litres);
  water.sidMmol = alkalinity * litres;
  return water;
}

Json Solution::toJson() const {
  Json json = Json::object();
  json.set("litres", litres);
  json.set("tdsMg", tdsMg);
  json.set("sidMmol", sidMmol);
  json.set("carbonMmol", carbonMmol);
  json.set("phosphateMmol", phosphateMmol);
  return json;
}

Solution Solution::fromJson(const Json &json) {
  Solution solution;
  solution.litres = json["litres"].asNumber();
  solution.tdsMg = json["tdsMg"].asNumber();
  solution.sidMmol = json["sidMmol"].asNumber();
  solution.carbonMmol = json["carbonMmol"].asNumber();
  solution.phosphateMmol = json["phosphateMmol"].asNumber();
  return solution;
}

// WORLD

World::World(Params &params) : params(params) {
  mixer.name = "Mixer reservoir";
  plant.name = "Plant reservoir";
  bottles = {
    {"pH Down", "acid", 800, 1000},
    {"Nutrient A", "nutrientA", 800, 1000},
    {"Nutrient B", "nutrientB", 800, 1000},
  };
  outputPump.name = "Submersible pump";
  inputPump.name = "Self-priming pump";
  for (int i = 0; i < 4; ++i) peristaltic[i].name = "PPUMP" + std::to_string(i + 1);
  reset(0);
}

Solution World::concentrate(const std::string &contents, double millilitres) const {
  Solution solution;
  solution.litres = millilitres / 1000;
  if (contents == "acid") {
    // Phosphoric acid: phosphate without cations. 25 % acid has about 300 mg of solids per mL.
    solution.phosphateMmol = params.get("acid.strength") * millilitres;
    solution.tdsMg = 98 * solution.phosphateMmol;
  } else if (contents == "nutrientA") {
    // Neutral salts (nitrates): no effect on the pH balance
    solution.tdsMg = params.get("nutrientA.tds") * millilitres;
  } else if (contents == "nutrientB") {
    // Monopotassium phosphate: potassium (strong cation) and dihydrogen phosphate
    const double phosphate = params.get("nutrientB.phosphate") * millilitres;
    solution.tdsMg = params.get("nutrientB.tds") * millilitres;
    solution.phosphateMmol = phosphate;
    solution.sidMmol = phosphate;
  }
  return solution;
}

void World::reset(int preset) {
  const double tds = params.get("water.tds");
  const double alkalinity = params.get("water.alkalinity");
  mixer.capacity = params.get("mixer.capacity");
  plant.capacity = params.get("plant.capacity");
  mixer.spilled = plant.spilled = 0;

  auto fill = [&](Tank &tank, double litres, double nutrientMlPerLitre) {
    tank.solution = Solution::water(litres, tds, alkalinity, 7.6);
    tank.solution.add(concentrate("nutrientA", nutrientMlPerLitre * litres));
    tank.solution.add(concentrate("nutrientB", nutrientMlPerLitre * litres));
  };
  switch (preset) {
    case 1:  // Fresh tap water
      fill(mixer, 6, 0);
      fill(plant, 30, 0);
      break;
    case 2:  // Mixer reservoir empty
      fill(mixer, 0.2, 0.9);
      fill(plant, 40, 0.9);
      break;
    default:  // Used nutrient solution (about 600 ppm, pH 7)
      fill(mixer, 6, 0.9);
      fill(plant, 30, 0.9);
      break;
  }
  for (auto &bottle : bottles) bottle.millilitres = 800;
  for (PumpState *pump : {&outputPump, &inputPump, &peristaltic[0], &peristaltic[1], &peristaltic[2], &peristaltic[3]}) {
    pump->runSeconds = pump->litres = pump->primed = 0;
  }
  // The probes start equilibrated with their solution
  phProbe.location = -1;
  tdsProbe.location = -1;
  updateProbe(phProbe, true, 1e6);
  updateProbe(tdsProbe, false, 1e6);
}

int World::bottleOfPump(int pump) const {
  // Bottles: 0 acid, 1 nutrient A, 2 nutrient B
  if (params.choice("peristaltic.mapping") == 1) {
    // Board silkscreen: left L298N (PPUMP1/2) "Nutrients" A and B, right L298N (PPUMP3) "Acid"
    static const int boardLabels[4] = {1, 2, 0, -1};
    return boardLabels[pump];
  }
  // Firmware: acidPump = PPUMP1, nutrientBPump = PPUMP3, nutrientAPump = PPUMP4
  static const int firmware[4] = {0, -1, 2, 1};
  return firmware[pump];
}

void World::refillBottle(int bottle) {
  if (bottle >= 0 && bottle < (int)bottles.size()) bottles[bottle].millilitres = bottles[bottle].capacity;
}

void World::setTankVolume(Tank &tank, double litres) {
  litres = clampValue(litres, 0, tank.capacity);
  if (litres < tank.solution.litres) {
    tank.solution.take(tank.solution.litres - litres);
  } else {
    topUpWithWater(tank, litres - tank.solution.litres);
  }
}

void World::topUpWithWater(Tank &tank, double litres) {
  if (litres <= 0) return;
  tank.solution.add(Solution::water(litres, params.get("water.tds"), params.get("water.alkalinity"), 7.6));
}

void World::dose(Tank &tank, int bottle, double millilitres) {
  if (bottle < 0 || bottle >= (int)bottles.size()) return;
  millilitres = std::min(millilitres, bottles[bottle].millilitres);
  bottles[bottle].millilitres -= millilitres;
  tank.solution.add(concentrate(bottles[bottle].contents, millilitres));
}

void World::transfer(Tank &from, Tank &to, double litres) {
  to.solution.add(from.solution.take(litres));
}

void World::warnOnce(const std::string &warning) {
  if (std::find(warnings.begin(), warnings.end(), warning) == warnings.end()) warnings.push_back(warning);
}

void World::updateFloat(Tank &tank, double floatParamPercent, int fault) {
  tank.floatHeight = floatParamPercent / 100;
  if (fault == 1) {
    tank.floatClosed = false;
    return;
  }
  if (fault == 2) {
    tank.floatClosed = true;
    return;
  }
  // Waves from the inflow make the float bob, so the contact chatters when the level is close to it
  const double level = tank.level() + tank.ripple;
  const double hysteresis = 0.004;
  if (level >= tank.floatHeight) {
    tank.floatClosed = true;
  } else if (level < tank.floatHeight - hysteresis) {
    tank.floatClosed = false;
  }
}

void World::updateProbe(ProbeState &probe, bool isPhProbe, double dt) {
  const int location = params.choice(isPhProbe ? "ph.location" : "tds.location");
  const bool moved = location != probe.location;
  probe.location = location;
  probe.temperature = params.get("temperature");

  // Target solution at the probe and how fast the probe follows it
  double targetPh = 7, targetTds = 0;
  bool wet = true;
  double timeConstant = params.get("mixing.still");
  auto inTank = [&](const Tank &tank, double depth) {
    wet = tank.solution.litres >= depth;
    targetPh = tank.solution.ph();
    targetTds = tank.solution.tdsPpm();
    if (tank.inflow > 0 || tank.outflow > 0) timeConstant = params.get("mixing.flowing");
  };
  if (location == 0) {
    inTank(mixer, mixerProbeDepth);
  } else if (location == 1) {
    inTank(plant, plantProbeDepth);
  } else if (isPhProbe) {
    static const double buffers[3] = {4.0, 7.0, 10.0};
    if (location <= 4) {
      targetPh = buffers[location - 2];
      targetTds = 0;
    } else {
      wet = false;
    }
  } else {
    if (location == 2) {
      targetTds = 707;
    } else if (location == 3) {
      targetTds = 0;
    } else {
      wet = false;
    }
  }

  probe.wet = wet;
  if (!wet) return;
  if (moved && dt >= 1e5) {
    probe.ph = targetPh;
    probe.tdsPpm = targetTds;
    return;
  }
  const double k = 1 - exp(-dt / std::max(0.05, timeConstant));
  if (isfinite(targetPh)) probe.ph += (targetPh - probe.ph) * k;
  probe.tdsPpm += (targetTds - probe.tdsPpm) * k;
}

void World::step(double dt, const Actuators &actuators) {
  seconds += dt;
  rippleClock += dt;
  warnings.clear();
  mixer.capacity = params.get("mixer.capacity");
  plant.capacity = params.get("plant.capacity");
  const bool supply = params.choice("fault.pumpSupply") == 0;

  mixer.inflow = mixer.outflow = plant.inflow = plant.outflow = 0;

  // Submersible output pump in the mixer reservoir
  {
    PumpState &pump = outputPump;
    pump.motorOn = actuators.outputPumpRelay && supply;
    pump.direction = pump.motorOn ? 1 : 0;
    pump.flow = 0;
    pump.dry = false;
    if (pump.motorOn) {
      pump.runSeconds += dt;
      const double intake = params.get("mixer.pumpIntake");
      if (params.choice("fault.outputPump") == 1) {
        warnOnce("Submersible pump jammed");
      } else if (mixer.solution.litres <= intake) {
        pump.dry = true;
        warnOnce("Submersible pump is running dry");
      } else {
        const double litres = std::min(params.get("output.flow") / 60 * dt, mixer.solution.litres - intake);
        transfer(mixer, plant, litres);
        pump.flow = litres / dt * 60;
        pump.litres += litres;
      }
    }
    mixer.outflow += pump.flow;
    plant.inflow += pump.flow;
  }

  // Self-priming input pump next to the plant reservoir
  {
    PumpState &pump = inputPump;
    const int fault = params.choice("fault.inputPump");
    pump.motorOn = actuators.inputPumpRelay && supply && fault != 2;
    pump.direction = pump.motorOn ? 1 : 0;
    pump.flow = 0;
    pump.dry = false;
    const double primeTime = params.get("input.prime");
    if (actuators.inputPumpRelay && supply && fault == 2) warnOnce("Self-priming pump jammed");
    if (pump.motorOn) {
      pump.runSeconds += dt;
      const double intake = params.get("plant.hoseIntake");
      if (plant.solution.litres <= intake || fault == 1) {
        // Sucking air: the pump loses its prime
        pump.primed = 0;
        pump.dry = true;
        warnOnce(fault == 1 ? "Self-priming pump has lost its prime" : "Self-priming pump intake is sucking air");
      } else if (pump.primed < primeTime) {
        pump.primed += dt;
      } else {
        const double litres = std::min(params.get("input.flow") / 60 * dt, plant.solution.litres - intake);
        transfer(plant, mixer, litres);
        pump.flow = litres / dt * 60;
        pump.litres += litres;
      }
    } else if (pump.primed > 0) {
      // The water in the intake hose slowly runs back to the reservoir
      pump.primed = std::max(0.0, pump.primed - dt * std::max(primeTime, 0.1) / 60);
    }
    plant.outflow += pump.flow;
    mixer.inflow += pump.flow;
  }

  // Peristaltic dosing pumps (L298N outputs)
  for (int i = 0; i < 4; ++i) {
    PumpState &pump = peristaltic[i];
    pump.direction = supply ? actuators.peristaltic[i] : 0;
    pump.motorOn = pump.direction != 0;
    pump.flow = 0;
    pump.dry = false;
    if (!pump.motorOn) continue;
    pump.runSeconds += dt;
    const int bottle = bottleOfPump(i);
    const double millilitres = params.get("peristaltic.flow") / 60 * dt;
    if (pump.direction > 0) {
      if (bottle < 0 || bottles[bottle].millilitres <= 0) {
        pump.dry = true;
        warnOnce(bottle < 0 ? pump.name + " (spare, no bottle) is running"
                            : bottles[bottle].name + " bottle is empty");
        continue;
      }
      const double moved = std::min(millilitres, bottles[bottle].millilitres);
      dose(mixer, bottle, moved);
      pump.flow = moved / 1000 / dt * 60;
      pump.litres += moved / 1000;
      mixer.inflow += pump.flow;
    } else {
      // The dosing tube ends above the water, so running backwards only sucks air
      pump.dry = true;
      warnOnce(pump.name + " is running backwards");
    }
  }

  // Plants drink and water evaporates from the plant reservoir
  const double uptake = std::min(plant.solution.litres, params.get("plant.uptake") / 3600 * dt);
  plant.solution.litres -= uptake;

  // CO2 exchange with the air (faster when the inflow splashes)
  for (Tank *tank : {&mixer, &plant}) {
    if (tank->solution.litres < 1e-6) continue;
    const double timeConstant = tank->inflow > 0 ? 600 : 3600;
    const double excess = (dissolvedCo2(tank->solution) - co2Equilibrium) * tank->solution.litres;
    tank->solution.carbonMmol = std::max(0.0, tank->solution.carbonMmol - excess * std::min(1.0, dt / timeConstant));
  }

  // Overflow
  for (Tank *tank : {&mixer, &plant}) {
    if (tank->solution.litres > tank->capacity) {
      const double excess = tank->solution.litres - tank->capacity;
      tank->solution.take(excess);
      tank->spilled += excess;
    }
    if (tank->spilled > 0) {
      char text[96];
      snprintf(text, sizeof(text), "%s overflowed: %.2f L spilled", tank->name.c_str(), tank->spilled);
      warnOnce(text);
    }
    const double stirring = std::min(1.0, tank->inflow / 3);
    const double wave = 0.004 * stirring * sin(rippleClock * 2 * M_PI / 1.3) * (0.7 + 0.3 * sin(rippleClock * 5.1));
    tank->ripple += (wave - tank->ripple) * std::min(1.0, dt * 5);
  }

  updateFloat(mixer, params.get("mixer.float"), params.choice("fault.mixerFloat"));
  updateFloat(plant, params.get("plant.float"), params.choice("fault.plantFloat"));
  updateProbe(phProbe, true, dt);
  updateProbe(tdsProbe, false, dt);
}

Json World::toJson() const {
  Json json = Json::object();
  json.set("seconds", seconds);
  json.set("mixer", mixer.solution.toJson());
  json.set("mixerSpilled", mixer.spilled);
  json.set("plant", plant.solution.toJson());
  json.set("plantSpilled", plant.spilled);
  Json bottleVolumes = Json::array();
  for (const auto &bottle : bottles) bottleVolumes.push(bottle.millilitres);
  json.set("bottles", bottleVolumes);
  json.set("inputPumpPrimed", inputPump.primed);
  json.set("phProbe", Json::object().set("ph", phProbe.ph).set("tds", phProbe.tdsPpm));
  json.set("tdsProbe", Json::object().set("ph", tdsProbe.ph).set("tds", tdsProbe.tdsPpm));
  return json;
}

void World::fromJson(const Json &json) {
  if (!json.isObject()) return;
  seconds = json["seconds"].asNumber();
  mixer.solution = Solution::fromJson(json["mixer"]);
  mixer.spilled = json["mixerSpilled"].asNumber();
  plant.solution = Solution::fromJson(json["plant"]);
  plant.spilled = json["plantSpilled"].asNumber();
  for (size_t i = 0; i < bottles.size() && i < json["bottles"].size(); ++i) {
    bottles[i].millilitres = json["bottles"][i].asNumber();
  }
  inputPump.primed = json["inputPumpPrimed"].asNumber();
  phProbe.location = params.choice("ph.location");
  tdsProbe.location = params.choice("tds.location");
  phProbe.ph = json["phProbe"]["ph"].asNumber(7);
  tdsProbe.tdsPpm = json["tdsProbe"]["tds"].asNumber(0);
}

static Json tankJson(const Tank &tank) {
  Json json = Json::object();
  const double ph = tank.solution.ph();
  json.set("name", tank.name);
  json.set("volume", tank.solution.litres);
  json.set("capacity", tank.capacity);
  json.set("level", tank.level());
  json.set("ripple", tank.ripple);
  json.set("tds", tank.solution.tdsPpm());
  json.set("ph", isfinite(ph) ? Json(ph) : Json());
  json.set("co2", dissolvedCo2(tank.solution));
  json.set("inflow", tank.inflow);
  json.set("outflow", tank.outflow);
  json.set("spilled", tank.spilled);
  json.set("floatHeight", tank.floatHeight);
  json.set("floatClosed", tank.floatClosed);
  return json;
}

static Json pumpJson(const PumpState &pump) {
  Json json = Json::object();
  json.set("name", pump.name);
  json.set("on", pump.motorOn);
  json.set("direction", pump.direction);
  json.set("flow", pump.flow);
  json.set("dry", pump.dry);
  json.set("primed", pump.primed);
  json.set("runSeconds", pump.runSeconds);
  json.set("litres", pump.litres);
  return json;
}

static Json probeJson(const ProbeState &probe) {
  Json json = Json::object();
  json.set("location", probe.location);
  json.set("wet", probe.wet);
  json.set("ph", probe.ph);
  json.set("tds", probe.tdsPpm);
  return json;
}

Json World::snapshot() const {
  Json json = Json::object();
  json.set("seconds", seconds);
  json.set("mixer", tankJson(mixer));
  json.set("plant", tankJson(plant));
  Json bottleList = Json::array();
  for (const auto &bottle : bottles) {
    bottleList.push(Json::object()
                        .set("name", bottle.name)
                        .set("contents", bottle.contents)
                        .set("ml", bottle.millilitres)
                        .set("capacity", bottle.capacity));
  }
  json.set("bottles", bottleList);
  json.set("outputPump", pumpJson(outputPump));
  json.set("inputPump", pumpJson(inputPump));
  Json peristalticList = Json::array();
  for (int i = 0; i < 4; ++i) peristalticList.push(pumpJson(peristaltic[i]).set("bottle", bottleOfPump(i)));
  json.set("peristaltic", peristalticList);
  json.set("phProbe", probeJson(phProbe));
  json.set("tdsProbe", probeJson(tdsProbe));
  Json warningList = Json::array();
  for (const auto &warning : warnings) warningList.push(warning);
  json.set("warnings", warningList);
  return json;
}

}  // namespace sim
