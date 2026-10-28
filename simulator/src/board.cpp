#include "board.h"

#include <math.h>

#include <algorithm>

#include "Arduino.h"
#include "sim_api.h"

namespace sim {

namespace {

// PH-4502C output that matches the default calibration of the firmware (defaultPhCalibration, measured at the
// ADC through the 10k/20k divider): pH 7.00 -> 2000.9 mV and pH 4.00 -> 2353.8 mV at the ADC
const double phBoardMilliVoltsAtPh7 = 2000.9 * 1.5;
const double phBoardMilliVoltsPerPh = (2353.8 - 2000.9) / 3 * 1.5;
// Keyestudio TDS meter output that matches defaultTdsCalibration: 51.5 ppm -> 0 mV and 1000 ppm -> 2366 mV at the ADC
const double tdsAtZeroMilliVolts = 51.5;
const double tdsMilliVoltsPerPpm = 2366.0 / (1000 - 51.5) * 1.5;
const double dividerRatio = 20.0 / 30.0;  // R1/R2, R3/R4, R5/R6
const double sensorWarmUpSeconds = 0.35;  // Time constant of the board outputs after power is switched on

// ESP32 ADC with 11 dB attenuation and the eFuse calibration used by analogReadMilliVolts(): inputs below about
// 140 mV read as 142 mV and the reading saturates at about 3.13 V.
const double adcMinimumMilliVolts = 142;
const double adcMaximumMilliVolts = 3134;

bool isInputOnlyPin(uint8_t pin) { return pin >= 34; }

}  // namespace

Board::Board(Params &params, World &world) : params(params), world(world), random(12345) {}

// Pins keep the modes set before setup() (library constructors run first and may configure their pins)
void Board::powerOn() {
  // The 74HC595 powers up in a random state (OE is tied to GND, so the outputs are live)
  srShift = random() & 0xFF;
  srLatch = random() & 0xFF;
  srClockLevel = srLatchLevel = 0;
  phBoard = SensorBoard();
  tdsBoard = SensorBoard();
  phBoard.powered = srOutput(sr::PH_ON);
  tdsBoard.powered = srOutput(sr::TDS_ON);
  for (uint8_t pin = 0; pin < 40; ++pin) pinState[pin].level = resolveLevel(pin);
}

double Board::noise(double sigma) { return sigma * gaussian(random); }

// GPIO

int Board::resolveLevel(uint8_t pin) const {
  const Pin &p = pinState[pin];
  if ((p.mode & OUTPUT) == OUTPUT && (p.mode & OPEN_DRAIN) == 0) return p.out;

  switch (pin) {
    case pins::BUTTON_IO0:  // 10k pull-up on the T-Display
      return buttons[IO0] ? LOW : HIGH;
    case pins::BUTTON_IO35:  // External pull-up on the T-Display
      return buttons[IO35] ? LOW : HIGH;
    case pins::ENC_SW:
      if (buttons[ENCODER_BUTTON]) return LOW;
      break;
    case pins::ABORT_SW:
      if (buttons[ABORT_BUTTON]) return LOW;
      break;
    case pins::ENC_A:
      if (encoderA) return LOW;
      break;
    case pins::ENC_B:
      if (encoderB) return LOW;
      break;
    case pins::FLOAT_RES:  // R7 pull-up, the float switch shorts to GND
      return world.plant.floatClosed ? LOW : HIGH;
    case pins::FLOAT_MIXER:  // R8 pull-up
      return world.mixer.floatClosed ? LOW : HIGH;
  }
  // GPIO 34-39 have no internal pull resistors
  if (!isInputOnlyPin(pin)) {
    if ((p.mode & PULLUP) == PULLUP) return HIGH;
    if ((p.mode & PULLDOWN) == PULLDOWN) return LOW;
  }
  return LOW;  // Floating
}

void Board::updateInput(uint8_t pin) {
  Pin &p = pinState[pin];
  const int level = resolveLevel(pin);
  if (level == p.level) return;
  p.level = level;
  if (pin == pins::SR_SER || pin == pins::SR_SCK || pin == pins::SR_RCK || pin == pins::SR_SCL) {
    shiftRegisterPinChanged(pin, level);
  }
  if (p.isr) {
    const bool fire = p.isrMode == CHANGE || (p.isrMode == RISING && level) || (p.isrMode == FALLING && !level)
                      || (p.isrMode == ONLOW && !level) || (p.isrMode == ONHIGH && level);
    if (fire) p.isr();
  }
}

void Board::pinMode(uint8_t pin, uint8_t mode) {
  if (pin >= 40) return;
  // GPIO 34-39 are input only
  if (isInputOnlyPin(pin) && (mode & OUTPUT) == OUTPUT) return;
  pinState[pin].mode = mode;
  updateInput(pin);
}

void Board::digitalWrite(uint8_t pin, uint8_t level) {
  if (pin >= 40) return;
  pinState[pin].out = level;
  updateInput(pin);
  if (pin == pins::TFT_BL) simhal::panelBacklight(pinState[pin].level == HIGH && (pinState[pin].mode & OUTPUT) == OUTPUT);
}

int Board::digitalRead(uint8_t pin) {
  if (pin >= 40) return 0;
  return pinState[pin].level;
}

void Board::attachInterrupt(uint8_t pin, void (*handler)(), int mode) {
  if (pin >= 40) return;
  pinState[pin].isr = handler;
  pinState[pin].isrMode = mode;
}

void Board::detachInterrupt(uint8_t pin) {
  if (pin >= 40) return;
  pinState[pin].isr = nullptr;
}

// 74HC595: data shifts in on the rising edge of SRCLK, the outputs latch on the rising edge of RCLK and a low
// !SRCLR clears the shift register (not the outputs)
void Board::shiftRegisterPinChanged(uint8_t pin, int level) {
  const bool clear = pinState[pins::SR_SCL].level == LOW;
  if (pin == pins::SR_SCL && clear) srShift = 0;
  if (pin == pins::SR_SCK) {
    if (level && !srClockLevel && !clear) {
      srShift = (uint8_t)((srShift << 1) | (pinState[pins::SR_SER].level ? 1 : 0));
    }
    srClockLevel = level;
  }
  if (pin == pins::SR_RCK) {
    if (level && !srLatchLevel) {
      srLatch = srShift;
      const bool phOn = srOutput(sr::PH_ON);
      const bool tdsOn = srOutput(sr::TDS_ON);
      if (phOn && !phBoard.powered) phBoard.poweredAt = now;
      if (tdsOn && !tdsBoard.powered) tdsBoard.poweredAt = now;
      phBoard.powered = phOn;
      tdsBoard.powered = tdsOn;
    }
    srLatchLevel = level;
  }
}

// ANALOG

bool Board::motorsRunning() const {
  if (world.outputPump.motorOn || world.inputPump.motorOn) return true;
  for (const auto &pump : world.peristaltic) {
    if (pump.motorOn) return true;
  }
  return false;
}

// Voltage at a board output (before the divider). The probe boards are powered through their GND (low-side
// switch); without power their outputs are pulled to ground by the divider.
double Board::sensorMilliVolts(uint8_t pin) {
  const double temperature = params.get("temperature");
  const bool bothPowered = phBoard.powered && tdsBoard.powered;
  const bool sameLiquid = world.phProbe.wet && world.tdsProbe.wet && world.phProbe.location <= 1
                          && world.phProbe.location == world.tdsProbe.location;

  if (pin == pins::PH3V || pin == pins::TEMP3V) {
    if (!phBoard.powered) return fabs(noise(1));
    const double warmUp = 1 - exp(-(now - phBoard.poweredAt) / sensorWarmUpSeconds);
    double output;
    if (pin == pins::TEMP3V) {
      output = 1700 + 12 * temperature;  // Thermistor output of the PH-4502C (not calibrated)
    } else if (params.choice("fault.phProbe") == 1 || !world.phProbe.wet) {
      // Floating electrode input: the output wanders and picks up hum
      phDrift += noise(15);
      phDrift *= 0.995;
      output = phBoardMilliVoltsAtPh7 - 400 + phDrift + noise(80);
    } else {
      const double slope = phBoardMilliVoltsPerPh * params.get("ph.slope") / 100 * (temperature + 273.15) / 298.15;
      output = phBoardMilliVoltsAtPh7 + params.get("ph.offset") + (7 - world.phProbe.ph) * slope;
      // The TDS probe's excitation current flows through the solution and shifts the pH electrode potential
      if (bothPowered && sameLiquid) output += 150 + noise(25);
    }
    output = std::max(0.0, std::min(5000.0, output)) * warmUp;
    return output + noise(params.get("noise"));
  }

  if (pin == pins::TDS3V) {
    if (!tdsBoard.powered) return fabs(noise(1));
    const double warmUp = 1 - exp(-(now - tdsBoard.poweredAt) / sensorWarmUpSeconds);
    double output = 0;
    if (params.choice("fault.tdsProbe") == 0 && world.tdsProbe.wet) {
      // Conductivity (and the board output) rises about 2 % per degree
      const double tds = world.tdsProbe.tdsPpm * (1 + 0.02 * (temperature - 25));
      output = (tds - tdsAtZeroMilliVolts) * tdsMilliVoltsPerPpm * (1 + params.get("tds.gain") / 100);
      if (bothPowered && sameLiquid) output += 40 + noise(10);
    }
    output = std::max(0.0, std::min(4600.0, output)) * warmUp;
    return std::max(0.0, output + noise(params.get("noise")));
  }

  if (pin == 34) {  // Battery voltage divider of the T-Display, enabled by GPIO14 (no battery: USB charger voltage)
    return pinState[pins::ADC_EN].level ? 2 * 2100 / 1.5 : 0;
  }
  return fabs(noise(20));  // Unconnected input
}

uint32_t Board::analogMilliVolts(uint8_t pin) {
  if (pin >= 40) return 0;
  double milliVolts = sensorMilliVolts(pin) * (pin == 34 ? 0.5 * 1.5 : dividerRatio);
  // Switching motors and relays induce spikes; the firmware's trimmed mean is meant to remove them
  if (motorsRunning() && uniform(random) < 0.08) {
    milliVolts += (uniform(random) < 0.5 ? -1 : 1) * params.get("motorNoise") * (0.3 + 0.7 * uniform(random));
  }
  milliVolts += noise(2);  // ADC noise
  if (milliVolts < adcMinimumMilliVolts) milliVolts = adcMinimumMilliVolts + fabs(noise(1));
  if (milliVolts > adcMaximumMilliVolts) milliVolts = adcMaximumMilliVolts;
  return (uint32_t)lround(milliVolts);
}

int Board::analogRaw(uint8_t pin) {
  const double milliVolts = analogMilliVolts(pin);
  const double raw = (milliVolts - adcMinimumMilliVolts) / (adcMaximumMilliVolts - adcMinimumMilliVolts) * 4095;
  return (int)std::max(0.0, std::min(4095.0, raw));
}

// SIMULATOR SIDE

void Board::setButton(Button button, bool pressed) {
  buttons[button] = pressed;
  switch (button) {
    case IO0:
      updateInput(pins::BUTTON_IO0);
      break;
    case IO35:
      updateInput(pins::BUTTON_IO35);
      break;
    case ENCODER_BUTTON:
      updateInput(pins::ENC_SW);
      break;
    case ABORT_BUTTON:
      updateInput(pins::ABORT_SW);
      break;
    default:
      break;
  }
}

void Board::setEncoderContacts(bool aClosed, bool bClosed) {
  encoderA = aClosed;
  encoderB = bClosed;
  updateInput(pins::ENC_A);
  updateInput(pins::ENC_B);
}

void Board::update(double time) {
  now = time;
  updateInput(pins::FLOAT_RES);
  updateInput(pins::FLOAT_MIXER);
}

bool Board::relayEnergized(int relay) const {
  const uint8_t pin = relay == 0 ? pins::RELAY1 : pins::RELAY2;
  const Pin &p = pinState[pin];
  if ((p.mode & OUTPUT) != OUTPUT) return false;  // Undriven input: the relay board's pull resistor keeps it off
  const int activeLevel = params.choice("relay.activeLevel") == 0 ? HIGH : LOW;
  return p.level == activeLevel;
}

// L298N: ENx enables the H-bridge; INa high / INb low drives forwards, the opposite backwards and equal inputs
// brake the motor. On this board IN1/IN3 are both PPUMP_A and IN2/IN4 are both PPUMP_B.
int Board::l298nOutput(int driver, int channel) const {
  static const int enableBits[2][2] = {{sr::PPUMP1_EN, sr::PPUMP2_EN}, {sr::PPUMP3_EN, sr::PPUMP4_EN}};
  if (!srOutput(enableBits[driver][channel])) return 0;
  const bool a = srOutput(sr::PPUMP_A);
  const bool b = srOutput(sr::PPUMP_B);
  if (a && !b) return 1;
  if (!a && b) return -1;
  return 0;
}

Actuators Board::actuators() const {
  Actuators actuators;
  actuators.outputPumpRelay = relayEnergized(0);
  actuators.inputPumpRelay = relayEnergized(1);
  actuators.peristaltic[0] = l298nOutput(0, 0);
  actuators.peristaltic[1] = l298nOutput(0, 1);
  actuators.peristaltic[2] = l298nOutput(1, 0);
  actuators.peristaltic[3] = l298nOutput(1, 1);
  return actuators;
}

Json Board::snapshot() const {
  Json json = Json::object();
  Json pinList = Json::object();
  static const uint8_t shownPins[] = {0, 2, 4, 12, 13, 14, 15, 17, 21, 22, 25, 26, 27, 32, 33, 35, 36, 37, 38, 39};
  for (uint8_t pin : shownPins) {
    const Pin &p = pinState[pin];
    pinList.set(std::to_string(pin), Json::object().set("mode", p.mode).set("level", p.level).set("isr", p.isr != nullptr));
  }
  json.set("pins", pinList);
  json.set("shiftRegister", srLatch);
  json.set("shiftRegisterInternal", srShift);
  Json leds = Json::object();
  leds.set("LED1", pinState[pins::LED_ERROR].level == HIGH && (pinState[pins::LED_ERROR].mode & OUTPUT) == OUTPUT);
  leds.set("LED2", tdsBoard.powered);
  leds.set("LED3", phBoard.powered);
  leds.set("D4", srOutput(sr::PPUMP1_EN));
  leds.set("D5", srOutput(sr::PPUMP2_EN));
  leds.set("D6", srOutput(sr::PPUMP3_EN));
  leds.set("D7", srOutput(sr::PPUMP4_EN));
  leds.set("D8", srOutput(sr::PPUMP_A));
  leds.set("D9", srOutput(sr::PPUMP_B));
  leds.set("backlight", pinState[pins::TFT_BL].level == HIGH && (pinState[pins::TFT_BL].mode & OUTPUT) == OUTPUT);
  json.set("leds", leds);
  Json relays = Json::array();
  relays.push(relayEnergized(0));
  relays.push(relayEnergized(1));
  json.set("relays", relays);
  Json drivers = Json::array();
  for (int driver = 0; driver < 2; ++driver) {
    static const int enableBits[2][2] = {{sr::PPUMP1_EN, sr::PPUMP2_EN}, {sr::PPUMP3_EN, sr::PPUMP4_EN}};
    Json d = Json::object();
    d.set("ena", srOutput(enableBits[driver][0]));
    d.set("enb", srOutput(enableBits[driver][1]));
    d.set("in1", srOutput(sr::PPUMP_A));
    d.set("in2", srOutput(sr::PPUMP_B));
    d.set("outA", l298nOutput(driver, 0));
    d.set("outB", l298nOutput(driver, 1));
    drivers.push(d);
  }
  json.set("l298n", drivers);
  json.set("phBoard", Json::object().set("powered", phBoard.powered).set("poweredFor", phBoard.powered ? now - phBoard.poweredAt : 0));
  json.set("tdsBoard", Json::object().set("powered", tdsBoard.powered).set("poweredFor", tdsBoard.powered ? now - tdsBoard.poweredAt : 0));
  Json buttonList = Json::object();
  buttonList.set("io0", buttons[IO0]);
  buttonList.set("io35", buttons[IO35]);
  buttonList.set("encoder", buttons[ENCODER_BUTTON]);
  buttonList.set("abort", buttons[ABORT_BUTTON]);
  json.set("buttons", buttonList);
  json.set("encoderA", encoderA);
  json.set("encoderB", encoderB);
  return json;
}

}  // namespace sim
