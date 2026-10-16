
#define DEBUG
// Use 115200 speed for programming
// Requires v2.x of the ESP32 board package (for analogReadMilliVolts) https://dl.espressif.com/dl/package_esp32_index.json

// TODO: The 5V on T-Display gives out just 3V for some reason

#include <KeyDetector.h>
#include <AiEsp32RotaryEncoder.h>
#include <Adafruit_ST7789.h>
#include <GEM_adafruit_gfx.h>
#include <Preferences.h>
#include <SPI.h>
#include <ph4502c_sensor.h>

// BUTTON HANDLING
const byte rightPin = 0;
Key keys[] = { { GEM_KEY_RIGHT, rightPin } };
KeyDetector myKeyDetector(keys, sizeof(keys) / sizeof(Key));

// TFT INITIALISATION
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 23
#define TFT_BL 4

// Hardware SPI (the pins are routed through the GPIO matrix). Software SPI made every screen redraw very slow.
#define TFT_SPI_FREQUENCY 40000000
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, TFT_CS, TFT_DC, TFT_RST);


const byte phSensorIn = 36;
const byte temperatureSensorIn = 39;
const byte tdsSensorIn = 32;

// Float switches pull the pin low when triggered. GPIO 34-39 have no internal pull ups, so external ones are needed.
const byte mixerFloatSensor = 38;
const byte reservoirFloatSensor = 37;

const byte ENC_SW = 21;
const byte ENC_A = 17;
const byte ENC_B = 22;


const byte shiftRegisterData = 2;    // SR_SER
const byte shiftRegisterClock = 15;  // SR_SCK
const byte shiftRegisterLatch = 33;  // SR_RCK
const byte shiftRegisterReset = 25;  // SR_SCL

const byte ABORT_SW = 13;

const byte LED_ERROR = 12;

// Shift register outputs
const byte PPUMP1_EN = 1;
const byte PPUMP2_EN = 2;
const byte PPUMP4_EN = 3;
const byte PPUMP_B = 4;
const byte PPUMP_A = 5;
const byte PPUMP3_EN = 6;

const byte TDS_ON = 7;
const byte PH_ON = 0;

// GLOBAL STATE
float targetTds = 1000;
float targetPh = 6;
float currentPh = 7;
float currentTds = 0;
char currentPhString[17] = "-";
char currentTdsString[17] = "-";
bool enablePrint = false;
const char *commandFromMenu = nullptr;

enum Rotation {
  Forward,
  Stopped,
  Backward
};

struct PeristalticPump {
  byte enablePin;
  byte directionAPin;
  byte directionBPin;
};

// Relay controlled pump that is blocked while its float switch is triggered
struct Pump {
  const char *name;
  byte relayPin;
  byte floatSwitchPin;
  bool requested;
  bool overflow;
  unsigned long lastTriggeredAt;
};

// Two point linear calibration from sensor voltage to the measured value. Points are stored in flash and
// survive restarts. New calibration points replace the low or high point depending on the reference value.
struct Calibration {
  float lowValue;
  float lowMilliVolts;
  float highValue;
  float highMilliVolts;
};

enum Sensor {
  NO_SENSOR,
  TDS_SENSOR,
  PH_SENSOR
};

enum State {
  IDLE,
  CIRCULATE,
  EMPTY,
  FILL,
  MEASURE,
  PLOT,
  ADJUST_PH,
  ADJUST_TDS,
  ADJUST_ALL,
  PUMP_PH,
  PUMP_NUTRIENT_A,
  PUMP_NUTRIENT_B
};

enum phAdjustState {
  PH_MEASURE,
  PH_PUMP,
  PH_CIRCULATE,
  PH_SETTLE
};

enum tdsAdjustState {
  TDS_MEASURE,
  TDS_A_PUMP,
  TDS_B_PUMP,
  TDS_CIRCULATE,
  TDS_SETTLE
};

enum Progress {
  IN_PROGRESS,
  FINISHED,
  FAILED
};

State state = IDLE;
phAdjustState phState = PH_MEASURE;
byte phAdjustmentRound = 0;
tdsAdjustState tdsState = TDS_MEASURE;
byte tdsAdjustmentRound = 0;
bool pumpBothNutrients = false;
unsigned long plotDurationSeconds = 120;

PeristalticPump acidPump;
PeristalticPump nutrientAPump;
PeristalticPump nutrientBPump;

Pump inputPump;
Pump outputPump;

// Level that switches a relay on. Many relay modules are active low; change this if the pumps run when they should be stopped.
const byte relayOnLevel = HIGH;
// How long a float switch needs to stay released before its pump is allowed to run again
const unsigned long floatSwitchReleaseMilliseconds = 1000;

#define PH4502C_TEMPERATURE_PIN 34
#define PH4502C_PH_PIN 35
#define PH4502C_PH_TRIGGER_PIN 14
#define PH4502C_CALIBRATION 14.8f
#define PH4502C_READING_INTERVAL 100
#define PH4502C_READING_COUNT 100
// NOTE: The ESP32 ADC has a 12-bit resolution (while most arduinos have 10-bit)
#define ADC_RESOLUTION 4096.0f

// Create an instance of the PH4502C_Sensor
PH4502C_Sensor ph4502c(
  phSensorIn,
  temperatureSensorIn,
  PH4502C_CALIBRATION,
  PH4502C_READING_INTERVAL,
  PH4502C_READING_COUNT,
  ADC_RESOLUTION);

// FORWARD DECLARATIONS
void fill();
void empty();
void circulate();
void adjustAll();
void adjustPh();
void adjustTds();
void pumpA();
void pumpB();
void pumpAcid();
void measure();
void abortFromMenu();

// UTILITY FUNCTIONS

// Parses a number, rejecting empty or malformed input (String::toFloat() silently returns 0 for those)
bool parseNumber(const String &text, float &value) {
  if (text.length() == 0) return false;
  char *end;
  value = strtod(text.c_str(), &end);
  return *end == '\0' && isfinite(value);
}

// Timer of the currently running timed step (pumping, circulating, settling, plotting). Steps are run one at a
// time, so a single timer is shared. It must be cancelled whenever an operation is interrupted, otherwise the
// next operation would continue the stale count down (see abortOperation()).
struct {
  unsigned long startedAt = 0;
  unsigned long durationMilliseconds = 0;
  bool running = false;
} countDown;

// Calls start() on the first call and stop() once durationSeconds has passed. Returns true when done.
bool runCountDown(unsigned long durationSeconds, void (*start)(), void (*stop)()) {
  if (!countDown.running) {
    countDown.running = true;
    countDown.startedAt = millis();
    countDown.durationMilliseconds = durationSeconds * 1000UL;
    start();
  } else if (millis() - countDown.startedAt >= countDown.durationMilliseconds) {
    countDown.running = false;
    stop();
    return true;
  }
  return false;
}

void cancelCountDown() {
  countDown.running = false;
}

void printData() {
  Serial.println("print data");
  // If enablePrint flag is set to true (checkbox on screen is checked)...
  if (enablePrint) {
    // ...print the number to Serial
    Serial.print("TDS target is: ");
    Serial.println(targetTds);

    Serial.print("pH target is: ");
    Serial.println(targetPh);
  } else {
    Serial.println("Printing is disabled, sorry:(");
  }
}

void printUsage() {
  Serial.println("Commands:");
  Serial.println("m # Measure and report pH and TDS");
  Serial.println("t 1000 # Adjust TDS to 1000 (without value: use target from menu)");
  Serial.println("p 6.0 # Adjust pH to 6.0 (without value: use target from menu)");
  Serial.println("a 1000 6.0 # Adjust TDS to 1000 and pH to 6.0 (without values: use targets from menu)");
  Serial.println("e # Empty tank");
  Serial.println("f # Fill tank");
  Serial.println("c # Circulate");
  Serial.println("u # Pump pH");
  Serial.println("n # Pump nutrients A and B (n a / n b: pump only A / B)");
  Serial.println("l 10 # Plot pH, TDS and temperature values for 10 seconds");
  Serial.println("k # Show sensor calibration");
  Serial.println("k ph 7.0 # Calibrate pH with the probe in pH 7.0 buffer (use pH 4.0 and 7.0 buffers)");
  Serial.println("k tds 707 # Calibrate TDS with the probe in a 707 ppm solution (below 100 ppm sets the low point)");
  Serial.println("k reset # Restore default calibration");
  Serial.println("1 # Read pH with the PH4502C library");
  Serial.println("! # Abort current operation (stop all pumps) and go to idle");
}

// MENU SETUP
GEMPage menuPageMain("Main Menu");

GEMItem pHValue("pH", currentPhString, true);
GEMItem TDSValue("TDS", currentTdsString, true);

GEMItem measureItem("Measure", measure);
GEMItem menuItemStop("Stop", abortFromMenu);

// PUMP

GEMPage menuPagePump("Pump", menuPageMain);
GEMItem menuItemPump("Pump", menuPagePump);

GEMPage menuPageReservoir("Reservoir", menuPagePump);
GEMItem menuItemReservoir("Reservoir", menuPageReservoir);

GEMItem menuItemCirculate("Circulate", circulate);
GEMItem menuItemFill("Fill", fill);
GEMItem menuItemEmpty("Empty", empty);

GEMPage menuPageNutrients("Nutrients", menuPagePump);
GEMItem menuItemNutrients("Nutrients", menuPageNutrients);

GEMItem menuItemPumpA("Pump A", pumpA);
GEMItem menuItemPumpB("Pump B", pumpB);
GEMItem menuItemPumpAcid("Pump acid", pumpAcid);

// ADJUST

GEMPage menuPageAdjust("Adjust", menuPageMain);
GEMItem menuItemAdjust("Adjust", menuPageAdjust);

GEMPage menuPageTarget("Target Values", menuPageAdjust);
GEMItem menuItemTarget("Target Values", menuPageTarget);

GEMItem menuItemTargetPh("Target pH:", targetPh);
GEMItem menuItemTargetTds("Target TDS:", targetTds);

GEMItem menuItemAdjustAll("Adjust all", adjustAll);
GEMItem menuItemAdjustPh("Adjust pH", adjustPh);
GEMItem menuItemAdjustTds("Adjust TDS", adjustTds);

GEMPage menuPageAdjusting("Adjusting", menuPageAdjust);
GEMItem menuItemCurrentPh("Current pH:", currentPh, true);
GEMItem menuItemCurrentTds("Current TDS:", currentTds, true);

GEM_adafruit_gfx menu(tft, GEM_POINTER_ROW, GEM_ITEMS_COUNT_AUTO);

byte registerState = 0;

void setShiftRegisterValue(byte state) {
  registerState = state;
  digitalWrite(shiftRegisterLatch, LOW);
  shiftOut(shiftRegisterData, shiftRegisterClock, MSBFIRST, registerState);
  digitalWrite(shiftRegisterLatch, HIGH);

#ifdef DEBUG
  Serial.println();
  Serial.print("Setting shift register to: 0b");
  for (int i = 7; i >= 0; i--) {
    Serial.print(bitRead(registerState, i));
  }
  Serial.println();
#endif
}

void setShiftRegisterValue(byte pin, bool state) {
  setShiftRegisterValue((registerState & ~(1 << pin)) | ((byte)state << pin));
}

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ENC_A, ENC_B, ENC_SW, -1, 4, false);

void rotary_onButtonClick() {
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500) {
    return;
  }
  lastTimePressed = millis();
  Serial.print("button pressed ");
  Serial.print(millis());
  Serial.println(" milliseconds after restart");
  menu.registerKeyPress(GEM_KEY_OK);
}

void rotary_loop() {
  //dont print anything unless value changed
  float changed = rotaryEncoder.encoderChanged();
  if (changed) {
    Serial.print("Changed: ");
    Serial.println(changed);
    Serial.print("Value: ");
    Serial.println(rotaryEncoder.readEncoder());
    if (changed > 0) {
      menu.registerKeyPress(GEM_KEY_UP);
    } else {
      menu.registerKeyPress(GEM_KEY_DOWN);
    }
  }
  if (rotaryEncoder.isEncoderButtonClicked()) {
    rotary_onButtonClick();
  }
}

void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

// Drives the relay from the requested state and the float switch. A triggered float switch blocks the pump
// immediately; the pump continues once the switch has stayed released for floatSwitchReleaseMilliseconds.
void updatePump(Pump &pump) {
  const bool triggered = !digitalRead(pump.floatSwitchPin);
  if (triggered) {
    pump.lastTriggeredAt = millis();
    if (!pump.overflow) {
      pump.overflow = true;
      Serial.print(pump.name);
      Serial.println(" float switch triggered, blocking pump");
    }
  } else if (pump.overflow && millis() - pump.lastTriggeredAt >= floatSwitchReleaseMilliseconds) {
    pump.overflow = false;
    Serial.print(pump.name);
    Serial.println(" float switch released, unblocking pump");
  }

  digitalWrite(pump.relayPin, pump.requested && !pump.overflow ? relayOnLevel : !relayOnLevel);
}

void updatePumps() {
  updatePump(inputPump);
  updatePump(outputPump);
  digitalWrite(LED_ERROR, inputPump.overflow || outputPump.overflow ? HIGH : LOW);
}

void engagePump(Pump &pump, bool pumping) {
  Serial.print("Set ");
  Serial.print(pump.name);
  Serial.print(" pumping to: ");
  Serial.println(pumping);
  pump.requested = pumping;
  updatePumps();
}

void engagePump(PeristalticPump pump, Rotation rotation) {
  if (rotation == Stopped) {
    setShiftRegisterValue(pump.enablePin, 0);
  } else {
    if (rotation == Forward) {
      setShiftRegisterValue(pump.directionAPin, 1);
      setShiftRegisterValue(pump.directionBPin, 0);
    } else {
      setShiftRegisterValue(pump.directionAPin, 0);
      setShiftRegisterValue(pump.directionBPin, 1);
    }
    setShiftRegisterValue(pump.enablePin, 1);
  }
}

void stopAllPumps() {
  engagePump(inputPump, false);
  engagePump(outputPump, false);
  engagePump(nutrientAPump, Stopped);
  engagePump(nutrientBPump, Stopped);
  engagePump(acidPump, Stopped);
}

// SENSOR POWER
// Only one of the pH and TDS sensors is powered at a time so that the probes do not interfere with each other.
// After powering a sensor, its readings are not used until it has had sensorWarmUpMilliseconds to stabilise.
const unsigned long sensorWarmUpMilliseconds = 2000;
Sensor poweredSensor = NO_SENSOR;
unsigned long sensorPoweredAt = 0;

void powerSensor(Sensor sensor) {
  if (sensor == poweredSensor) return;

  byte value = registerState & ~((1 << TDS_ON) | (1 << PH_ON));
  if (sensor == TDS_SENSOR) value |= 1 << TDS_ON;
  if (sensor == PH_SENSOR) value |= 1 << PH_ON;
  setShiftRegisterValue(value);
  cancelSampling();

  poweredSensor = sensor;
  sensorPoweredAt = millis();
}

// Powers the sensor and returns true once it has been powered long enough to be read
bool sensorReady(Sensor sensor) {
  powerSensor(sensor);
  return millis() - sensorPoweredAt >= sensorWarmUpMilliseconds;
}

// SENSOR READING
// The ESP32 ADC is noisy and non-linear, so:
// - analogReadMilliVolts() is used, which corrects the readings with the chip's factory calibration
// - the first reading after switching channels is discarded to let the ADC sample capacitor settle
// - many samples are taken and the lowest and highest quarters are discarded (trimmed mean), which
//   removes the spikes caused e.g. by pump motors and relays
// - readings that are saturated or too noisy (e.g. a disconnected probe) are rejected (NAN)
const byte sensorSampleCount = 32;
const byte sensorSampleIntervalMilliseconds = 10;
const float sensorMaxMilliVolts = 3100;       // ADC saturates around here with 11 dB attenuation
const float sensorMaxSpreadMilliVolts = 100;  // Max spread of the middle half of the samples

// Sampling is spread over loop() iterations (one sample per call) so that the UI stays responsive while measuring
struct {
  bool active = false;
  byte pin;
  byte count;
  unsigned long lastSampleAt;
  uint16_t samples[sensorSampleCount];
} sampler;

void cancelSampling() {
  sampler.active = false;
}

float evaluateSamples(byte pin, uint16_t *samples) {
  // Insertion sort
  for (byte i = 1; i < sensorSampleCount; ++i) {
    uint16_t sample = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > sample) {
      samples[j + 1] = samples[j];
      --j;
    }
    samples[j + 1] = sample;
  }

  const byte trim = sensorSampleCount / 4;
  uint32_t sum = 0;
  for (byte i = trim; i < sensorSampleCount - trim; ++i) {
    sum += samples[i];
  }
  const float milliVolts = (float)sum / (sensorSampleCount - 2 * trim);
  const float spread = samples[sensorSampleCount - trim - 1] - samples[trim];

#ifdef DEBUG
  Serial.print("GPIO");
  Serial.print(pin);
  Serial.print("_mV:");
  Serial.print(milliVolts);
  Serial.print(",GPIO");
  Serial.print(pin);
  Serial.print("_spread_mV:");
  Serial.print(spread);
  Serial.print(",");
#endif

  if (milliVolts >= sensorMaxMilliVolts) {
    Serial.print("Sensor in GPIO ");
    Serial.print(pin);
    Serial.println(" is saturated");
    return NAN;
  }
  if (spread > sensorMaxSpreadMilliVolts) {
    Serial.print("Sensor in GPIO ");
    Serial.print(pin);
    Serial.print(" is too noisy (spread ");
    Serial.print(spread);
    Serial.println(" mV). Is the probe connected?");
    return NAN;
  }
  return milliVolts;
}

// Takes at most one sample per call. Returns true once all samples have been taken, with milliVolts set
// (NAN if the reading is unusable). Non-blocking: call repeatedly until it returns true.
bool sampleMilliVolts(byte pin, float &milliVolts) {
  if (!sampler.active || sampler.pin != pin) {
    sampler.active = true;
    sampler.pin = pin;
    sampler.count = 0;
    analogReadMilliVolts(pin);  // Discarded
    sampler.lastSampleAt = millis();
    return false;
  }
  if (millis() - sampler.lastSampleAt < sensorSampleIntervalMilliseconds) return false;

  sampler.lastSampleAt = millis();
  sampler.samples[sampler.count++] = analogReadMilliVolts(pin);
  if (sampler.count < sensorSampleCount) return false;

  sampler.active = false;
  milliVolts = evaluateSamples(pin, sampler.samples);
  return true;
}

// Blocking version of sampleMilliVolts() for places where responsiveness does not matter (e.g. calibration)
float readMilliVolts(byte pin) {
  float milliVolts;
  while (!sampleMilliVolts(pin, milliVolts)) {
    delay(1);
  }
  return milliVolts;
}

// CALIBRATION

// Temperature the calibration values are referenced to
const float calibrationTemperature = 25;

// Defaults are the previously used fits (valueTopH and valueToTDS) converted from raw ADC values to millivolts
const Calibration defaultPhCalibration = { 4.00, 2353.8, 7.00, 2000.9 };
const Calibration defaultTdsCalibration = { 51.5, 0, 1000, 2366.0 };

// Reference values below these replace the low calibration point, others the high point
const float phCalibrationSplit = 5.5;
const float tdsCalibrationSplit = 100;

const float minCalibrationMilliVoltsDifference = 30;

Calibration phCalibration = defaultPhCalibration;
Calibration tdsCalibration = defaultTdsCalibration;
Preferences preferences;

float applyCalibration(const Calibration &calibration, float milliVolts) {
  return calibration.lowValue
         + (milliVolts - calibration.lowMilliVolts) * (calibration.highValue - calibration.lowValue)
             / (calibration.highMilliVolts - calibration.lowMilliVolts);
}

bool isUsable(const Calibration &calibration) {
  return isfinite(calibration.lowValue) && isfinite(calibration.lowMilliVolts)
         && isfinite(calibration.highValue) && isfinite(calibration.highMilliVolts)
         && calibration.lowValue < calibration.highValue
         && abs(calibration.highMilliVolts - calibration.lowMilliVolts) >= minCalibrationMilliVoltsDifference;
}

void loadCalibration(const char *key, Calibration &calibration, const Calibration &defaultCalibration) {
  Calibration stored;
  if (preferences.getBytes(key, &stored, sizeof(stored)) == sizeof(stored) && isUsable(stored)) {
    calibration = stored;
  } else {
    calibration = defaultCalibration;
  }
}

void loadCalibrations() {
  preferences.begin("mixer", true);
  loadCalibration("pHCal", phCalibration, defaultPhCalibration);
  loadCalibration("TDSCal", tdsCalibration, defaultTdsCalibration);
  preferences.end();
}

void saveCalibrations() {
  preferences.begin("mixer", false);
  preferences.putBytes("pHCal", &phCalibration, sizeof(phCalibration));
  preferences.putBytes("TDSCal", &tdsCalibration, sizeof(tdsCalibration));
  preferences.end();
}

bool addCalibrationPoint(Calibration &calibration, bool lowPoint, float value, float milliVolts) {
  Calibration updated = calibration;
  if (lowPoint) {
    updated.lowValue = value;
    updated.lowMilliVolts = milliVolts;
  } else {
    updated.highValue = value;
    updated.highMilliVolts = milliVolts;
  }
  if (!isUsable(updated)) {
    Serial.println("Calibration rejected: the points are too close to each other. Check the probe and the solution.");
    return false;
  }
  calibration = updated;
  return true;
}

void printCalibration(const char *name, const Calibration &calibration) {
  Serial.print(name);
  Serial.print(" calibration: ");
  Serial.print(calibration.lowValue);
  Serial.print(" @ ");
  Serial.print(calibration.lowMilliVolts);
  Serial.print(" mV, ");
  Serial.print(calibration.highValue);
  Serial.print(" @ ");
  Serial.print(calibration.highMilliVolts);
  Serial.println(" mV");
}

// MEASUREMENTS

// The temperature sensor conversion has not been implemented yet, so readings are compensated
// to the assumed water temperature instead
const bool temperatureSensorCalibrated = false;
const float assumedWaterTemperature = 25;

float voltageToTemperature(float milliVolts) {
  return NAN;  // TODO
}

float measureTemperature() {
  if (!temperatureSensorCalibrated) {
    return assumedWaterTemperature;
  }

  const float temperature = voltageToTemperature(readMilliVolts(temperatureSensorIn));
  if (isnan(temperature) || temperature < 0 || temperature > 50) {
    Serial.println("Invalid temperature reading, using assumed water temperature");
    return assumedWaterTemperature;
  }
  return temperature;
}

void reportTemperature(float temperature) {
  Serial.print("Temperature:");
  Serial.print(temperature);
}

// Converts a pH value from one temperature to another (the electrode slope is proportional to the absolute temperature)
float compensatePh(float pH, float temperature, float fromTemperature) {
  return 7 + (pH - 7) * (fromTemperature + 273.15f) / (temperature + 273.15f);
}

float milliVoltsToPh(float milliVolts, float temperature) {
  if (isnan(milliVolts)) return NAN;

  const float pH = compensatePh(applyCalibration(phCalibration, milliVolts), temperature, calibrationTemperature);
  if (pH < 0 || pH > 14) {
    Serial.println("pH reading out of range. Is the probe connected and calibrated?");
    return NAN;
  }
  return pH;
}

void formatMeasurement(char *target, float value) {
  if (isnan(value)) {
    strcpy(target, "error");
  } else {
    dtostrf(value, 1, 2, target);
  }
}

// Only the main page shows the measurements, so other pages are not redrawn when the values change
void redrawMenu() {
  if (menu.getCurrentMenuPage() == &menuPageMain && menu.readyForKey() && !menu.isEditMode()) {
    menu.drawMenu();
  }
}

void reportPh(float value) {
  Serial.print("pH:");
  Serial.print(value);
  currentPh = value;
  formatMeasurement(currentPhString, value);
  redrawMenu();
}

void reportTds(float value) {
  Serial.print("TDS:");
  Serial.print(value);
  currentTds = value;
  formatMeasurement(currentTdsString, value);
  redrawMenu();
}

// Conductivity rises roughly 2% per degree, so the voltage is converted to what it would be at calibrationTemperature
float compensateTdsMilliVolts(float milliVolts, float temperature) {
  return milliVolts / (1 + 0.02f * (temperature - calibrationTemperature));
}

float milliVoltsToTds(float milliVolts, float temperature) {
  if (isnan(milliVolts)) return NAN;

  return max(applyCalibration(tdsCalibration, compensateTdsMilliVolts(milliVolts, temperature)), 0.0f);
}

// Powers the sensor, waits for it to warm up and reads it. Non-blocking: call repeatedly until it returns true.
bool measureSensor(Sensor sensor, float &value) {
  if (!sensorReady(sensor)) return false;

  float milliVolts;
  if (!sampleMilliVolts(sensor == TDS_SENSOR ? tdsSensorIn : phSensorIn, milliVolts)) return false;

  const float temperature = measureTemperature();
  value = sensor == TDS_SENSOR ? milliVoltsToTds(milliVolts, temperature) : milliVoltsToPh(milliVolts, temperature);
  return true;
}

// Measures TDS and then pH, one sensor powered at a time. Non-blocking: call repeatedly until it returns true.
Sensor measureAllStep = TDS_SENSOR;

bool measureAll() {
  float value;
  if (!measureSensor(measureAllStep, value)) return false;

  if (measureAllStep == TDS_SENSOR) {
    reportTds(value);
    Serial.print(",");
    measureAllStep = PH_SENSOR;
    return false;
  }

  reportTemperature(measureTemperature());
  Serial.print(",");
  reportPh(value);
  Serial.println();
  measureAllStep = TDS_SENSOR;
  return true;
}

// Powers the sensors down and restarts measureAll() from the beginning
void stopMeasuring() {
  measureAllStep = TDS_SENSOR;
  cancelSampling();
  powerSensor(NO_SENSOR);
}

// Blocking calibration measurement
float measureForCalibration(Sensor sensor, byte pin) {
  powerSensor(sensor);
  while (!sensorReady(sensor)) {
    delay(10);
  }
  const float milliVolts = readMilliVolts(pin);
  powerSensor(NO_SENSOR);
  return milliVolts;
}

void calibrate(String args) {
  args.trim();
  args.toLowerCase();

  if (args == "reset") {
    phCalibration = defaultPhCalibration;
    tdsCalibration = defaultTdsCalibration;
    saveCalibrations();
  } else if (args.length() > 0) {
    const int splitLocation = args.indexOf(' ');
    const String sensor = splitLocation < 0 ? args : args.substring(0, splitLocation);
    float reference;
    if (splitLocation < 0 || !parseNumber(args.substring(splitLocation + 1), reference)) {
      Serial.println("Usage: k ph 7.0 / k tds 707");
      return;
    }

    const float temperature = measureTemperature();
    bool calibrated = false;
    if (sensor == "ph" && reference >= 1 && reference <= 13) {
      const float milliVolts = measureForCalibration(PH_SENSOR, phSensorIn);
      if (isnan(milliVolts)) return;
      calibrated = addCalibrationPoint(phCalibration, reference < phCalibrationSplit,
                                       compensatePh(reference, calibrationTemperature, temperature), milliVolts);
    } else if (sensor == "tds" && reference >= 0 && reference <= 5000) {
      const float milliVolts = measureForCalibration(TDS_SENSOR, tdsSensorIn);
      if (isnan(milliVolts)) return;
      calibrated = addCalibrationPoint(tdsCalibration, reference < tdsCalibrationSplit,
                                       reference, compensateTdsMilliVolts(milliVolts, temperature));
    } else {
      Serial.println("Unknown sensor or reference value out of range");
      return;
    }
    if (!calibrated) return;
    saveCalibrations();
  }

  printCalibration("pH", phCalibration);
  printCalibration("TDS", tdsCalibration);
}

// PUMP FUNCTIONS
bool empty(byte delaySeconds) {
  return runCountDown(
    delaySeconds,
    []() {
      Serial.println("Engaging output pump");
      engagePump(outputPump, true);
    },
    []() {
      Serial.println("Stopping output pump");
      engagePump(outputPump, false);
    });
}

bool fill(byte delaySeconds) {
  return runCountDown(
    delaySeconds,
    []() {
      Serial.println("Engaging input pump");
      engagePump(inputPump, true);
    },
    []() {
      Serial.println("Stopping input pump");
      engagePump(inputPump, false);
    });
}

// Circulates and keeps measuring TDS and pH. Readings taken while the pumps are running are only indicative.
bool circulate(byte delaySeconds) {
  bool done = runCountDown(
    delaySeconds,
    []() {
      Serial.println("Starting circulation");
      engagePump(inputPump, true);
      engagePump(outputPump, true);
    },
    []() {
      Serial.println("Stopping circulation");
      engagePump(inputPump, false);
      engagePump(outputPump, false);
      stopMeasuring();
    });

  if (!done) measureAll();
  return done;
}

// Waits for the solution to calm down after circulation
bool settle(byte delaySeconds) {
  return runCountDown(
    delaySeconds,
    []() {
      Serial.println("Letting the solution settle");
    },
    []() {});
}

bool pumpPh(byte durationSeconds) {
  return runCountDown(
    durationSeconds,
    []() {
      Serial.println("Engaging acid pump");
      engagePump(acidPump, Forward);
    },
    []() {
      Serial.println("Stopping acid pump");
      engagePump(acidPump, Stopped);
    });
}

bool pumpNutrientA(byte durationSeconds) {
  return runCountDown(
    durationSeconds,
    []() {
      Serial.println("Engaging TDS A pump");
      engagePump(nutrientAPump, Forward);
    },
    []() {
      Serial.println("Stopping TDS A pump");
      engagePump(nutrientAPump, Stopped);
    });
}

bool pumpNutrientB(byte durationSeconds) {
  return runCountDown(
    durationSeconds,
    []() {
      Serial.println("Engaging TDS B pump");
      engagePump(nutrientBPump, Forward);
    },
    []() {
      Serial.println("Stopping TDS B pump");
      engagePump(nutrientBPump, Stopped);
    });
}

// ADJUSTMENT
const byte maxAdjustmentRounds = 6;
const byte circulateSeconds = 40;
const byte settleSeconds = 15;
const byte acidPumpSeconds = 2;
const byte nutrientPumpSeconds = 1;

void resetAdjustments() {
  phState = PH_MEASURE;
  phAdjustmentRound = 0;
  tdsState = TDS_MEASURE;
  tdsAdjustmentRound = 0;
}

// Only lowers the pH, as there is no pump for raising it
Progress phAdjustment(float target) {
  static const float phThreshold = 0.1f;

  switch (phState) {
    case PH_MEASURE:
      {
        float pH;
        if (!measureSensor(PH_SENSOR, pH)) break;
        powerSensor(NO_SENSOR);

        Serial.print("pH adjustment round: ");
        Serial.println(phAdjustmentRound + 1);
        Serial.print("Target: ");
        Serial.println(target);
        reportPh(pH);
        Serial.println();

        if (isnan(pH)) {
          Serial.println("Could not read pH, aborting adjustment");
          return FAILED;
        }
        if (pH <= target + phThreshold) {
          Serial.println("pH target reached");
          return FINISHED;
        }
        if (phAdjustmentRound >= maxAdjustmentRounds) {
          Serial.println("pH target not reached within the maximum number of rounds, giving up");
          return FINISHED;
        }
        phAdjustmentRound++;
        phState = PH_PUMP;
        break;
      }
    case PH_PUMP:
      {
        if (pumpPh(acidPumpSeconds)) phState = PH_CIRCULATE;
        break;
      }
    case PH_CIRCULATE:
      {
        if (circulate(circulateSeconds)) phState = PH_SETTLE;
        break;
      }
    case PH_SETTLE:
      {
        if (settle(settleSeconds)) phState = PH_MEASURE;
        break;
      }
  }

  return IN_PROGRESS;
}

// Only raises the TDS, as there is no pump for diluting the solution
Progress tdsAdjustment(float target) {
  static const float tdsThreshold = 20;

  switch (tdsState) {
    case TDS_MEASURE:
      {
        float TDS;
        if (!measureSensor(TDS_SENSOR, TDS)) break;
        powerSensor(NO_SENSOR);

        Serial.print("TDS adjustment round: ");
        Serial.println(tdsAdjustmentRound + 1);
        Serial.print("Target: ");
        Serial.println(target);
        reportTds(TDS);
        Serial.println();

        if (isnan(TDS)) {
          Serial.println("Could not read TDS, aborting adjustment");
          return FAILED;
        }
        if (TDS >= target - tdsThreshold) {
          Serial.println("TDS target reached");
          return FINISHED;
        }
        if (tdsAdjustmentRound >= maxAdjustmentRounds) {
          Serial.println("TDS target not reached within the maximum number of rounds, giving up");
          return FINISHED;
        }
        tdsAdjustmentRound++;
        tdsState = TDS_A_PUMP;
        break;
      }
    case TDS_A_PUMP:
      {
        if (pumpNutrientA(nutrientPumpSeconds)) tdsState = TDS_B_PUMP;
        break;
      }
    case TDS_B_PUMP:
      {
        if (pumpNutrientB(nutrientPumpSeconds)) tdsState = TDS_CIRCULATE;
        break;
      }
    case TDS_CIRCULATE:
      {
        if (circulate(circulateSeconds)) tdsState = TDS_SETTLE;
        break;
      }
    case TDS_SETTLE:
      {
        if (settle(settleSeconds)) tdsState = TDS_MEASURE;
        break;
      }
  }

  return IN_PROGRESS;
}

// MENU HANDLERS
void fill() {
  commandFromMenu = "f";
  Serial.println("Fill");
}

void empty() {
  commandFromMenu = "e";
  Serial.println("Empty");
}

void circulate() {
  commandFromMenu = "c";
  Serial.println("Circulate");
}

void adjustAll() {
  commandFromMenu = "a";
  Serial.println("adjust all");
}

void adjustPh() {
  commandFromMenu = "p";
  Serial.println("adjust ph");
}

void adjustTds() {
  commandFromMenu = "t";
  Serial.println("adjust ec");
}

void pumpA() {
  commandFromMenu = "n a";
  Serial.println("pump a");
}

void pumpB() {
  commandFromMenu = "n b";
  Serial.println("pump b");
}

void pumpAcid() {
  commandFromMenu = "u";
  Serial.println("pump acid");
}

void measure() {
  commandFromMenu = "m";
  Serial.println("measure");
}

void abortFromMenu() {
  commandFromMenu = "!";
  Serial.println("stop");
}

// SETUP
void setupMenu() {
  Serial.println("setup menu");

  // Create GEMAppearance object with general values (that will be used for every menu page if not overridden)
  GEMAppearance appearanceGeneral;
  //appearanceGeneral.menuPointerType = GEM_POINTER_ROW;
  //appearanceGeneral.menuItemsPerScreen = GEM_ITEMS_COUNT_AUTO;
  appearanceGeneral.menuItemsPerScreen = 6;
  appearanceGeneral.menuItemHeight = 20;
  appearanceGeneral.menuPageScreenTopOffset = 20;
  appearanceGeneral.menuValuesLeftOffset = 150;
  menu.setAppearance(appearanceGeneral);  // Note there is no `&` operator when setting general (or global) appearance of the menu

  // Add menu items to menu page
  pHValue.setPrecision(2);
  TDSValue.setPrecision(2);
  menuPageMain.addMenuItem(pHValue);
  menuPageMain.addMenuItem(TDSValue);
  menuPageMain.addMenuItem(measureItem);
  menuPageMain.addMenuItem(menuItemStop);

  menuPageMain.addMenuItem(menuItemPump);

  menuPagePump.addMenuItem(menuItemReservoir);
  menuPageReservoir.addMenuItem(menuItemCirculate);
  menuPageReservoir.addMenuItem(menuItemFill);
  menuPageReservoir.addMenuItem(menuItemEmpty);

  menuPagePump.addMenuItem(menuItemNutrients);
  menuPageNutrients.addMenuItem(menuItemPumpA);
  menuPageNutrients.addMenuItem(menuItemPumpB);
  menuPageNutrients.addMenuItem(menuItemPumpAcid);

  menuPageMain.addMenuItem(menuItemAdjust);
  menuPageAdjust.addMenuItem(menuItemTarget);
  menuPageAdjust.addMenuItem(menuItemAdjustAll);
  menuPageAdjust.addMenuItem(menuItemAdjustPh);
  menuPageAdjust.addMenuItem(menuItemAdjustTds);

  menuItemTargetPh.setPrecision(2);
  menuItemTargetTds.setPrecision(2);
  menuPageTarget.addMenuItem(menuItemTargetPh);
  menuPageTarget.addMenuItem(menuItemTargetTds);

  menuItemCurrentPh.setPrecision(2);
  menuItemCurrentTds.setPrecision(2);
  menuPageAdjusting.addMenuItem(menuItemTargetPh);
  menuPageAdjusting.addMenuItem(menuItemTargetTds);
  menuPageAdjusting.addMenuItem(menuItemCurrentPh);
  menuPageAdjusting.addMenuItem(menuItemCurrentTds);

  // Add menu page to menu and set it as current
  menu.setMenuPageCurrent(menuPageMain);

  menu.setTextSize(2);

  menu.invertKeysDuringEdit();
  Serial.println("setup menu done");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial.println("setup");

  pinMode(shiftRegisterReset, OUTPUT);
  pinMode(shiftRegisterLatch, OUTPUT);
  pinMode(shiftRegisterData, OUTPUT);
  pinMode(shiftRegisterClock, OUTPUT);
  digitalWrite(shiftRegisterReset, HIGH);
  setShiftRegisterValue(0);

  ph4502c.init();
  // Peristaltic pumps connected to shift out register
  // TODO: Verify pump assignment
  acidPump.enablePin = PPUMP1_EN;
  nutrientAPump.enablePin = PPUMP4_EN;
  nutrientBPump.enablePin = PPUMP3_EN;

  acidPump.directionAPin = nutrientAPump.directionAPin = nutrientBPump.directionAPin = PPUMP_A;
  acidPump.directionBPin = nutrientAPump.directionBPin = nutrientBPump.directionBPin = PPUMP_B;

  // TODO: ensure the float switches are assigned to the correct pumps
  inputPump = { "Input pump", 26, mixerFloatSensor, false, false, 0 };
  outputPump = { "Output pump", 27, reservoirFloatSensor, false, false, 0 };

  pinMode(mixerFloatSensor, INPUT);
  pinMode(reservoirFloatSensor, INPUT);
  pinMode(inputPump.relayPin, OUTPUT);
  pinMode(outputPump.relayPin, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  pinMode(ABORT_SW, INPUT_PULLUP);
  stopAllPumps();

  const byte ADC_EN = 14;
  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);
  analogReadResolution(12);
  analogSetPinAttenuation(phSensorIn, ADC_11db);
  analogSetPinAttenuation(temperatureSensorIn, ADC_11db);
  analogSetPinAttenuation(tdsSensorIn, ADC_11db);
  loadCalibrations();

  pinMode(rightPin, INPUT_PULLUP);
  pinMode(TFT_BL, OUTPUT);     // TTGO T-Display enable Backlight pin 4
  digitalWrite(TFT_BL, HIGH);  // T-Display turn on Backlight
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);  // Must be before tft.init(), which would otherwise use the default SPI pins
  tft.init(135, 240);          // Initialize ST7789 240x135
  tft.setSPISpeed(TFT_SPI_FREQUENCY);
  tft.setRotation(3);
  menu.init();
  setupMenu();
  menu.drawMenu();

  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 1000, true);

  Serial.println("setup done");
}

// COMMANDS

// Stops everything and resets all operation state so that the next operation starts from the beginning
void abortOperation() {
  stopAllPumps();
  cancelCountDown();
  stopMeasuring();
  resetAdjustments();
  state = IDLE;
}

void startOperation(State newState) {
  cancelCountDown();
  stopMeasuring();
  resetAdjustments();
  state = newState;
}

bool parsePhTarget(const String &args) {
  float value = targetPh;
  if (args.length() > 0 && !parseNumber(args, value)) {
    Serial.println("Invalid pH target");
    return false;
  }
  // Pumping acid towards a nonsensical target would ruin the solution
  if (value < 4 || value > 8) {
    Serial.println("pH target must be between 4 and 8");
    return false;
  }
  targetPh = value;
  return true;
}

bool parseTdsTarget(const String &args) {
  float value = targetTds;
  if (args.length() > 0 && !parseNumber(args, value)) {
    Serial.println("Invalid TDS target");
    return false;
  }
  if (value <= 0 || value > 3000) {
    Serial.println("TDS target must be between 0 and 3000");
    return false;
  }
  targetTds = value;
  return true;
}

void handleCommand(String incoming) {
  incoming.trim();
  if (incoming.length() == 0) return;

  const char command = incoming[0];
  String args = incoming.substring(1);
  args.trim();
  Serial.print("Received command: ");
  Serial.println(incoming);

  if (state != IDLE && command != '!' && command != '?') {
    Serial.println("Operation in progress. Cannot run command. Please wait for previous command to finish or abort with '!'");
    return;
  }

  switch (command) {
    case '1':
      {
        powerSensor(PH_SENSOR);
        delay(sensorWarmUpMilliseconds);
        float v = ph4502c.read_ph_level();
        powerSensor(NO_SENSOR);
        Serial.print("p45:");
        Serial.println(v);
        break;
      }
    case '?':
      {
        printUsage();
        break;
      }
    case 't':
      {
        if (!parseTdsTarget(args)) break;
        Serial.print("Starting TDS adjustment with target: ");
        Serial.println(targetTds);
        startOperation(ADJUST_TDS);
        break;
      }
    case 'p':
      {
        if (!parsePhTarget(args)) break;
        Serial.print("Starting pH adjustment with target: ");
        Serial.println(targetPh);
        startOperation(ADJUST_PH);
        break;
      }
    case 'a':
      {
        const int splitLocation = args.indexOf(' ');
        if (args.length() > 0 && splitLocation < 0) {
          Serial.println("Usage: a 1000 6.0");
          break;
        }
        const String tdsArgs = args.length() > 0 ? args.substring(0, splitLocation) : "";
        String phArgs = args.length() > 0 ? args.substring(splitLocation + 1) : "";
        phArgs.trim();
        // Validate both before changing either target
        const float previousTdsTarget = targetTds;
        if (!parseTdsTarget(tdsArgs)) break;
        if (!parsePhTarget(phArgs)) {
          targetTds = previousTdsTarget;
          break;
        }
        Serial.print("Starting TDS adjustment with target: ");
        Serial.print(targetTds);
        Serial.print(" followed by pH adjustment with target: ");
        Serial.println(targetPh);
        startOperation(ADJUST_ALL);
        break;
      }
    case 'e':
      {
        startOperation(EMPTY);
        break;
      }
    case 'f':
      {
        startOperation(FILL);
        break;
      }
    case 'c':
      {
        startOperation(CIRCULATE);
        break;
      }
    case 'n':
      {
        args.toLowerCase();
        if (args == "b") {
          startOperation(PUMP_NUTRIENT_B);
        } else if (args == "a" || args.length() == 0) {
          pumpBothNutrients = args.length() == 0;
          startOperation(PUMP_NUTRIENT_A);
        } else {
          Serial.println("Usage: n / n a / n b");
        }
        break;
      }
    case 'u':
      {
        startOperation(PUMP_PH);
        break;
      }
    case 'm':
      {
        startOperation(MEASURE);
        break;
      }
    case 'l':
      {
        float seconds = 120;
        if (args.length() > 0 && (!parseNumber(args, seconds) || seconds < 1 || seconds > 86400)) {
          Serial.println("Plot duration must be between 1 and 86400 seconds");
          break;
        }
        plotDurationSeconds = seconds;
        startOperation(PLOT);
        break;
      }
    case 'k':
      {
        calibrate(args);
        break;
      }
    case '!':
      {
        Serial.println("Abort!");
        abortOperation();
        break;
      }
    default:
      {
        Serial.println("Unknown command");
        break;
      }
  }
}

// Returns true once the abort switch has been held down for a moment (filters out noise)
bool abortSwitchPressed() {
  static const unsigned long debounceMilliseconds = 50;
  static unsigned long releasedAt = 0;
  static bool reported = false;

  if (digitalRead(ABORT_SW)) {
    releasedAt = millis();
    reported = false;
    return false;
  }
  if (!reported && millis() - releasedAt >= debounceMilliseconds) {
    reported = true;
    return true;
  }
  return false;
}

// LOOP
void loop() {
  // If menu is ready to accept button press...
  if (menu.readyForKey()) {
    // ...detect key press using KeyDetector library
    myKeyDetector.detect();
    // Pass pressed button to menu
    // (pressed button ID is stored in trigger property of KeyDetector object)
    menu.registerKeyPress(myKeyDetector.trigger);
  }

  rotary_loop();

  updatePumps();

  static const byte emptyDelaySeconds = 10;
  static const byte fillDelaySeconds = 10;
  static const byte circulateDelaySeconds = 10;

  if (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  if (commandFromMenu != nullptr) {
    const char *command = commandFromMenu;
    commandFromMenu = nullptr;
    handleCommand(command);
  }

  if (abortSwitchPressed()) {
    Serial.println("Abort switch pressed");
    abortOperation();
  }

  static State previousState = IDLE;
  if (previousState != state) {
    previousState = state;
    Serial.print("State changed to: ");
    Serial.println(state);
  }

  switch (state) {
    case CIRCULATE:
      {
        if (circulate(circulateDelaySeconds)) state = IDLE;
        break;
      }
    case EMPTY:
      {
        if (empty(emptyDelaySeconds)) state = IDLE;
        break;
      }
    case FILL:
      {
        if (fill(fillDelaySeconds)) state = IDLE;
        break;
      }
    case MEASURE:
      {
        if (measureAll()) {
          powerSensor(NO_SENSOR);
          state = IDLE;
        }
        break;
      }
    case PLOT:
      {
        if (runCountDown(plotDurationSeconds, []() {}, []() {})) {
          Serial.println("Stopping plotting");
          stopMeasuring();
          state = IDLE;
          break;
        }
        measureAll();
        break;
      }
    case ADJUST_PH:
      {
        Progress progress = phAdjustment(targetPh);
        if (progress != IN_PROGRESS) {
          Serial.println(progress == FINISHED ? "pH adjustment done" : "pH adjustment failed");
          abortOperation();
        }
        break;
      }
    case ADJUST_TDS:
    case ADJUST_ALL:
      {
        Progress progress = tdsAdjustment(targetTds);
        if (progress == IN_PROGRESS) break;

        Serial.println(progress == FINISHED ? "TDS adjustment done" : "TDS adjustment failed");
        if (state == ADJUST_ALL && progress == FINISHED) {
          Serial.println("Starting pH adjustment");
          startOperation(ADJUST_PH);
        } else {
          abortOperation();
        }
        break;
      }
    case PUMP_PH:
      {
        if (pumpPh(acidPumpSeconds)) state = CIRCULATE;
        break;
      }
    case PUMP_NUTRIENT_A:
      {
        if (pumpNutrientA(nutrientPumpSeconds)) state = pumpBothNutrients ? PUMP_NUTRIENT_B : CIRCULATE;
        break;
      }
    case PUMP_NUTRIENT_B:
      {
        if (pumpNutrientB(nutrientPumpSeconds)) state = CIRCULATE;
        break;
      }
    case IDLE:
    default:
      {
        break;
      }
  }
}
