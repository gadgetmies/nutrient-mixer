
#define DEBUG
// Use 115200 speed for programming
// Perhaps necessary to use v2.x of ESP32 boards https://dl.espressif.com/dl/package_esp32_index.json

// TODO: The 5V on T-Display gives out just 3V for some reason
//.      GPIO pins 38 and 39 do not seem to go up

#include <KeyDetector.h>
#include <AiEsp32RotaryEncoder.h>
#include <Adafruit_ST7789.h>
#include <GEM_adafruit_gfx.h>
#include <CountDown.h>
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

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);


const byte phSensorIn = 36;
const byte temperatureSensorIn = 39;
const byte tdsSensorIn = 32;

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

const byte PPUMP1_EN = 1;
const byte PPUMP2_EN = 2;
const byte PPUMP4_EN = 3;
const byte PPUMP_B = 4;
const byte PPUMP_A = 5;
const byte PPUMP3_EN = 6;

const byte TDS_ON = 7;
const byte PH_ON = 0;

// GLOBAL STATE
float targetTds = 2;
float targetPh = 6;
float currentPh = 7;
float currentTds = 0;
char currentPhString[17] = "-\0";
char currentTdsString[17] = "-\0";
bool enablePrint = false;
char commandFromMenu = 0;

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

struct Pump {
  byte relayPin;
};

enum PeristalticPumps {
  ACID,
  NUTRIENT_A,
  NUTRIENT_B
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
  PUMP_NUTRIENT,
  ABORT
};

enum phAdjustState {
  PH_MEASURE,
  PH_PUMP,
  PH_CIRCULATE
};

enum tdsAdjustState {
  TDS_MEASURE,
  TDS_A_PUMP,
  TDS_B_PUMP,
  TDS_CIRCULATE
};

PeristalticPump acidPump;
PeristalticPump nutrientAPump;
PeristalticPump nutrientBPump;

Pump inputPump;
Pump outputPump;

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

// UTILITY FUNCTIONS
void (*reset)(void) = 0;

float ainToVoltage(long value) {
  // Measured a couple of values and created the linear regression equation in Numbers (scatter plot, series->trendlines)
  return value * 0.0008 + 0.1625;
}

float voltageToPH(float voltage) {
  return -5.69 * voltage + 21.13;
}

float valueTopH(int value) {
  return -0.0068 * value + 22.626;
}

float voltageToTDS(float voltage) {
  return 395 * voltage - 39.7;
}

float valueToTDS(float value) {
  return 0.3207 * value + 116.685;
}

bool runCountDown(CountDown& countDown, bool initialise, byte delaySeconds, void (*start)(), bool (*during)(), void (*stop)()) {
  if (initialise) {
    countDown.start(delaySeconds * 1000);
    start();
  } else if (countDown.remaining() == 0) {
    stop();
    return true;
  } else {
    return during();
  }
  return false;
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
  Serial.println("t 1000 # Adjust TDS to 1000");
  Serial.println("p 7.0 # Adjust pH to 7.0");
  Serial.println("a 1000 7.0 # Adjust TDS to 1000 and pH to 7.0");
  Serial.println("e # Empty tank");
  Serial.println("f # Fill tank");
  Serial.println("c # Circulate");
  Serial.println("u # Pump pH");
  Serial.println("n # Pump nutrients");
  Serial.println("l 10 # Plot pH, TDS and temperature values for 10 seconds");
  Serial.println("! # Abort current operation (stop all pumps) and go to idle");
}

// MENU SETUP
GEMPage menuPageMain("Main Menu");

GEMItem pHValue("pH", currentPhString, true);
GEMItem TDSValue("TDS", currentTdsString, true);

GEMItem measureItem("Measure", measure);

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

  Serial.println();
  Serial.print("Setting shift register to: 0b");
  for (int i = 7; i >= 0; i--) {
    Serial.print(bitRead(registerState, i));
  }
  Serial.println();
}

void setShiftRegisterValue(byte pin, bool state) {
  setShiftRegisterValue(registerState & ~(1 << pin) | ((byte)state << pin));
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

void engagePump(Pump pump, bool pumping) {
  // MEASUREMENTS
  Serial.print("Engage pump in GPIO: ");
  Serial.println(pump.relayPin);
  Serial.print("Set pumping to: ");
  Serial.println(pumping);
  pinMode(pump.relayPin, OUTPUT);
  digitalWrite(pump.relayPin, pumping ? HIGH : LOW);
  Serial.println("pump engaged");
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

float voltageToTemperature(float voltage) {
  return voltage;  // TODO
}

float measureTemperature() {
  static const byte measureSamples = 5;
  static const byte measureDelayMilliseconds = 100;

  float sum = 0;
  for (byte i = 0; i < measureSamples; ++i) {
    delay(measureDelayMilliseconds);
    long value = analogRead(temperatureSensorIn);
    float voltage = ainToVoltage(value);
    const float temperatureValue = voltage;  // TODO
#ifdef DEBUG
    if (i == 0) {
      Serial.print("Temperature_output_value:");
      Serial.print(value);
      Serial.print(",Temperature_output_voltage:");
      Serial.print(voltage);
      Serial.print(",");
    }
#endif

    sum += voltage;
  }

  return voltageToTemperature(sum / measureSamples);
}

void reportTemperature(float temperature) {
  Serial.print("Temperature:");
  Serial.print(temperature);
}

void setPhEnabled(bool state) {
  setShiftRegisterValue(PH_ON, state ? HIGH : LOW);
  if (state) {
    setShiftRegisterValue(TDS_ON, LOW);
  }
}

float measurePh(float temperature) {
  static const byte measureSamples = 5;
  static const byte measureDelayMilliseconds = 100;
  // TODO: add delay with state machine delay(1000);

  float sum = 0;
  for (byte i = 0; i < measureSamples; ++i) {
    // TODO: remove delay
    delay(measureDelayMilliseconds);
    long value = analogRead(phSensorIn);
    float voltage = ainToVoltage(value);
#ifdef DEBUG
    if (i == 0) {
      Serial.print(",pH_value:");
      Serial.print(value);
      Serial.print(",pH_voltage:");
      Serial.print(voltage);
      Serial.print(",");
    }
#endif
    sum += value;
  }

  // pH 3.00 = 2.29
  // pH 7.00 = 1.75
  return valueTopH(sum / measureSamples);
}

void reportPh(float value) {
  Serial.print("pH:");
  Serial.print(value);
  snprintf(currentPhString, sizeof(currentPhString), "%f", value);
}

void reportTds(float value) {
  Serial.print("TDS:");
  Serial.print(value);
  snprintf(currentTdsString, sizeof(currentTdsString), "%f", value);
}

void setTdsEnabled(bool state) {
  setShiftRegisterValue(TDS_ON, state ? HIGH : LOW);
  if (state) {
    setShiftRegisterValue(PH_ON, LOW);
  }
}

float measureTds() {
  static const byte tdsMeasureSamples = 5;

  float sum = 0;
  for (byte i = 0; i < tdsMeasureSamples; ++i) {
    float temperature = 20;  // TODO: read temperature from sensor
    long value = analogRead(tdsSensorIn);
    float voltage = ainToVoltage(value);
#ifdef DEBUG
    if (i == 0) {
      Serial.print("TDS_output_value:");
      Serial.print(value);
      Serial.print(",TDS_output_voltage:");
      Serial.print(voltage);
      Serial.print(",");
    }
#endif
    sum += value;
  }

  return valueToTDS(sum / tdsMeasureSamples);
}

// PUMP FUNCTIONS
bool empty(byte delaySeconds) {
  static CountDown countDown;
  static bool started = false;

  return runCountDown(
    countDown,
    delaySeconds,
    started,
    []() {
      Serial.println("Engaging output pump");
      engagePump(outputPump, true);
      started = true;
    },
    []() -> bool {
      Serial.println("Pumping");
      return false;
    },
    []() {
      Serial.println("Stopping output pump");
      engagePump(outputPump, false);
      started = false;
    });
}

bool fill(byte delaySeconds) {
  static CountDown countDown;
  static bool started = false;

  return runCountDown(
    countDown,
    started,
    delaySeconds,
    []() {
      Serial.println("Engaging input pump");
      engagePump(inputPump, true);
      started = true;
    },
    []() -> bool {
      Serial.println("Pumping");
      return false;
    },
    []() {
      Serial.println("Stopping input pump");
      engagePump(inputPump, false);
      started = false;
    });
}

enum CirculationState {
  CIRCULATION_STATE_MEASUREMENT_STOPPED,
  CIRCULATION_STATE_ENABLE_TDS,
  CIRCULATION_STATE_MEASURE_TDS,
  CIRCULATION_STATE_ENABLE_PH,
  CIRCULATION_STATE_MEASURE_PH,
};

bool circulate(byte delaySeconds) { 
  static CountDown countDown;
  static bool started = false;
  static CirculationState state = CIRCULATION_STATE_MEASUREMENT_STOPPED;

  return runCountDown(
    countDown,
    started,
    delaySeconds, 
    []() {
      Serial.println("Starting circulation");
      engagePump(inputPump, true);
      engagePump(outputPump, true);
      started = true;
    },
    []() -> bool {
      Serial.println("Circulating and measuring");

      // TODO: Generalise measurement as it is needed in multiple places
      int circulationCounter = countDown.remaining() % 3000;
      if (circulationCounter < 1000 && state == CIRCULATION_STATE_MEASUREMENT_STOPPED) {
        setTdsEnabled(true);
        state = CIRCULATION_STATE_ENABLE_TDS;
      } else if (circulationCounter >= 1000 && state == CIRCULATION_STATE_ENABLE_TDS) {
        float TDS = measureTds();
        reportTds(TDS);
        Serial.print(",");
        setTdsEnabled(false);
        state = CIRCULATION_STATE_MEASURE_TDS;
      } else if (state == CIRCULATION_STATE_MEASURE_TDS) {
        setPhEnabled(true);
        state = CIRCULATION_STATE_ENABLE_PH;
      } else if (circulationCounter >= 2000 && state == CIRCULATION_STATE_ENABLE_PH) {
        float pH = measurePh(20);
        reportPh(pH);
        Serial.println();
        state = CIRCULATION_STATE_MEASUREMENT_STOPPED;
      }
      
      return false;
    },
    []() {
      Serial.println("Stopping circulation");
      engagePump(inputPump, false);
      engagePump(outputPump, false);
      started = false;
    });
}

bool pumpPh(byte durationSeconds) {
  static CountDown countDown;
  static bool started = false;

  return runCountDown(
    countDown,
    started,
    durationSeconds,
    []() {
      Serial.println("Engaging acid pump");
      engagePump(acidPump, Forward);
      started = true;
    },
    []() -> bool {
      Serial.println("Acid pump engaged");
      return false;
    },
    []() {
      Serial.println("Stopping acid pump");
      engagePump(acidPump, Stopped);
      started = false;
    });
}

bool pumpNutrients(tdsAdjustState &state, byte delaySeconds) {
  static CountDown countDown;
  static bool started = false;

  switch (state) {
    case TDS_A_PUMP:
      {
        bool done = runCountDown(
          countDown,
          started,
          delaySeconds,
          []() {
            Serial.println("Engaging TDS A pump");
            engagePump(nutrientAPump, Forward);
            started = true;
          },
          []() -> bool {
            Serial.println("TDS A pumping");
            return false;
          },
          []() {
            Serial.println("Stopping TDS A pump");
            engagePump(nutrientAPump, Stopped);
            started = false;
          });

        if (done) {
          Serial.println("Nutrient A pumped, next B");
          state = TDS_B_PUMP;
        }
        break;
      }
    case TDS_B_PUMP:
      {
        bool done = runCountDown(
          countDown,
          started,
          delaySeconds,
          []() {
            Serial.println("Engaging TDS B pump");
            engagePump(nutrientBPump, Forward);
            started = true;
          },
          []() -> bool {
            Serial.println("TDS B pumping");
            return false;
          },
          []() {
            Serial.println("Stopping TDS B pump");
            engagePump(nutrientBPump, Stopped);
            started = false;
          });
        if (done) {
          Serial.println("Nutrient B pumped, next circulate");
          return true;
        }
        break;
      }
  }
  return false;
}

// ADJUSTMENT
bool phAdjustment(float target) {
  static const byte phMeasureSamples = 5;
  static const byte phCirculateSeconds = 40;
  static const byte acidPumpDelaySeconds = 2;
  static const float phThreshold = 0.1f;
  static phAdjustState state = PH_MEASURE;
  static byte adjustmentRound = 0;
  float diff = 0;

  setPhEnabled(true);

  switch (state) {
    case PH_MEASURE:
      {
        Serial.print("Adjustment round: ");
        Serial.println(adjustmentRound);
        Serial.print("Target: ");
        Serial.println(target);
        float temperature = measureTemperature();
        float pH = measurePh(20);  // temperature);
        reportPh(pH);
        Serial.println();

        diff = abs(pH - target);
        if (pH < target || diff < phThreshold || adjustmentRound > 5) {
          adjustmentRound = 0;
          return true;
        } else {
          state = PH_PUMP;
          adjustmentRound++;
        }
        break;
      }
    case PH_PUMP:
      {
        bool done = pumpPh(round(acidPumpDelaySeconds));  // * diff));
        if (done) state = PH_CIRCULATE;
        break;
      }
    case PH_CIRCULATE:
      {
        bool done = circulate(phCirculateSeconds);
        if (done) state = PH_MEASURE;
        break;
      }
  }

  setPhEnabled(false);
  return false;
}

bool tdsAdjustment(float target = 0) {
  static const byte TDSCirculateSeconds = 40;
  static const byte TDSPumpDelaySeconds = 1;
  static float TDSThreshold = 0.1f;
  static tdsAdjustState state = TDS_MEASURE;
  float diff = 0;
  static byte adjustmentRound = 0;

  setTdsEnabled(true);

  switch (state) {
    case TDS_MEASURE:
      {
        // TODO: wait for TDS to settle?
        Serial.print("Adjustment round: ");
        Serial.println(adjustmentRound + 1);
        Serial.print("Target: ");
        Serial.println(target);
        float TDS = measureTds();
        reportTds(TDS);
        Serial.println();
        diff = abs(TDS - target);
        if (target < 0.1 || TDS > target || diff < TDSThreshold || adjustmentRound > 5) {
          adjustmentRound = 0;
          return true;
        } else {
          state = TDS_A_PUMP;
          adjustmentRound++;
        }
        break;
      }
    case TDS_A_PUMP:
    case TDS_B_PUMP:
      {
        bool done = pumpNutrients(state, round(TDSPumpDelaySeconds));  // * diff));
        if (done) {
          state = (tdsAdjustState)((int)state + 1);
          Serial.print("Pump A: ");
          Serial.println(TDS_A_PUMP);
          Serial.print("Pump B: ");
          Serial.println(TDS_B_PUMP);
          Serial.print("Circulate: ");
          Serial.println(TDS_CIRCULATE);
          Serial.print("Next pump state: ");
          Serial.println(state);
        }
        break;
      }
    case TDS_CIRCULATE:
      {
        bool done = circulate(TDSCirculateSeconds);
        if (done) {
          state = TDS_MEASURE;
        }
        break;
      }
  }

  setTdsEnabled(false);

  return false;
}

// MENU HANDLERS
void fill() {
  commandFromMenu = 'f';
  Serial.println("Fill");
}

void empty() {
  commandFromMenu = 'e';
  Serial.println("Empty");
}

void circulate() {
  commandFromMenu = 'c';
  Serial.println("Circulate");
}

void adjustAll() {
  Serial.println("adjust all");
}

void adjustPh() {
  commandFromMenu = 'p';
  Serial.println("adjust ph");
}

void adjustTds() {
  Serial.println("adjust ec");
}

void pumpA() {
  Serial.println("pump a");
}

void pumpB() {
  Serial.println("pump b");
}

void pumpAcid() {
  Serial.println("pump acid");
}

void measure() {
  Serial.println("measure");
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
  Serial.println("setup");

  pinMode(shiftRegisterReset, OUTPUT);
  pinMode(shiftRegisterLatch, OUTPUT);
  pinMode(shiftRegisterData, OUTPUT);
  pinMode(shiftRegisterClock, OUTPUT);
  digitalWrite(shiftRegisterReset, HIGH);
  setShiftRegisterValue(0);

  ph4502c.init();
  // Peristaltic pumps connected to shift out register
  acidPump.enablePin = 1;
  // TODO: Verify pump assignment
  nutrientAPump.enablePin = 3;
  nutrientBPump.enablePin = 6;

  acidPump.directionAPin = nutrientAPump.directionAPin = nutrientBPump.directionAPin = 5;
  acidPump.directionBPin = nutrientAPump.directionBPin = nutrientBPump.directionBPin = 4;
  inputPump.relayPin = 26;
  outputPump.relayPin = 27;

  pinMode(inputPump.relayPin, OUTPUT);
  pinMode(outputPump.relayPin, OUTPUT);
  digitalWrite(inputPump.relayPin, HIGH);
  digitalWrite(outputPump.relayPin, HIGH);
  //pinMode(phSensorIn, INPUT);
  //pinMode(temperatureSensorIn, INPUT);
  //pinMode(tdsSensorIn, INPUT);
  const byte ADC_EN = 14;
  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);

  pinMode(rightPin, INPUT_PULLUP);
  tft.init(240, 135);
  pinMode(TFT_BL, OUTPUT);     // TTGO T-Display enable Backlight pin 4
  digitalWrite(TFT_BL, HIGH);  // T-Display turn on Backlight
  tft.init(135, 240);          // Initialize ST7789 240x135
  tft.setRotation(3);
  menu.init();
  setupMenu();
  menu.drawMenu();

  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 1000, true);

  Serial.println("setup done");
}

bool inputPumpStopped = false;
bool outputPumpStopped = false;

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

  byte reservoirFloatSwitchTriggered = !digitalRead(reservoirFloatSensor);
  byte mixerFloatSwitchTriggered = !digitalRead(mixerFloatSensor);

  // TODO: ensure input and output are assigned correctly
  if (digitalRead(outputPump.relayPin) && reservoirFloatSwitchTriggered) {
    outputPumpStopped = true;
    engagePump(outputPump, false);
    Serial.println("Reservoir overflowing, stopping output pump");
    digitalWrite(LED_ERROR, HIGH);
  } else if (outputPumpStopped && !reservoirFloatSwitchTriggered) {
    outputPumpStopped = false;
    engagePump(outputPump, true);
    Serial.println("Reservoir overflow cleared, starting output pump");
    digitalWrite(LED_ERROR, inputPumpStopped ? HIGH : LOW);
  }

  if (digitalRead(inputPump.relayPin) && mixerFloatSwitchTriggered) {
    inputPumpStopped = true;
    engagePump(inputPump, false);
    Serial.println("Reservoir overflowing, stopping input pump");
    digitalWrite(LED_ERROR, HIGH);
  } else if (inputPumpStopped && !mixerFloatSwitchTriggered) {
    inputPumpStopped = false;
    engagePump(inputPump, true);
    Serial.println("Mixer overflow cleared, starting input pump");
    digitalWrite(LED_ERROR, outputPumpStopped ? HIGH : LOW);
  }

  static const byte emptyDelaySeconds = 10;
  static const byte fillDelaySeconds = 10;
  static const byte circulateDelaySeconds = 10;
  static byte plotDurationSeconds = 120;
  static State state = IDLE;
  bool done = false;
  bool commandFromSerial = false;

  char command = 0;
  String args;

  if (Serial.available() > 0) {
    String incoming = Serial.readString();
    incoming.trim();
    command = incoming[0];
    args = incoming.substring(2, incoming.length());
    Serial.print("Received command: ");
    Serial.println(command);

    if (state != IDLE && command != '!') {
      Serial.println("Operation in progress. Cannot run command. Please wait for previous command to finish or abort with '!'");
    }

    commandFromSerial = true;
  }

  if (commandFromMenu != 0) {
    command = commandFromMenu;
    commandFromMenu = 0;
  }

  switch (command) {
    case 0:
      break;
    case '1':
      {
        float v = ph4502c.read_ph_level();
        Serial.print("p45:");
        Serial.println(v);
        /*
        Serial.print(",TDS:");
        Serial.print(analogRead(tdsSensorIn));
        Serial.print(',');
        Serial.print("Temp:");
        Serial.print(analogRead(temperatureSensorIn));
        Serial.print(',');
        Serial.print("pH:");
        Serial.println(analogRead(phSensorIn));
        */
        break;
      }
    case '?':
      {
        printUsage();
        break;
      }
    case 't':
      {
        if (commandFromSerial) {
          targetTds = args.toFloat();
        }
        Serial.print("Starting TDS adjustment with target: ");
        Serial.println(targetTds);
        state = ADJUST_TDS;
        break;
      }
    case 'p':
      {
        if (commandFromSerial) {
          targetPh = args.toFloat();
        }
        Serial.print("Starting pH adjustment with target: ");
        Serial.println(targetPh);
        state = ADJUST_PH;
        break;
      }
    case 'a':
      {
        state = ADJUST_ALL;
        if (commandFromSerial) {
          byte splitLocation = args.indexOf(' ');
          targetTds = args.substring(0, splitLocation).toFloat();
          targetPh = args.substring(splitLocation + 1).toFloat();
        }
        break;
      }
    case 'e':
      {
        state = EMPTY;
        break;
      }
    case 'f':
      {
        state = FILL;
        break;
      }
    case 'c':
      {
        state = CIRCULATE;
        break;
      }
    case 'n':
      {
        state = PUMP_NUTRIENT;
        break;
      }
    case 'u':
      {
        state = PUMP_PH;
        break;
      }
    case 'm':
      {
        state = MEASURE;
        break;
      }
    case 'l':
      {
        state = PLOT;
        plotDurationSeconds = args.toFloat();
        break;
      }
    default:
      {
        Serial.println("Unknown command");
        break;
      }
    case '!':
      {
        Serial.println("Abort!");
        state = ABORT;
        break;
      }
  }

  if (!digitalRead(ABORT_SW)) {
    //state = ABORT;
  }

  if (command != 0) {
    Serial.print("Command: ");
    Serial.println(command);
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
        done = circulate(circulateDelaySeconds);
        break;
      }
    case EMPTY:
      {
        done = empty(emptyDelaySeconds);
        break;
      }
    case FILL:
      {
        done = fill(fillDelaySeconds);
        break;
      }
    case MEASURE:
      {
        // TODO: enable sensors before measurement
        float TDS = measureTds();
        Serial.println();
        reportTds(TDS);
        Serial.println();
        float temperature = measureTemperature();
        Serial.println();
        reportTemperature(temperature);
        Serial.println();
        float pH = measurePh(20);  //temperature);
        Serial.println();
        reportPh(pH);
        Serial.println();
        state = IDLE;
        break;
      }
    case PLOT:
      {
        static CountDown countDown;
        static bool started = false;

        bool done = runCountDown(
          countDown,
          started,
          plotDurationSeconds,
          []() {
            started = true;
          },
          []() -> bool {},
          []() {
            started = false;
          });

        if (done) {
          Serial.println("Stopping plotting");
          state = IDLE;
        }

        // TODO: enable sensors first
        float TDS = measureTds();
        reportTds(TDS);
        Serial.print(",");
        float temperature = measureTemperature();
        reportTemperature(temperature);
        Serial.print(",");
        float pH = measurePh(20);  //temperature);
        reportPh(pH);
        Serial.println();
        break;
      }
    case ADJUST_PH:
      {
        done = phAdjustment(targetPh);
        if (done) {
          Serial.println("pH adjustment done");
        }
        break;
      }
    case ADJUST_TDS:
    case ADJUST_ALL:
      {
        done = tdsAdjustment(targetTds);
        if (done) {
          Serial.println("TDS adjustment done");
          if (state == ADJUST_ALL) {
            Serial.println("Starting pH adjustment");
            state = ADJUST_PH;
            done = false;
          }
        }
        break;
      }
    case PUMP_PH:
      {
        bool done = pumpPh(2);
        if (done) state = CIRCULATE;
        break;
      }
    case PUMP_NUTRIENT:
      {
        bool done = tdsAdjustment();
        if (done) {
          Serial.println("TDS at (or over) target. Circulating.");
          state = CIRCULATE;
        }
        break;
      }

    case ABORT:
      {
        engagePump(inputPump, false);
        engagePump(outputPump, false);
        engagePump(nutrientAPump, Stopped);
        engagePump(nutrientBPump, Stopped);
        engagePump(acidPump, Stopped);
        state = IDLE;
        reset();
        break;
      }
    case IDLE:
    default:
      {
        break;
      }
  }

  if (done) state = IDLE;
}
