// Use 115200 speed for programming
// Perhaps necessary to use v2.x of ESP32 boards https://dl.espressif.com/dl/package_esp32_index.json

// TODO: The 5V on T-Display gives out just 3V for some reason
//.      GPIO pins 38 and 39 do not seem to go up

#include <KeyDetector.h>
#include <AiEsp32RotaryEncoder.h>
#include <Adafruit_ST7789.h>
#include <GEM_adafruit_gfx.h>
#include <CountDown.h>

// BUTTON HANDLING
const byte rightPin = 0;
Key keys[] = {{GEM_KEY_RIGHT, rightPin}};
KeyDetector myKeyDetector(keys, sizeof(keys)/sizeof(Key));

// TFT INITIALISATION
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 23
#define TFT_BL 4

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// GLOBAL STATE
float targetTDS = 2;
float targetpH = 6;
float currentpH = 7;
float currentTDS = 0;
char currentpHString[17] = "-\0";
char currentTDSString[17] = "-\0";
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

enum pHadjustState {
  PH_MEASURE,
  PH_PUMP,
  PH_CIRCULATE
};

enum TDSadjustState {
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

const byte pHSensorIn = 27;
const byte temperatureSensorIn = 26;
const byte TDSSensorIn = 32;

// FORWARD DECLARATIONS
void fill();
void empty();
void circulate();
void adjustAll();
void adjustpH();
void adjustTDS();
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

bool runCountDown(byte delaySeconds, void (*start)(), void (*stop)()) {
  static CountDown countDown;
  static bool running = false;
  if (!running) {
    running = true;
    countDown.start(delaySeconds * 1000);
    start();
  } else if (countDown.remaining() == 0) {
    running = false;
    stop();
    return true;
  }
  return false;
}

void printData() {
  Serial.println("print data");
  // If enablePrint flag is set to true (checkbox on screen is checked)...
  if (enablePrint) {
    // ...print the number to Serial
    Serial.print("TDS target is: ");
    Serial.println(targetTDS);

    Serial.print("pH target is: ");
    Serial.println(targetpH);
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

GEMItem pHValue("pH", currentpHString, true);
GEMItem TDSValue("TDS", currentTDSString, true);

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

GEMItem menuItemTargetpH("Target pH:", targetpH);
GEMItem menuItemTargetTDS("Target TDS:", targetTDS);

GEMItem menuItemAdjustAll("Adjust all", adjustAll);
GEMItem menuItemAdjustpH("Adjust pH", adjustpH);
GEMItem menuItemAdjustTDS("Adjust TDS", adjustTDS);

GEM_adafruit_gfx menu(tft, GEM_POINTER_ROW, GEM_ITEMS_COUNT_AUTO);

// ROTARY ENCODER SETUP
#define ROTARY_ENCODER_A_PIN 17
#define ROTARY_ENCODER_B_PIN 22
#define ROTARY_ENCODER_BUTTON_PIN 21

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, 4, false);

void rotary_onButtonClick() {
	static unsigned long lastTimePressed = 0;
	//ignore multiple press in that time milliseconds
	if (millis() - lastTimePressed < 500)
	{
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
	if (changed)
	{
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
	if (rotaryEncoder.isEncoderButtonClicked())
	{
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
}

void engagePump(PeristalticPump pump, Rotation rotation) {
  if (rotation == Stopped) {
    digitalWrite(pump.enablePin, LOW);
  } else {
    if (rotation == Forward) {
      digitalWrite(pump.directionAPin, HIGH);
      digitalWrite(pump.directionBPin, LOW);
    } else {
      digitalWrite(pump.directionAPin, HIGH);
      digitalWrite(pump.directionBPin, LOW);
    }
    digitalWrite(pump.enablePin, HIGH);
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

float measurepH(float temperature) {
  static const byte measureSamples = 5;
  static const byte measureDelayMilliseconds = 100;

  float sum = 0;
  for (byte i = 0; i < measureSamples; ++i) {
    delay(measureDelayMilliseconds);
    long value = analogRead(pHSensorIn);
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

void reportpH(float value) {
  Serial.print("pH:");
  Serial.print(value);
}

void reportTDS(float value) {
  Serial.print("TDS:");
  Serial.print(value);
}

float measureTDS() {
  static const byte TDSMeasureSamples = 5;

  float sum = 0;
  for (byte i = 0; i < TDSMeasureSamples; ++i) {
    float temperature = 20;  // TODO: read temperature from sensor
    long value = analogRead(TDSSensorIn);
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

  return valueToTDS(sum / TDSMeasureSamples);
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

bool circulate(byte delaySeconds) {
  static long counter = 0;

  if (counter % 10 == 0) {
    float TDS = measureTDS();
    reportTDS(TDS);
    Serial.print(",");
    float pH = measurepH(20);
    reportpH(pH);
    Serial.println();
  }
  counter++;

  return runCountDown(
    delaySeconds, []() {
      counter = 0;
      Serial.println("Starting circulation");
      engagePump(inputPump, true);
      engagePump(outputPump, true);
    },
    []() {
      Serial.println("Stopping circulation");
      engagePump(inputPump, false);
      engagePump(outputPump, false);
    });
}

bool pumppH(byte durationSeconds) {
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

bool pumpNutrients(TDSadjustState &state, byte delaySeconds) {
  switch (state) {
    case TDS_A_PUMP:
      {
        bool done = runCountDown(
          delaySeconds,
          []() {
            Serial.println("Engaging TDS A pump");
            engagePump(nutrientAPump, Forward);
          },
          []() {
            Serial.println("Stopping TDS A pump");
            engagePump(nutrientAPump, Stopped);
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
          delaySeconds,
          []() {
            Serial.println("Engaging TDS B pump");
            engagePump(nutrientBPump, Forward);
          },
          []() {
            Serial.println("Stopping TDS B pump");
            engagePump(nutrientBPump, Stopped);
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
bool pHadjustment(float target) {
  static const byte pHMeasureSamples = 5;
  static const byte pHCirculateSeconds = 40;
  static const byte acidPumpDelaySeconds = 2;
  static const float pHthreshold = 0.1f;
  static pHadjustState state = PH_MEASURE;
  static byte adjustmentRound = 0;
  float diff = 0;

  switch (state) {
    case PH_MEASURE:
      {
        Serial.print("Adjustment round: ");
        Serial.println(adjustmentRound);
        Serial.print("Target: ");
        Serial.println(target);
        float temperature = measureTemperature();
        float pH = measurepH(20);  // temperature);
        reportpH(pH);
        Serial.println();

        diff = abs(pH - target);
        if (pH < target || diff < pHthreshold || adjustmentRound > 5) {
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
        bool done = pumppH(round(acidPumpDelaySeconds));  // * diff));
        if (done) state = PH_CIRCULATE;
        break;
      }
    case PH_CIRCULATE:
      {
        bool done = circulate(pHCirculateSeconds);
        if (done) state = PH_MEASURE;
        break;
      }
  }

  return false;
}

bool TDSadjustment(float target = 0) {
  static const byte TDSCirculateSeconds = 40;
  static const byte TDSPumpDelaySeconds = 1;
  static float TDSThreshold = 0.1f;
  static TDSadjustState state = TDS_MEASURE;
  float diff = 0;
  static byte adjustmentRound = 0;

  switch (state) {
    case TDS_MEASURE:
      {
        Serial.print("Adjustment round: ");
        Serial.println(adjustmentRound + 1);
        Serial.print("Target: ");
        Serial.println(target);
        float TDS = measureTDS();
        reportTDS(TDS);
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
          state = (TDSadjustState)((int)state + 1);
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

void adjustpH() {
  Serial.println("adjust ph");
}

void adjustTDS() {
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
  menu.setAppearance(appearanceGeneral); // Note there is no `&` operator when setting general (or global) appearance of the menu

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
  menuPageAdjust.addMenuItem(menuItemAdjustpH);
  menuPageAdjust.addMenuItem(menuItemAdjustTDS);

  menuItemTargetpH.setPrecision(2);
  menuItemTargetTDS.setPrecision(2);
  menuPageTarget.addMenuItem(menuItemTargetpH);
  menuPageTarget.addMenuItem(menuItemTargetTDS);

  // Add menu page to menu and set it as current
  menu.setMenuPageCurrent(menuPageMain);
  
  menu.setTextSize(2);

  menu.invertKeysDuringEdit();
}

void setup() {
  acidPump.enablePin = 12;
  acidPump.directionAPin = 33;
  acidPump.directionBPin = 25;

  nutrientAPump.enablePin = 15;
  nutrientAPump.directionAPin = 33;
  nutrientAPump.directionBPin = 25;

  nutrientBPump.enablePin = 2;
  nutrientBPump.directionAPin = 33;  // 3
  nutrientBPump.directionBPin = 25;  // 21

  inputPump.relayPin = 39;
  outputPump.relayPin = 38;

  pinMode(nutrientAPump.enablePin, OUTPUT);
  pinMode(nutrientAPump.directionAPin, OUTPUT);
  pinMode(nutrientAPump.directionBPin, OUTPUT);

  pinMode(nutrientBPump.enablePin, OUTPUT);
  pinMode(nutrientBPump.directionAPin, OUTPUT);
  pinMode(nutrientBPump.directionBPin, OUTPUT);

  pinMode(acidPump.enablePin, OUTPUT);
  pinMode(acidPump.directionAPin, OUTPUT);
  pinMode(acidPump.directionBPin, OUTPUT);

  pinMode(inputPump.relayPin, OUTPUT);
  pinMode(outputPump.relayPin, OUTPUT);
  digitalWrite(inputPump.relayPin, HIGH);
  digitalWrite(outputPump.relayPin, HIGH);

  pinMode(pHSensorIn, INPUT);
  pinMode(temperatureSensorIn, INPUT);
  pinMode(TDSSensorIn, INPUT);

  pinMode(rightPin, INPUT_PULLUP);
  Serial.begin(115200);
  tft.init(240, 135);
  pinMode(TFT_BL, OUTPUT);      // TTGO T-Display enable Backlight pin 4
  digitalWrite(TFT_BL, HIGH);   // T-Display turn on Backlight
  tft.init(135, 240);           // Initialize ST7789 240x135
  tft.setRotation(1);
  menu.init();
  setupMenu();
  menu.drawMenu();

  rotaryEncoder.begin();
	rotaryEncoder.setup(readEncoderISR);
	rotaryEncoder.setBoundaries(0, 1000, true);
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

  static const byte emptyDelaySeconds = 10;
  static const byte fillDelaySeconds = 10;
  static const byte circulateDelaySeconds = 10;
  static byte plotDurationSeconds = 120;
  static State state = IDLE;
  bool done = false;

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
  }

  if (commandFromMenu != 0) {
    command = commandFromMenu;
    commandFromMenu = 0;
  }

  switch (command) {
    case 0:
      break;
    case '?':
      {
        printUsage();
        break;
      }
    case 't':
      {
        targetTDS = args.toFloat();
        Serial.print("Starting TDS adjustment with target: ");
        Serial.println(targetTDS);
        state = ADJUST_TDS;
        break;
      }
    case 'p':
      {
        targetpH = args.toFloat();
        Serial.print("Starting pH adjustment with target: ");
        Serial.println(targetpH);
        state = ADJUST_PH;
        break;
      }
    case 'a':
      {
        state = ADJUST_ALL;
        byte splitLocation = args.indexOf(' ');
        targetTDS = args.substring(0, splitLocation).toFloat();
        targetpH = args.substring(splitLocation + 1).toFloat();
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
        float TDS = measureTDS();
        Serial.println();
        reportTDS(TDS);
        Serial.println();
        float temperature = measureTemperature();
        Serial.println();
        reportTemperature(temperature);
        Serial.println();
        float pH = measurepH(20);  //temperature);
        Serial.println();
        reportpH(pH);
        Serial.println();
        state = IDLE;
        break;
      }
    case PLOT:
      {
        bool done = runCountDown(
          plotDurationSeconds,
          []() {},
          []() {});

        if (done) {
          Serial.println("Stopping plotting");
          state = IDLE;
        }

        float TDS = measureTDS();
        reportTDS(TDS);
        Serial.print(",");
        float temperature = measureTemperature();
        reportTemperature(temperature);
        Serial.print(",");
        float pH = measurepH(20);  //temperature);
        reportpH(pH);
        Serial.println();
        break;
      }
    case ADJUST_PH:
      {
        done = pHadjustment(targetpH);
        if (done) {
          Serial.println("pH adjustment done");
        }
        break;
      }
    case ADJUST_TDS:
    case ADJUST_ALL:
      {
        done = TDSadjustment(targetTDS);
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
        bool done = pumppH(2);
        if (done) state = CIRCULATE;
        break;
      }
    case PUMP_NUTRIENT:
      {
        bool done = TDSadjustment();
        if (done) state = CIRCULATE;
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
