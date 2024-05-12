#include <CountDown.h>
#include <DFRobot_PH.h>
#include <WiFi.h>
#include <CQRobotTDS.h>
#include <ESP32MQTTClient.h>

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
  ADJUST_PH,
  ADJUST_TDS,
  ADJUST_ALL,
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

const byte pHSensorIn = 36;
const byte temperatureSensorIn = 39;
const byte TDSSensorIn = 34;

float targetpH = 7;
float currentpH = 7;

float targetTDS = 2;
float currentTDS = 2;

const char *ssid = "razorwlan";
const char *password = "mansikkaaempaeri";

const char *mqttServer = "mqtt://sensorpi.local:1883";
const char *mqttTopic = "homeassistant/sensor/nutrient-mixer";

ESP32MQTTClient mqttClient;
DFRobot_PH pHsensor;

void setup() {
  acidPump.enablePin = 17;
  acidPump.directionAPin = 4;
  acidPump.directionBPin = 16;

  nutrientAPump.enablePin = 5;
  nutrientAPump.directionAPin = 18;
  nutrientAPump.directionBPin = 19;

  nutrientBPump.enablePin = 22;
  nutrientBPump.directionAPin = 18;
  nutrientBPump.directionBPin = 19;

  inputPump.relayPin = 15;
  outputPump.relayPin = 2;

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

  pinMode(pHSensorIn, INPUT);
  pinMode(temperatureSensorIn, INPUT);
  pinMode(TDSSensorIn, INPUT);

  Serial.begin(115200);
  pHsensor.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");

  Serial.println("Connecting to MQTT");
  mqttClient.enableDebuggingMessages();
  mqttClient.setURI(mqttServer);
  mqttClient.enableLastWillMessage("lwt", "I am going offline");
  mqttClient.setKeepAlive(30);
  mqttClient.loopStart();

  printUsage();
}

void onConnectionEstablishedCallback(esp_mqtt_client_handle_t client) {
  Serial.println("MQTT connection established");
  if (mqttClient.isMyTurn(client))  // can be omitted if only one client
  {
    mqttClient.subscribe(mqttTopic, [](const String &payload) {
      log_i("%s: %s", mqttTopic, payload.c_str());
    });
  }
}

esp_err_t handleMQTT(esp_mqtt_event_handle_t event) {
  Serial.println("MQTT event received");
  mqttClient.onEventCallback(event);
  return ESP_OK;
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

void engagePump(Pump pump, bool pumping) {
  digitalWrite(pump.relayPin, pumping ? HIGH : LOW);
}

float measurepH() {
  static DFRobot_PH ph;
  static const byte pHMeasureSamples = 5;
  static const byte pHMeasureDelayMilliseconds = 100;
  static const float temperature = 20;

  float currentValue = 0;
  for (byte i = 0; i < pHMeasureSamples; ++i) {
    delay(pHMeasureDelayMilliseconds);
    float voltage = analogRead(pHSensorIn) / 1024.0 * 3300;  // TODO: verify this applies for ESP32
    Serial.print("pH sensor output voltage: ");
    Serial.println(analogRead(pHSensorIn));
    float pHValue = pHsensor.readPH(voltage, temperature);
    Serial.print("pH sensor voltage converted value: ");
    Serial.println(pHValue);
    currentValue = (currentValue * (i - 1) + pHValue) / i;
  }

  Serial.print("Returning pH: ");
  Serial.println(currentValue);
  return currentValue;
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

bool circulate(byte delaySeconds) {
  return runCountDown(
    delaySeconds, []() {
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

void reportpH(float value) {
  Serial.print("pH: ");
  Serial.println(value);
}

bool pHadjustment(float target) {
  static const byte pHMeasureSamples = 5;
  static const byte pHCirculateSeconds = 20;
  static const byte acidPumpDelaySeconds = 5;
  static const float pHthreshold = 0.1f;
  static pHadjustState state = PH_MEASURE;
  static byte adjustmentRound = 0;
  float diff = 0;

  switch (state) {
    case PH_MEASURE:
      {
        Serial.print("Adjustment round: ");
        Serial.println(adjustmentRound);
        float pH = measurepH();
        reportpH(pH);

        diff = pH - target;
        if (diff < pHthreshold || adjustmentRound > 5) {
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
        bool done = pumppH(round(acidPumpDelaySeconds)); // * diff));
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

void reportTDS(float value) {
  Serial.print("TDS: ");
  Serial.println(value);
}

float measureTDS() {
  static CQRobotTDS tds(TDSSensorIn);
  static const byte TDSMeasureSamples = 5;

  float currentValue = 0;
  for (byte i = 0; i < TDSMeasureSamples; ++i) {
    float temperature = 20;  // TODO: read temperature from sensor
    Serial.print("TDS sensor output voltage: ");
    Serial.println(analogRead(TDSSensorIn));
    float TDS = tds.update(temperature);
    Serial.print("TDS sensor voltage converted value: ");
    Serial.println(TDS);
    currentValue = (currentValue * (i - 1) + TDS) / i;
  }

  return currentValue;
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

bool TDSadjustment(float target) {
  static const byte TDSCirculateSeconds = 20;
  static const byte TDSPumpDelaySeconds = 5;
  static float TDSThreshold = 0.1f;
  static TDSadjustState state = TDS_MEASURE;
  float diff = 0;
  static byte adjustmentRound = 0;

  switch (state) {
    case TDS_MEASURE:
      {
        Serial.print("Adjustment round: ");
        Serial.println(adjustmentRound);
        float TDS = measureTDS();
        reportTDS(TDS);
        diff = TDS - target;
        if (diff < TDSThreshold || adjustmentRound > 5) {
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

void printUsage() {
  Serial.println("Commands:");
  Serial.println("m # Measure and report pH and TDS");
  Serial.println("t 2.0 # Adjust TDS to 2.0");
  Serial.println("p 7.0 # Adjust pH to 7.0");
  Serial.println("a 2.0 7.0 # Adjust TDS to 2.0 and pH to 7.0");
  Serial.println("e # Empty tank");
  Serial.println("f # Fill tank");
  Serial.println("c # Circulate");
  Serial.println("! # Abort current operation (stop all pumps) and go to idle");
}

void loop() {
  static const byte emptyDelaySeconds = 30;
  static const byte fillDelaySeconds = 30;
  static const byte circulateDelaySeconds = 100;
  static State state = IDLE;
  bool done = false;

  if (Serial.available() > 0) {
    String incoming = Serial.readString();
    incoming.trim();
    char command = incoming[0];
    String args = incoming.substring(2, incoming.length());
    Serial.print("Received command: ");
    Serial.println(command);

    if (state != IDLE && command != '!') {
      Serial.println("Operation in progress. Cannot run command. Please wait for previous command to finish or abort with '!'");
    }

    switch (command) {
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
          targetpH = args.substring(0, splitLocation).toFloat();
          targetTDS = args.substring(splitLocation + 1).toFloat();
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
      case 'm':
        {
          state = MEASURE;
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
        reportTDS(TDS);
        float pH = measurepH();
        reportpH(pH);
        state = IDLE;
        break;
      }
    case ADJUST_PH:
      {
        done = pHadjustment(targetpH);
        break;
      }
    case ADJUST_TDS:
    case ADJUST_ALL:
      {
        done = TDSadjustment(targetTDS);
        if (state == ADJUST_ALL && done) {
          state = ADJUST_PH;
          done = false;
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
      }
    case IDLE:
    default:
      {
        break;
      }
  }

  if (done) state = IDLE;
}

/*
#define MQTT_DEVICE_NAME "watering_station_%s/"
#define MQTT_MOISTURE_SENSOR_PREFIX MQTT_DEVICE_NAME "moisture_"
#define HOMEASSISTANT_SENSOR_TOPIC_PREFIX "homeassistant/sensor/"
#define MQTT_WATER_TANK_SENSOR_NAME MQTT_DEVICE_NAME "water_tank_"
#define MQTT_WATER_TANK_SENSOR_TOPIC HOMEASSISTANT_SENSOR_TOPIC_PREFIX MQTT_WATER_TANK_SENSOR_NAME
#define MQTT_RELAY_PREFIX MQTT_DEVICE_NAME "relay_"

const char waterTankConfigTopicTemplate[] = MQTT_WATER_TANK_SENSOR_TOPIC "/config";
  const char waterTankConfigValueTemplate[] = "{\"name\": \"Water tank level\", \"~\": \"" MQTT_WATER_TANK_SENSOR_TOPIC "\", \"stat_t\": \"~/state\", \"value_template\": \"{{ value_json.level}}\", \"unit_of_meas\": \"%%\"}";
  char waterTankConfigTopic[sizeof(waterTankConfigTopicTemplate) + MAC_ADDRESS_LENGTH] = "";
  sprintf(waterTankConfigTopic, waterTankConfigTopicTemplate, macAddress);
  char waterTankConfigValue[sizeof(waterTankConfigValueTemplate) + MAC_ADDRESS_LENGTH] = "";
  sprintf(waterTankConfigValue, waterTankConfigValueTemplate, macAddress);
  client.publish(waterTankConfigTopic, waterTankConfigValue, true);

  const char relayConfigTopicTemplate[] = "homeassistant/switch/" MQTT_RELAY_PREFIX "%u/config";
  const char relayConfigValueTemplate[] = "{\"name\": \"Water pump %u\", \"~\": \"homeassistant/switch/" MQTT_RELAY_PREFIX "%u\", \"state_topic\": \"~/state\", \"cmd_t\": \"~/set\", \"payload_on\": \"ON\", \"payload_off\": \"OFF\"}";
  const char relayCommandTopicTemplate[] = "homeassistant/switch/" MQTT_RELAY_PREFIX "%u/set";
  const char relayStateTopicTemplate[] = "homeassistant/switch/" MQTT_RELAY_PREFIX "%u/state";
  for (byte i = 1; i <= RELAY_PIN_COUNT; ++i) {
    char relayConfigTopic[sizeof(relayConfigTopicTemplate) + (MAC_ADDRESS_LENGTH * 2)] = "";
    sprintf(relayConfigTopic, relayConfigTopicTemplate, macAddress, i);
    char relayConfigValue[sizeof(relayConfigValueTemplate) + MAC_ADDRESS_LENGTH * 2] = "";
    sprintf(relayConfigValue, relayConfigValueTemplate, i, macAddress, i);

    client.publish(
      relayConfigTopic,
      relayConfigValue,
      true
    );

    char commandTopic[sizeof(relayCommandTopicTemplate) + MAC_ADDRESS_LENGTH];
    sprintf(commandTopic, relayCommandTopicTemplate, macAddress, i);
    char stateTopic[sizeof(relayStateTopicTemplate) + MAC_ADDRESS_LENGTH];
    sprintf(stateTopic, relayStateTopicTemplate, macAddress, i);
    
    client.subscribe(commandTopic, [i, stateTopic](const String& message) {
      const boolean setOn = message == "ON";
      setRelayState(i, setOn);
      client.publish(stateTopic, setOn ? "ON" : "OFF", true);
    });
  }
  */