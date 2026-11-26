// Compiles the unmodified firmware (nutrient_mixer.ino, with prototypes generated like the Arduino builder does)
// and lets the simulator UI show the firmware's internal state. Only this file knows the firmware's variables.
#include "nutrient_mixer.ino.cpp"

#include "json.h"

namespace sim {

template <size_t N>
static const char *nameOf(const char *const (&names)[N], int value) {
  return value >= 0 && value < (int)N ? names[value] : "?";
}

static Json calibrationJson(const Calibration &calibration) {
  return Json::object()
      .set("lowValue", calibration.lowValue)
      .set("lowMilliVolts", calibration.lowMilliVolts)
      .set("highValue", calibration.highValue)
      .set("highMilliVolts", calibration.highMilliVolts);
}

static Json pumpJson(const Pump &pump) {
  return Json::object().set("requested", pump.requested).set("blocked", pump.overflow);
}

Json firmwareSnapshot() {
  static const char *const stateNames[] = {"IDLE", "CIRCULATE", "EMPTY", "FILL", "MEASURE", "PLOT", "ADJUST_PH",
                                           "ADJUST_TDS", "ADJUST_ALL", "PUMP_PH", "PUMP_NUTRIENT_A", "PUMP_NUTRIENT_B"};
  static const char *const phStateNames[] = {"PH_MEASURE", "PH_PUMP", "PH_CIRCULATE", "PH_SETTLE"};
  static const char *const tdsStateNames[] = {"TDS_MEASURE", "TDS_A_PUMP", "TDS_B_PUMP", "TDS_CIRCULATE", "TDS_SETTLE"};
  static const char *const sensorNames[] = {"none", "TDS", "pH"};

  Json json = Json::object();
  json.set("state", nameOf(stateNames, state));
  json.set("phState", nameOf(phStateNames, phState));
  json.set("tdsState", nameOf(tdsStateNames, tdsState));
  json.set("phRound", phAdjustmentRound);
  json.set("tdsRound", tdsAdjustmentRound);
  json.set("maxRounds", maxAdjustmentRounds);
  json.set("targetPh", targetPh);
  json.set("targetTds", targetTds);
  json.set("acidSeconds", acidPumpSeconds);
  json.set("nutrientASeconds", nutrientAPumpSeconds);
  json.set("nutrientBSeconds", nutrientBPumpSeconds);
  json.set("emptySeconds", emptySeconds);
  json.set("fillSeconds", fillSeconds);
  json.set("circulateSeconds", circulateManuallySeconds);
  json.set("ph", currentPhString);
  json.set("tds", currentTdsString);
  const unsigned long elapsed = millis() - countDown.startedAt;
  json.set("countDown", countDown.running ? Json((countDown.durationMilliseconds - std::min(elapsed, countDown.durationMilliseconds)) / 1000.0) : Json());
  json.set("countDownTotal", countDown.running ? Json(countDown.durationMilliseconds / 1000.0) : Json());
  json.set("poweredSensor", nameOf(sensorNames, poweredSensor));
  json.set("qrScreen", qrScreenVisible);
  json.set("editMode", menu.isEditMode());
  json.set("register", registerState);
  json.set("inputPump", pumpJson(inputPump));
  json.set("outputPump", pumpJson(outputPump));
  json.set("phCalibration", calibrationJson(phCalibration));
  json.set("tdsCalibration", calibrationJson(tdsCalibration));
  json.set("wifi", wifiStateString);
  json.set("hotspot", hotspotName);
  return json;
}

}  // namespace sim
