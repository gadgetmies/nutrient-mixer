#include "actions.h"

#include "sim.h"
#include "wifi_sim.h"

namespace sim {

namespace {

bool buttonFromName(const std::string &name, Button &button) {
  if (name == "io0") {
    button = IO0;
  } else if (name == "io35") {
    button = IO35;
  } else if (name == "encoder") {
    button = ENCODER_BUTTON;
  } else if (name == "abort") {
    button = ABORT_BUTTON;
  } else {
    return false;
  }
  return true;
}

Tank *tankFromName(World &world, const std::string &name) {
  if (name == "mixer") return &world.mixer;
  if (name == "plant") return &world.plant;
  return nullptr;
}

}  // namespace

std::string performAction(const Json &message) {
  Simulator &s = simulator();
  const std::string type = message["type"].asString();

  if (type == "serial") {
    s.serialInput(message["text"].asString());
    return "";
  }
  if (type == "encoder") {
    const int detents = (int)message["detents"].asNumber();
    if (detents != 0) s.rotateEncoder(std::max(-50, std::min(50, detents)));
    return "";
  }
  if (type == "button") {
    Button button;
    if (!buttonFromName(message["button"].asString(), button)) return "unknown button";
    const std::string action = message["action"].asString();
    if (action == "press") {
      s.pressButton(button);
    } else if (action == "release") {
      s.releaseButton(button);
    } else {
      s.clickButton(button, message["hold"].asNumber(0.12));
    }
    return "";
  }
  if (type == "param") {
    std::string error;
    const Json &value = message["value"];
    if (value.isNumber()) {
      if (!s.params.set(message["key"].asString(), value.asNumber())) return "unknown parameter";
    } else if (!s.params.setText(message["key"].asString(), value.toText(), &error)) {
      return error;
    }
    return "";
  }
  if (type == "resetParams") {
    s.params.resetToDefaults();
    return "";
  }
  if (type == "world") {
    const std::string action = message["action"].asString();
    if (action == "reset") {
      s.resetWorld((int)message["preset"].asNumber());
    } else if (action == "refill") {
      s.world.refillBottle((int)message["bottle"].asNumber());
    } else if (action == "volume") {
      Tank *tank = tankFromName(s.world, message["tank"].asString());
      if (!tank) return "unknown tank";
      s.world.setTankVolume(*tank, message["litres"].asNumber());
    } else if (action == "clearSpill") {
      s.world.mixer.spilled = s.world.plant.spilled = 0;
    } else {
      return "unknown world action";
    }
    return "";
  }
  if (type == "wifi") {
    WifiWorld &wifi = wifiWorld();
    const std::string action = message["action"].asString();
    if (action == "network") {
      Network *network = wifi.findNetwork(message["ssid"].asString());
      if (!network) return "unknown network";
      if (message.has("on")) network->on = message["on"].asBool();
      if (message.has("password")) network->password = message["password"].asString();
    } else if (action == "phoneJoin") {
      wifi.phoneJoin(message["ssid"].asString(), message["password"].asString());
    } else if (action == "phoneScan") {
      // The phone camera reads the QR code from the display (WIFI:T:WPA;S:<name>;P:<password>;;)
      if (!firmwareSnapshot()["qrScreen"].asBool() || !s.backlight) {
        wifi.phoneMessage = "No QR code in view. Show it with WiFi > Show QR code.";
      } else {
        wifi.phoneJoin(wifi.apSsid, wifi.apPassword);
      }
    } else if (action == "phoneSubmit") {
      wifi.phoneSubmit(message["ssid"].asString(), message["password"].asString());
    } else if (action == "phoneLeave") {
      wifi.phoneJoined = false;
      wifi.phoneMessage = "Disconnected from the hotspot";
    } else {
      return "unknown wifi action";
    }
    return "";
  }
  if (type == "device") {
    const std::string action = message["action"].asString();
    if (action == "reset") {
      s.requestRestart();
    } else if (action == "eraseFlash") {
      s.eraseFlash();
      s.requestRestart();
    } else {
      return "unknown device action";
    }
    return "";
  }
  return "unknown message type " + type;
}

void handleUiMessage(const Json &message) {
  Simulator &s = simulator();
  const std::string type = message["type"].asString();
  if (type == "speed") {
    s.setSpeed(message["value"].asNumber(1));
  } else if (type == "pause") {
    s.setPaused(message["value"].asBool());
  } else if (type == "step") {
    s.stepFor(message["seconds"].asNumber(0.1));
  } else {
    s.post([message] {
      const std::string error = performAction(message);
      if (!error.empty()) fprintf(stderr, "UI action failed: %s\n", error.c_str());
    });
  }
}

}  // namespace sim
