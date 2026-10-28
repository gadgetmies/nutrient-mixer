# Arduino Hydroponics Nutrient Mixer

Implemented for ESP32 Devkit (v1?) / TTGO T-Display. Requires v2.x of the ESP32 board package.

Send `?` over serial (115200 baud) to list the commands.

## WiFi

There are no network credentials in the code. When no network has been configured, the device starts a hotspot
named `Mix-XXXX` and shows a setup screen with a QR code for joining it. Scan the code with a phone camera (or
join the hotspot with the name and password shown next to the code), choose the network on the setup page that
opens and enter its password. If the page does not open by itself, go to http://192.168.4.1. The credentials
are stored in the ESP32's flash.

The hotspot password is generated on first boot. Any key returns from the setup screen to the menu; the screen
can be shown again from WiFi > Show QR code (or `w qr`).

With a configured network the device keeps reconnecting in the background instead of opening the hotspot.
To change the network, use WiFi > Forget WiFi in the menu or send `w reset`. Requires the WiFiManager library
(tzapu, 2.0 or newer).

## Sensors

The pH and TDS sensors are powered through the shift register, one at a time so that the probes do not
interfere with each other. A sensor is read only after it has been powered for 2 seconds
(`sensorWarmUpMilliseconds`).

## Sensor calibration

The pH and TDS sensors use a two point calibration that is stored in flash. To calibrate, put the probe in the
reference solution, wait until the readings are stable (e.g. `l 60`) and send:

- `k ph 4.0` and `k ph 7.0` with pH 4.0 and 7.0 buffers
- `k tds 707` with a TDS standard solution (a value below 100 ppm, e.g. `k tds 0` in distilled water, sets the low point)

`k` shows the current calibration and `k reset` restores the defaults.

## Float switches

A triggered float switch blocks its pump immediately and lights the error LED. The pump continues once the
switch has stayed released for a second.

## Menu

The menu follows `menu.txt` on the `local-main` branch. Pump amounts (dose run time, also used per adjustment
round), reservoir pump times and the targets are saved in flash. Requires GEM 1.5 or newer (`GEMSpinner`).

## Simulator

`simulator/` runs this firmware on a Mac against a simulated T-Display, the mixer board (`eagle/`) and the
pumps, reservoirs and sensors around it, with a browser UI: `cd simulator && make run`. See
[simulator/README.md](simulator/README.md).
