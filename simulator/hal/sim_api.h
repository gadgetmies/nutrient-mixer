// Interface between the simulated Arduino/ESP32 API (this directory) and the simulator core (../src).
// The firmware and its libraries only see the Arduino API; everything they do with the hardware ends up here.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>

namespace simhal {

// Virtual time in microseconds since boot. It only advances when the firmware spends time (each API call has a
// small cost, delay() spends the requested time), so runs are deterministic at any simulation speed.
uint64_t micros64();
void spend(uint64_t microseconds);

// GPIO
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t level);
int digitalRead(uint8_t pin);
// Voltage at an ADC pin in millivolts as seen by the ADC (before ADC noise and saturation are applied)
uint32_t analogReadMilliVolts(uint8_t pin);
int analogReadRaw(uint8_t pin);
void attachInterrupt(uint8_t pin, void (*handler)(), int mode);
void detachInterrupt(uint8_t pin);
uint32_t random32();

// Serial port (UART0, connected to the USB serial converter of the T-Display)
void serialWrite(const uint8_t *data, size_t length);
int serialAvailable();
int serialRead();
int serialPeek();

// NVS flash storage used by Preferences
enum NvsType : uint8_t {
  NVS_U8 = 1,
  NVS_I8,
  NVS_U16,
  NVS_I16,
  NVS_U32,
  NVS_I32,
  NVS_U64,
  NVS_I64,
  NVS_STR,
  NVS_BLOB
};
bool nvsGet(const std::string &space, const std::string &key, NvsType &type, std::vector<uint8_t> &data);
void nvsSet(const std::string &space, const std::string &key, NvsType type, const std::vector<uint8_t> &data);
bool nvsRemove(const std::string &space, const std::string &key);
void nvsClear(const std::string &space);

// ST7789 panel of the T-Display: 135 x 240 pixels (RGB565) in the panel's own (portrait) orientation
const int panelWidth = 135;
const int panelHeight = 240;
uint16_t *panelPixels();
void panelChanged();
void panelBacklight(bool on);

}  // namespace simhal
