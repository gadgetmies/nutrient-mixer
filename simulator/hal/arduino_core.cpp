// Arduino core functions on top of the simulator. Each call spends a little virtual time, roughly what it takes
// on the ESP32, so that busy waiting loops in libraries see time pass.
#include <stdarg.h>

#include "Arduino.h"
#include "sim_api.h"

HardwareSerial Serial;

static uint8_t adcResolutionBits = 12;

void pinMode(uint8_t pin, uint8_t mode) {
  simhal::spend(2);
  simhal::pinMode(pin, mode);
}

void digitalWrite(uint8_t pin, uint8_t value) {
  simhal::spend(1);
  simhal::digitalWrite(pin, value ? HIGH : LOW);
}

int digitalRead(uint8_t pin) {
  simhal::spend(1);
  return simhal::digitalRead(pin);
}

int analogRead(uint8_t pin) {
  simhal::spend(20);
  const int raw = simhal::analogReadRaw(pin);  // 12 bits
  if (adcResolutionBits >= 12) return raw << (adcResolutionBits - 12);
  return raw >> (12 - adcResolutionBits);
}

uint32_t analogReadMilliVolts(uint8_t pin) {
  simhal::spend(25);
  return simhal::analogReadMilliVolts(pin);
}

void analogReadResolution(uint8_t bits) { adcResolutionBits = constrain(bits, 9, 12); }
void analogSetAttenuation(adc_attenuation_t) {}
// The simulator models the 11 dB attenuation that the firmware uses for all its analog inputs
void analogSetPinAttenuation(uint8_t, adc_attenuation_t) {}
void analogWrite(uint8_t pin, int value) { digitalWrite(pin, value > 127 ? HIGH : LOW); }

unsigned long millis() {
  simhal::spend(1);
  return (unsigned long)(simhal::micros64() / 1000);
}

unsigned long micros() {
  simhal::spend(1);
  return (unsigned long)simhal::micros64();
}

void delay(uint32_t milliseconds) { simhal::spend((uint64_t)milliseconds * 1000); }
void delayMicroseconds(uint32_t microseconds) { simhal::spend(microseconds); }
void yield() { simhal::spend(1); }

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value) {
  for (uint8_t i = 0; i < 8; i++) {
    if (bitOrder == LSBFIRST) {
      digitalWrite(dataPin, !!(value & (1 << i)));
    } else {
      digitalWrite(dataPin, !!(value & (1 << (7 - i))));
    }
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
  }
}

uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
  uint8_t value = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    digitalWrite(clockPin, HIGH);
    if (bitOrder == LSBFIRST) {
      value |= digitalRead(dataPin) << i;
    } else {
      value |= digitalRead(dataPin) << (7 - i);
    }
    digitalWrite(clockPin, LOW);
  }
  return value;
}

void attachInterrupt(uint8_t pin, void (*handler)(void), int mode) { simhal::attachInterrupt(pin, handler, mode); }
void detachInterrupt(uint8_t pin) { simhal::detachInterrupt(pin); }

long random(long howBig) {
  if (howBig <= 0) return 0;
  return simhal::random32() % howBig;
}

long random(long howSmall, long howBig) {
  if (howSmall >= howBig) return howSmall;
  return random(howBig - howSmall) + howSmall;
}

void randomSeed(unsigned long) {}

long map(long x, long inMin, long inMax, long outMin, long outMax) {
  if (inMax == inMin) return outMin;
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

extern "C" uint32_t esp_random(void) { return simhal::random32(); }

// SERIAL

int HardwareSerial::available() {
  simhal::spend(1);
  return simhal::serialAvailable();
}

int HardwareSerial::read() {
  simhal::spend(1);
  return simhal::serialRead();
}

int HardwareSerial::peek() {
  simhal::spend(1);
  return simhal::serialPeek();
}

size_t HardwareSerial::write(uint8_t c) {
  simhal::serialWrite(&c, 1);
  return 1;
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size) {
  simhal::serialWrite(buffer, size);
  return size;
}

// NON-ISO STDLIB (stdlib_noniso.h)

extern "C" char *dtostrf(double number, signed char width, unsigned char precision, char *buffer) {
  // Same output as the arduino-esp32 implementation: fixed notation, right aligned to width (left if negative)
  char format[16];
  snprintf(format, sizeof(format), "%%%d.%df", width, precision);
  sprintf(buffer, format, number);
  return buffer;
}

static char *formatUnsigned(unsigned long value, char *buffer, int base, bool negative) {
  char digits[sizeof(unsigned long) * 8 + 1];
  int n = 0;
  if (base < 2 || base > 36) base = 10;
  do {
    const int digit = value % base;
    digits[n++] = digit < 10 ? '0' + digit : 'a' + digit - 10;
    value /= base;
  } while (value);
  char *out = buffer;
  if (negative) *out++ = '-';
  while (n) *out++ = digits[--n];
  *out = 0;
  return buffer;
}

extern "C" char *ltoa(long value, char *buffer, int base) {
  if (value < 0 && base == 10) return formatUnsigned(-(unsigned long)value, buffer, base, true);
  return formatUnsigned((unsigned long)value, buffer, base, false);
}
extern "C" char *itoa(int value, char *buffer, int base) {
  if (value < 0 && base == 10) return formatUnsigned(-(unsigned long)(long)value, buffer, base, true);
  return formatUnsigned((unsigned int)value, buffer, base, false);
}
extern "C" char *utoa(unsigned int value, char *buffer, int base) { return formatUnsigned(value, buffer, base, false); }
extern "C" char *ultoa(unsigned long value, char *buffer, int base) { return formatUnsigned(value, buffer, base, false); }
