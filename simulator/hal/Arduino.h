// Simulated Arduino core for the ESP32 (arduino-esp32 2.x API as used by the firmware and its libraries)
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
#include <algorithm>
#include <cmath>
#endif

#include "esp_attr.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "pgmspace.h"
#include "stdlib_noniso.h"

typedef uint8_t byte;
typedef bool boolean;
typedef uint16_t word;

#define HIGH 0x1
#define LOW 0x0

// Pin modes (same values as arduino-esp32)
#define INPUT 0x01
#define OUTPUT 0x03
#define PULLUP 0x04
#define INPUT_PULLUP 0x05
#define PULLDOWN 0x08
#define INPUT_PULLDOWN 0x09
#define OPEN_DRAIN 0x10
#define OUTPUT_OPEN_DRAIN 0x12
#define ANALOG 0xC0

// Interrupt modes
#define DISABLED 0x00
#define RISING 0x01
#define FALLING 0x02
#define CHANGE 0x03
#define ONLOW 0x04
#define ONHIGH 0x05

#define LSBFIRST 0
#define MSBFIRST 1

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))
#define _min(a, b) ((a) < (b) ? (a) : (b))
#define _max(a, b) ((a) > (b) ? (a) : (b))

#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))

#define digitalPinToInterrupt(p) (p)
#define NOT_AN_INTERRUPT -1

#define interrupts()
#define noInterrupts()

typedef enum {
  ADC_0db,
  ADC_2_5db,
  ADC_6db,
  ADC_11db
} adc_attenuation_t;

#ifdef __cplusplus
using std::abs;
using std::isinf;
using std::isnan;
using std::max;
using std::min;
using ::round;

#include "HardwareSerial.h"
#include "WString.h"

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);

int analogRead(uint8_t pin);
uint32_t analogReadMilliVolts(uint8_t pin);
void analogReadResolution(uint8_t bits);
void analogSetAttenuation(adc_attenuation_t attenuation);
void analogSetPinAttenuation(uint8_t pin, adc_attenuation_t attenuation);
void analogWrite(uint8_t pin, int value);

unsigned long millis();
unsigned long micros();
void delay(uint32_t milliseconds);
void delayMicroseconds(uint32_t microseconds);
void yield();

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

void attachInterrupt(uint8_t pin, void (*handler)(void), int mode);
void detachInterrupt(uint8_t pin);

long random(long howBig);
long random(long howSmall, long howBig);
void randomSeed(unsigned long seed);
long map(long x, long inMin, long inMax, long outMin, long outMax);

void setup();
void loop();
#endif
