// Arduino Print class: formats numbers and text the same way as the Arduino core
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "WString.h"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class Print;

class Printable {
 public:
  virtual ~Printable() {}
  virtual size_t printTo(Print &p) const = 0;
};

class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t *buffer, size_t size) {
    size_t n = 0;
    while (size--) {
      if (write(*buffer++)) {
        n++;
      } else {
        break;
      }
    }
    return n;
  }
  size_t write(const char *text) { return text ? write((const uint8_t *)text, strlen(text)) : 0; }
  size_t write(const char *buffer, size_t size) { return write((const uint8_t *)buffer, size); }
  virtual int availableForWrite() { return 0; }
  virtual void flush() {}

  size_t printf(const char *format, ...) __attribute__((format(printf, 2, 3)));

  size_t print(const __FlashStringHelper *text) { return print(reinterpret_cast<const char *>(text)); }
  size_t print(const String &text) { return write(text.c_str(), text.length()); }
  size_t print(const char text[]) { return write(text); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(unsigned char number, int base = DEC) { return print((unsigned long)number, base); }
  size_t print(int number, int base = DEC) { return print((long)number, base); }
  size_t print(unsigned int number, int base = DEC) { return print((unsigned long)number, base); }
  size_t print(long number, int base = DEC);
  size_t print(unsigned long number, int base = DEC);
  size_t print(long long number, int base = DEC);
  size_t print(unsigned long long number, int base = DEC);
  size_t print(double number, int digits = 2) { return printFloat(number, digits); }
  size_t print(const Printable &printable) { return printable.printTo(*this); }

  size_t println(void) { return write("\r\n"); }
  template <typename T>
  size_t println(const T &value) {
    const size_t n = print(value);
    return n + println();
  }
  template <typename T>
  size_t println(const T &value, int format) {
    const size_t n = print(value, format);
    return n + println();
  }
  size_t println(const char text[]) {
    const size_t n = print(text);
    return n + println();
  }

 private:
  size_t printNumber(unsigned long long number, uint8_t base);
  size_t printFloat(double number, uint8_t digits);
};
