#include "Print.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include <vector>

size_t Print::printf(const char *format, ...) {
  va_list arguments;
  va_start(arguments, format);
  va_list copy;
  va_copy(copy, arguments);
  const int length = vsnprintf(nullptr, 0, format, copy);
  va_end(copy);
  if (length < 0) {
    va_end(arguments);
    return 0;
  }
  std::vector<char> buffer(length + 1);
  vsnprintf(buffer.data(), buffer.size(), format, arguments);
  va_end(arguments);
  return write((const uint8_t *)buffer.data(), length);
}

size_t Print::print(long number, int base) {
  if (base == 0) return write((uint8_t)number);
  if (base == 10 && number < 0) {
    const size_t n = print('-');
    return n + printNumber(-(unsigned long long)(long long)number, 10);
  }
  return printNumber((unsigned long)number, base);
}

size_t Print::print(unsigned long number, int base) {
  if (base == 0) return write((uint8_t)number);
  return printNumber(number, base);
}

size_t Print::print(long long number, int base) {
  if (base == 0) return write((uint8_t)number);
  if (base == 10 && number < 0) {
    const size_t n = print('-');
    return n + printNumber(-(unsigned long long)number, 10);
  }
  return printNumber((unsigned long long)number, base);
}

size_t Print::print(unsigned long long number, int base) {
  if (base == 0) return write((uint8_t)number);
  return printNumber(number, base);
}

size_t Print::printNumber(unsigned long long number, uint8_t base) {
  char buffer[8 * sizeof(long long) + 1];
  char *text = &buffer[sizeof(buffer) - 1];
  *text = '\0';
  if (base < 2) base = 10;
  do {
    const char digit = number % base;
    number /= base;
    *--text = digit < 10 ? digit + '0' : digit + 'A' - 10;
  } while (number);
  return write(text);
}

// Same algorithm as the Arduino core (rounds, then prints the integer part and the requested digits)
size_t Print::printFloat(double number, uint8_t digits) {
  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
  if (number > 4294967040.0) return print("ovf");
  if (number < -4294967040.0) return print("ovf");

  size_t n = 0;
  if (number < 0.0) {
    n += print('-');
    number = -number;
  }

  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i) rounding /= 10.0;
  number += rounding;

  unsigned long integerPart = (unsigned long)number;
  double remainder = number - (double)integerPart;
  n += print(integerPart);

  if (digits > 0) n += print('.');
  while (digits-- > 0) {
    remainder *= 10.0;
    const unsigned int digit = (unsigned int)remainder;
    n += print(digit);
    remainder -= digit;
  }
  return n;
}
