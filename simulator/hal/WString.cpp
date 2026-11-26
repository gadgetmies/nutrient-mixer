#include "WString.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static std::string formatInteger(unsigned long long number, unsigned char base, bool negative) {
  if (base < 2) base = 10;
  char buffer[72];
  int i = sizeof(buffer) - 1;
  buffer[i] = 0;
  do {
    const int digit = number % base;
    buffer[--i] = digit < 10 ? '0' + digit : 'a' + digit - 10;
    number /= base;
  } while (number > 0);
  if (negative) buffer[--i] = '-';
  return std::string(buffer + i);
}

static std::string formatSigned(long long number, unsigned char base) {
  // Like the Arduino core, only base 10 numbers get a sign; other bases show the two's complement
  if (base == 10 && number < 0) return formatInteger(-(unsigned long long)number, base, true);
  return formatInteger((unsigned long)number, base, false);
}

static std::string formatFloat(double number, unsigned int decimals) {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "%.*f", decimals, number);
  return buffer;
}

String::String(unsigned char number, unsigned char base) : value(formatInteger(number, base, false)) {}
String::String(int number, unsigned char base) : value(formatSigned(number, base)) {}
String::String(unsigned int number, unsigned char base) : value(formatInteger(number, base, false)) {}
String::String(long number, unsigned char base) : value(formatSigned(number, base)) {}
String::String(unsigned long number, unsigned char base) : value(formatInteger(number, base, false)) {}
String::String(long long number, unsigned char base) : value(formatSigned(number, base)) {}
String::String(unsigned long long number, unsigned char base) : value(formatInteger(number, base, false)) {}
String::String(float number, unsigned int decimals) : value(formatFloat(number, decimals)) {}
String::String(double number, unsigned int decimals) : value(formatFloat(number, decimals)) {}

bool String::equalsIgnoreCase(const String &other) const {
  if (value.size() != other.value.size()) return false;
  for (size_t i = 0; i < value.size(); ++i) {
    if (tolower((unsigned char)value[i]) != tolower((unsigned char)other.value[i])) return false;
  }
  return true;
}

bool String::startsWith(const String &prefix, unsigned int offset) const {
  return offset <= value.size() && value.compare(offset, prefix.value.size(), prefix.value) == 0;
}

bool String::endsWith(const String &suffix) const {
  return value.size() >= suffix.value.size()
         && value.compare(value.size() - suffix.value.size(), suffix.value.size(), suffix.value) == 0;
}

char &String::operator[](unsigned int index) {
  if (index >= value.size()) {
    dummy = 0;
    return dummy;
  }
  return value[index];
}

void String::getBytes(unsigned char *buffer, unsigned int size, unsigned int index) const {
  if (!size || !buffer) return;
  if (index >= value.size()) {
    buffer[0] = 0;
    return;
  }
  unsigned int n = value.size() - index;
  if (n > size - 1) n = size - 1;
  memcpy(buffer, value.data() + index, n);
  buffer[n] = 0;
}

int String::indexOf(char c, unsigned int from) const {
  const size_t position = value.find(c, from);
  return position == std::string::npos ? -1 : (int)position;
}

int String::indexOf(const String &text, unsigned int from) const {
  const size_t position = value.find(text.value, from);
  return position == std::string::npos ? -1 : (int)position;
}

int String::lastIndexOf(char c) const {
  const size_t position = value.rfind(c);
  return position == std::string::npos ? -1 : (int)position;
}

int String::lastIndexOf(const String &text) const {
  const size_t position = value.rfind(text.value);
  return position == std::string::npos ? -1 : (int)position;
}

String String::substring(unsigned int from, unsigned int to) const {
  if (from > to) {
    const unsigned int swap = from;
    from = to;
    to = swap;
  }
  if (from >= value.size()) return String();
  if (to > value.size()) to = value.size();
  return String(value.substr(from, to - from));
}

void String::replace(char find, char replacement) {
  for (char &c : value) {
    if (c == find) c = replacement;
  }
}

void String::replace(const String &find, const String &replacement) {
  if (find.value.empty()) return;
  size_t position = 0;
  while ((position = value.find(find.value, position)) != std::string::npos) {
    value.replace(position, find.value.size(), replacement.value);
    position += replacement.value.size();
  }
}

void String::remove(unsigned int index, unsigned int count) {
  if (index >= value.size()) return;
  value.erase(index, count);
}

void String::toLowerCase() {
  for (char &c : value) c = tolower((unsigned char)c);
}

void String::toUpperCase() {
  for (char &c : value) c = toupper((unsigned char)c);
}

void String::trim() {
  size_t begin = 0;
  while (begin < value.size() && isspace((unsigned char)value[begin])) ++begin;
  size_t end = value.size();
  while (end > begin && isspace((unsigned char)value[end - 1])) --end;
  value = value.substr(begin, end - begin);
}

long String::toInt() const { return atol(value.c_str()); }
float String::toFloat() const { return (float)atof(value.c_str()); }
double String::toDouble() const { return atof(value.c_str()); }

String operator+(const String &a, const String &b) {
  String result(a);
  result.concat(b);
  return result;
}
String operator+(const String &a, const char *b) {
  String result(a);
  result.concat(b);
  return result;
}
String operator+(const char *a, const String &b) {
  String result(a);
  result.concat(b);
  return result;
}
String operator+(const String &a, char b) {
  String result(a);
  result.concat(b);
  return result;
}
String operator+(const String &a, int b) { return a + String(b); }
String operator+(const String &a, unsigned int b) { return a + String(b); }
String operator+(const String &a, long b) { return a + String(b); }
String operator+(const String &a, unsigned long b) { return a + String(b); }
String operator+(const String &a, float b) { return a + String(b); }
String operator+(const String &a, double b) { return a + String(b); }
