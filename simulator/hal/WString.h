// Arduino String class (subset of the Arduino core API that sketches and libraries commonly use)
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>

class __FlashStringHelper;

class String {
 public:
  String() {}
  String(const char *text) : value(text ? text : "") {}
  String(const String &other) = default;
  String(String &&other) = default;
  String(const __FlashStringHelper *text) : value(reinterpret_cast<const char *>(text)) {}
  String(const std::string &text) : value(text) {}
  explicit String(char c) : value(1, c) {}
  explicit String(unsigned char number, unsigned char base = 10);
  explicit String(int number, unsigned char base = 10);
  explicit String(unsigned int number, unsigned char base = 10);
  explicit String(long number, unsigned char base = 10);
  explicit String(unsigned long number, unsigned char base = 10);
  explicit String(long long number, unsigned char base = 10);
  explicit String(unsigned long long number, unsigned char base = 10);
  explicit String(float number, unsigned int decimals = 2);
  explicit String(double number, unsigned int decimals = 2);

  String &operator=(const String &other) = default;
  String &operator=(String &&other) = default;
  String &operator=(const char *text) {
    value = text ? text : "";
    return *this;
  }

  unsigned int length() const { return value.size(); }
  bool isEmpty() const { return value.empty(); }
  const char *c_str() const { return value.c_str(); }
  bool reserve(unsigned int size) {
    value.reserve(size);
    return true;
  }

  bool concat(const String &other) {
    value += other.value;
    return true;
  }
  bool concat(const char *text) {
    if (text) value += text;
    return true;
  }
  bool concat(char c) {
    value += c;
    return true;
  }
  bool concat(unsigned char number) { return concat(String(number)); }
  bool concat(int number) { return concat(String(number)); }
  bool concat(unsigned int number) { return concat(String(number)); }
  bool concat(long number) { return concat(String(number)); }
  bool concat(unsigned long number) { return concat(String(number)); }
  bool concat(float number) { return concat(String(number)); }
  bool concat(double number) { return concat(String(number)); }

  template <typename T>
  String &operator+=(const T &other) {
    concat(other);
    return *this;
  }

  int compareTo(const String &other) const { return value.compare(other.value); }
  bool equals(const String &other) const { return value == other.value; }
  bool equals(const char *text) const { return value == (text ? text : ""); }
  bool equalsIgnoreCase(const String &other) const;
  bool startsWith(const String &prefix, unsigned int offset = 0) const;
  bool endsWith(const String &suffix) const;
  bool operator==(const String &other) const { return equals(other); }
  bool operator==(const char *text) const { return equals(text); }
  bool operator!=(const String &other) const { return !equals(other); }
  bool operator!=(const char *text) const { return !equals(text); }
  bool operator<(const String &other) const { return compareTo(other) < 0; }
  bool operator>(const String &other) const { return compareTo(other) > 0; }
  bool operator<=(const String &other) const { return compareTo(other) <= 0; }
  bool operator>=(const String &other) const { return compareTo(other) >= 0; }

  char charAt(unsigned int index) const { return index < value.size() ? value[index] : 0; }
  void setCharAt(unsigned int index, char c) {
    if (index < value.size()) value[index] = c;
  }
  char operator[](unsigned int index) const { return charAt(index); }
  char &operator[](unsigned int index);
  void getBytes(unsigned char *buffer, unsigned int size, unsigned int index = 0) const;
  void toCharArray(char *buffer, unsigned int size, unsigned int index = 0) const {
    getBytes(reinterpret_cast<unsigned char *>(buffer), size, index);
  }

  int indexOf(char c, unsigned int from = 0) const;
  int indexOf(const String &text, unsigned int from = 0) const;
  int lastIndexOf(char c) const;
  int lastIndexOf(const String &text) const;
  String substring(unsigned int from) const { return substring(from, value.size()); }
  String substring(unsigned int from, unsigned int to) const;

  void replace(char find, char replacement);
  void replace(const String &find, const String &replacement);
  void remove(unsigned int index) { remove(index, value.size()); }
  void remove(unsigned int index, unsigned int count);
  void toLowerCase();
  void toUpperCase();
  void trim();

  long toInt() const;
  float toFloat() const;
  double toDouble() const;

  const std::string &str() const { return value; }

 private:
  std::string value;
  char dummy = 0;
};

String operator+(const String &a, const String &b);
String operator+(const String &a, const char *b);
String operator+(const char *a, const String &b);
String operator+(const String &a, char b);
String operator+(const String &a, int b);
String operator+(const String &a, unsigned int b);
String operator+(const String &a, long b);
String operator+(const String &a, unsigned long b);
String operator+(const String &a, float b);
String operator+(const String &a, double b);
