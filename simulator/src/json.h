// Minimal JSON value with a parser and a serializer. Used for the UI protocol, saved state and scenario checks.
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

class Json {
 public:
  enum Type { NUL, BOOL, NUMBER, STRING, ARRAY, OBJECT };

  Json() {}
  Json(std::nullptr_t) {}
  Json(bool value) : type(BOOL), boolean(value) {}
  Json(int value) : type(NUMBER), number(value) {}
  Json(unsigned value) : type(NUMBER), number(value) {}
  Json(long value) : type(NUMBER), number(value) {}
  Json(unsigned long value) : type(NUMBER), number(value) {}
  Json(long long value) : type(NUMBER), number(value) {}
  Json(unsigned long long value) : type(NUMBER), number(value) {}
  Json(double value) : type(NUMBER), number(value) {}
  Json(const char *value) : type(STRING), text(value) {}
  Json(const std::string &value) : type(STRING), text(value) {}

  static Json array() {
    Json j;
    j.type = ARRAY;
    return j;
  }
  static Json object() {
    Json j;
    j.type = OBJECT;
    return j;
  }

  // Parses text; returns false (and sets error) on malformed input
  static bool parse(const std::string &input, Json &result, std::string *error = nullptr);
  std::string dump() const;
  void dump(std::string &out) const;

  Type getType() const { return type; }
  bool isNull() const { return type == NUL; }
  bool isBool() const { return type == BOOL; }
  bool isNumber() const { return type == NUMBER; }
  bool isString() const { return type == STRING; }
  bool isArray() const { return type == ARRAY; }
  bool isObject() const { return type == OBJECT; }

  bool asBool(bool fallback = false) const { return type == BOOL ? boolean : (type == NUMBER ? number != 0 : fallback); }
  double asNumber(double fallback = 0) const { return type == NUMBER ? number : (type == BOOL ? boolean : fallback); }
  const std::string &asString() const;
  std::string toText() const;  // Strings as is, other values serialized

  // Arrays
  size_t size() const { return type == ARRAY ? items.size() : (type == OBJECT ? members.size() : 0); }
  const Json &operator[](size_t index) const;
  void push(const Json &value);
  const std::vector<Json> &arrayItems() const { return items; }

  // Objects (members keep their insertion order)
  bool has(const std::string &key) const;
  const Json &operator[](const std::string &key) const;
  const Json &operator[](const char *key) const { return (*this)[std::string(key)]; }
  Json &set(const std::string &key, const Json &value);
  const std::vector<std::pair<std::string, Json>> &objectMembers() const { return members; }

  // Looks up a dotted path such as "tanks.mixer.volume" or "pumps.1.flow"
  const Json &path(const std::string &dottedPath) const;

  static void escape(const std::string &text, std::string &out);

 private:
  Type type = NUL;
  bool boolean = false;
  double number = 0;
  std::string text;
  std::vector<Json> items;
  std::vector<std::pair<std::string, Json>> members;
};
