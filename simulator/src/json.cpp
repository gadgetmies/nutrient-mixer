#include "json.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const Json nullJson;
static const std::string emptyString;

const std::string &Json::asString() const { return type == STRING ? text : emptyString; }

std::string Json::toText() const { return type == STRING ? text : dump(); }

const Json &Json::operator[](size_t index) const {
  if (type != ARRAY || index >= items.size()) return nullJson;
  return items[index];
}

void Json::push(const Json &value) {
  if (type != ARRAY) {
    *this = array();
  }
  items.push_back(value);
}

bool Json::has(const std::string &key) const {
  if (type != OBJECT) return false;
  for (const auto &member : members) {
    if (member.first == key) return true;
  }
  return false;
}

const Json &Json::operator[](const std::string &key) const {
  if (type != OBJECT) return nullJson;
  for (const auto &member : members) {
    if (member.first == key) return member.second;
  }
  return nullJson;
}

Json &Json::set(const std::string &key, const Json &value) {
  if (type != OBJECT) *this = object();
  for (auto &member : members) {
    if (member.first == key) {
      member.second = value;
      return *this;
    }
  }
  members.emplace_back(key, value);
  return *this;
}

const Json &Json::path(const std::string &dottedPath) const {
  const Json *current = this;
  size_t start = 0;
  while (start <= dottedPath.size()) {
    size_t end = dottedPath.find('.', start);
    if (end == std::string::npos) end = dottedPath.size();
    const std::string part = dottedPath.substr(start, end - start);
    if (!part.empty()) {
      if (current->isArray()) {
        char *rest;
        const long index = strtol(part.c_str(), &rest, 10);
        if (*rest) return nullJson;
        current = &(*current)[(size_t)index];
      } else {
        current = &(*current)[part];
      }
    }
    start = end + 1;
  }
  return *current;
}

void Json::escape(const std::string &value, std::string &out) {
  out += '"';
  for (unsigned char c : value) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (c < 0x20) {
          char buffer[8];
          snprintf(buffer, sizeof(buffer), "\\u%04x", c);
          out += buffer;
        } else {
          out += (char)c;
        }
    }
  }
  out += '"';
}

void Json::dump(std::string &out) const {
  switch (type) {
    case NUL:
      out += "null";
      break;
    case BOOL:
      out += boolean ? "true" : "false";
      break;
    case NUMBER: {
      if (!isfinite(number)) {
        out += "null";
      } else if (number == (double)(long long)number && fabs(number) < 1e15) {
        out += std::to_string((long long)number);
      } else {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%.10g", number);
        out += buffer;
      }
      break;
    }
    case STRING:
      escape(text, out);
      break;
    case ARRAY:
      out += '[';
      for (size_t i = 0; i < items.size(); ++i) {
        if (i) out += ',';
        items[i].dump(out);
      }
      out += ']';
      break;
    case OBJECT:
      out += '{';
      for (size_t i = 0; i < members.size(); ++i) {
        if (i) out += ',';
        escape(members[i].first, out);
        out += ':';
        members[i].second.dump(out);
      }
      out += '}';
      break;
  }
}

std::string Json::dump() const {
  std::string out;
  dump(out);
  return out;
}

namespace {

struct Parser {
  const std::string &s;
  size_t i = 0;
  std::string error;

  explicit Parser(const std::string &input) : s(input) {}

  void skipSpace() {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) ++i;
  }

  bool fail(const char *message) {
    if (error.empty()) error = std::string(message) + " at offset " + std::to_string(i);
    return false;
  }

  bool literal(const char *word) {
    const size_t n = strlen(word);
    if (s.compare(i, n, word) != 0) return fail("invalid literal");
    i += n;
    return true;
  }

  static void appendUtf8(std::string &out, unsigned code) {
    if (code < 0x80) {
      out += (char)code;
    } else if (code < 0x800) {
      out += (char)(0xC0 | (code >> 6));
      out += (char)(0x80 | (code & 0x3F));
    } else if (code < 0x10000) {
      out += (char)(0xE0 | (code >> 12));
      out += (char)(0x80 | ((code >> 6) & 0x3F));
      out += (char)(0x80 | (code & 0x3F));
    } else {
      out += (char)(0xF0 | (code >> 18));
      out += (char)(0x80 | ((code >> 12) & 0x3F));
      out += (char)(0x80 | ((code >> 6) & 0x3F));
      out += (char)(0x80 | (code & 0x3F));
    }
  }

  bool parseString(std::string &out) {
    if (s[i] != '"') return fail("expected string");
    ++i;
    while (i < s.size() && s[i] != '"') {
      char c = s[i++];
      if (c != '\\') {
        out += c;
        continue;
      }
      if (i >= s.size()) return fail("unterminated escape");
      c = s[i++];
      switch (c) {
        case 'n':
          out += '\n';
          break;
        case 'r':
          out += '\r';
          break;
        case 't':
          out += '\t';
          break;
        case 'b':
          out += '\b';
          break;
        case 'f':
          out += '\f';
          break;
        case 'u': {
          if (i + 4 > s.size()) return fail("bad unicode escape");
          unsigned code = strtoul(s.substr(i, 4).c_str(), nullptr, 16);
          i += 4;
          if (code >= 0xD800 && code < 0xDC00 && s.compare(i, 2, "\\u") == 0 && i + 6 <= s.size()) {
            const unsigned low = strtoul(s.substr(i + 2, 4).c_str(), nullptr, 16);
            if (low >= 0xDC00 && low < 0xE000) {
              code = 0x10000 + ((code - 0xD800) << 10) + (low - 0xDC00);
              i += 6;
            }
          }
          appendUtf8(out, code);
          break;
        }
        default:
          out += c;
      }
    }
    if (i >= s.size()) return fail("unterminated string");
    ++i;
    return true;
  }

  bool parseValue(Json &out, int depth) {
    if (depth > 64) return fail("nesting too deep");
    skipSpace();
    if (i >= s.size()) return fail("unexpected end");
    const char c = s[i];
    if (c == '{') {
      ++i;
      out = Json::object();
      skipSpace();
      if (i < s.size() && s[i] == '}') {
        ++i;
        return true;
      }
      while (true) {
        skipSpace();
        std::string key;
        if (i >= s.size() || !parseString(key)) return fail("expected key");
        skipSpace();
        if (i >= s.size() || s[i] != ':') return fail("expected ':'");
        ++i;
        Json value;
        if (!parseValue(value, depth + 1)) return false;
        out.set(key, value);
        skipSpace();
        if (i < s.size() && s[i] == ',') {
          ++i;
          continue;
        }
        if (i < s.size() && s[i] == '}') {
          ++i;
          return true;
        }
        return fail("expected ',' or '}'");
      }
    }
    if (c == '[') {
      ++i;
      out = Json::array();
      skipSpace();
      if (i < s.size() && s[i] == ']') {
        ++i;
        return true;
      }
      while (true) {
        Json value;
        if (!parseValue(value, depth + 1)) return false;
        out.push(value);
        skipSpace();
        if (i < s.size() && s[i] == ',') {
          ++i;
          continue;
        }
        if (i < s.size() && s[i] == ']') {
          ++i;
          return true;
        }
        return fail("expected ',' or ']'");
      }
    }
    if (c == '"') {
      std::string text;
      if (!parseString(text)) return false;
      out = Json(text);
      return true;
    }
    if (c == 't') {
      out = Json(true);
      return literal("true");
    }
    if (c == 'f') {
      out = Json(false);
      return literal("false");
    }
    if (c == 'n') {
      out = Json();
      return literal("null");
    }
    const char *start = s.c_str() + i;
    char *end;
    const double number = strtod(start, &end);
    if (end == start) return fail("unexpected character");
    i += end - start;
    out = Json(number);
    return true;
  }
};

}  // namespace

bool Json::parse(const std::string &input, Json &result, std::string *error) {
  Parser parser(input);
  Json value;
  if (!parser.parseValue(value, 0)) {
    if (error) *error = parser.error;
    return false;
  }
  parser.skipSpace();
  if (parser.i != input.size()) {
    if (error) *error = "trailing characters at offset " + std::to_string(parser.i);
    return false;
  }
  result = value;
  return true;
}
