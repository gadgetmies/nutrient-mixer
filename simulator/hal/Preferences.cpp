#include "Preferences.h"

#include "sim_api.h"

using simhal::NvsType;

bool Preferences::begin(const char *name, bool readOnly_, const char *) {
  if (started || !name || strlen(name) > 15) return false;
  space = name;
  readOnly = readOnly_;
  started = true;
  return true;
}

void Preferences::end() { started = false; }

bool Preferences::clear() {
  if (!started || readOnly) return false;
  simhal::nvsClear(space);
  return true;
}

bool Preferences::remove(const char *key) {
  if (!started || readOnly || !key) return false;
  return simhal::nvsRemove(space, key);
}

bool Preferences::isKey(const char *key) {
  NvsType type;
  std::vector<uint8_t> data;
  return started && key && simhal::nvsGet(space, key, type, data);
}

template <typename T>
size_t Preferences::putValue(const char *key, int type, T value) {
  // NVS keys are at most 15 characters
  if (!started || readOnly || !key || strlen(key) > 15) return 0;
  std::vector<uint8_t> data(sizeof(T));
  memcpy(data.data(), &value, sizeof(T));
  simhal::spend(2000);  // Flash write
  simhal::nvsSet(space, key, (NvsType)type, data);
  return sizeof(T);
}

template <typename T>
T Preferences::getValue(const char *key, int type, T defaultValue) {
  if (!started || !key) return defaultValue;
  NvsType storedType;
  std::vector<uint8_t> data;
  // Like NVS, a value stored with a different type is not found
  if (!simhal::nvsGet(space, key, storedType, data) || storedType != type || data.size() != sizeof(T)) {
    return defaultValue;
  }
  T value;
  memcpy(&value, data.data(), sizeof(T));
  return value;
}

size_t Preferences::putChar(const char *key, int8_t value) { return putValue(key, simhal::NVS_I8, value); }
size_t Preferences::putUChar(const char *key, uint8_t value) { return putValue(key, simhal::NVS_U8, value); }
size_t Preferences::putShort(const char *key, int16_t value) { return putValue(key, simhal::NVS_I16, value); }
size_t Preferences::putUShort(const char *key, uint16_t value) { return putValue(key, simhal::NVS_U16, value); }
size_t Preferences::putInt(const char *key, int32_t value) { return putValue(key, simhal::NVS_I32, value); }
size_t Preferences::putUInt(const char *key, uint32_t value) { return putValue(key, simhal::NVS_U32, value); }
size_t Preferences::putLong64(const char *key, int64_t value) { return putValue(key, simhal::NVS_I64, value); }
size_t Preferences::putULong64(const char *key, uint64_t value) { return putValue(key, simhal::NVS_U64, value); }
// Floats and doubles are stored as blobs by arduino-esp32
size_t Preferences::putFloat(const char *key, float value) { return putBytes(key, &value, sizeof(value)); }
size_t Preferences::putDouble(const char *key, double value) { return putBytes(key, &value, sizeof(value)); }

size_t Preferences::putString(const char *key, const char *value) {
  if (!started || readOnly || !key || !value || strlen(key) > 15) return 0;
  const size_t length = strlen(value);
  std::vector<uint8_t> data(value, value + length);
  simhal::spend(2000);
  simhal::nvsSet(space, key, simhal::NVS_STR, data);
  return length;
}

size_t Preferences::putBytes(const char *key, const void *value, size_t length) {
  if (!started || readOnly || !key || !value || !length || strlen(key) > 15) return 0;
  const uint8_t *bytes = static_cast<const uint8_t *>(value);
  std::vector<uint8_t> data(bytes, bytes + length);
  simhal::spend(2000);
  simhal::nvsSet(space, key, simhal::NVS_BLOB, data);
  return length;
}

int8_t Preferences::getChar(const char *key, int8_t d) { return getValue(key, simhal::NVS_I8, d); }
uint8_t Preferences::getUChar(const char *key, uint8_t d) { return getValue(key, simhal::NVS_U8, d); }
int16_t Preferences::getShort(const char *key, int16_t d) { return getValue(key, simhal::NVS_I16, d); }
uint16_t Preferences::getUShort(const char *key, uint16_t d) { return getValue(key, simhal::NVS_U16, d); }
int32_t Preferences::getInt(const char *key, int32_t d) { return getValue(key, simhal::NVS_I32, d); }
uint32_t Preferences::getUInt(const char *key, uint32_t d) { return getValue(key, simhal::NVS_U32, d); }
int64_t Preferences::getLong64(const char *key, int64_t d) { return getValue(key, simhal::NVS_I64, d); }
uint64_t Preferences::getULong64(const char *key, uint64_t d) { return getValue(key, simhal::NVS_U64, d); }

float Preferences::getFloat(const char *key, float defaultValue) {
  float value;
  return getBytes(key, &value, sizeof(value)) == sizeof(value) ? value : defaultValue;
}

double Preferences::getDouble(const char *key, double defaultValue) {
  double value;
  return getBytes(key, &value, sizeof(value)) == sizeof(value) ? value : defaultValue;
}

size_t Preferences::getString(const char *key, char *value, size_t maxLength) {
  NvsType type;
  std::vector<uint8_t> data;
  if (!started || !key || !value || !simhal::nvsGet(space, key, type, data) || type != simhal::NVS_STR) return 0;
  if (data.size() + 1 > maxLength) return 0;
  memcpy(value, data.data(), data.size());
  value[data.size()] = 0;
  return data.size() + 1;
}

String Preferences::getString(const char *key, String defaultValue) {
  NvsType type;
  std::vector<uint8_t> data;
  if (!started || !key || !simhal::nvsGet(space, key, type, data) || type != simhal::NVS_STR) return defaultValue;
  return String(std::string(data.begin(), data.end()));
}

size_t Preferences::getBytesLength(const char *key) {
  NvsType type;
  std::vector<uint8_t> data;
  if (!started || !key || !simhal::nvsGet(space, key, type, data) || type != simhal::NVS_BLOB) return 0;
  return data.size();
}

size_t Preferences::getBytes(const char *key, void *buffer, size_t maxLength) {
  NvsType type;
  std::vector<uint8_t> data;
  if (!started || !key || !buffer || !simhal::nvsGet(space, key, type, data) || type != simhal::NVS_BLOB) return 0;
  if (data.size() > maxLength) return 0;
  memcpy(buffer, data.data(), data.size());
  return data.size();
}
