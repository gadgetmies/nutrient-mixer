#pragma once

#include "Stream.h"

class HardwareSerial : public Stream {
 public:
  void begin(unsigned long baud, uint32_t config = 0, int8_t rxPin = -1, int8_t txPin = -1) { baudRate = baud; }
  void end() {}
  int available() override;
  int read() override;
  int peek() override;
  void flush() override {}
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  using Print::write;
  int availableForWrite() override { return 128; }
  operator bool() const { return true; }
  unsigned long baudRate = 0;
};

extern HardwareSerial Serial;
