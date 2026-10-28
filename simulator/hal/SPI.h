#pragma once

#include "Arduino.h"

#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

class SPISettings {
 public:
  SPISettings(uint32_t clock = 1000000, uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE0)
      : clock(clock), bitOrder(bitOrder), dataMode(dataMode) {}
  uint32_t clock;
  uint8_t bitOrder;
  uint8_t dataMode;
};

// The pins given to begin() are only recorded; the display model reads the transfers from Adafruit_ST7789.
class SPIClass {
 public:
  void begin(int8_t sck = -1, int8_t miso = -1, int8_t mosi = -1, int8_t ss = -1) {
    sckPin = sck;
    misoPin = miso;
    mosiPin = mosi;
    ssPin = ss;
    started = true;
  }
  void end() { started = false; }
  void beginTransaction(SPISettings) {}
  void endTransaction() {}
  uint8_t transfer(uint8_t) { return 0; }

  int8_t sckPin = -1, misoPin = -1, mosiPin = -1, ssPin = -1;
  bool started = false;
};

extern SPIClass SPI;
