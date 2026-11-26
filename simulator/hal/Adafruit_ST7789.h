// ST7789 display of the T-Display. Drawing goes into the simulated panel memory (135 x 240 RGB565, portrait).
// Each operation spends the time its SPI transfer takes, so screen redraws cost as much time as on the device.
#pragma once

#include "Adafruit_ST77xx.h"
#include "SPI.h"

class Adafruit_ST7789 : public Adafruit_GFX {
 public:
  Adafruit_ST7789(int8_t cs, int8_t dc, int8_t rst);
  Adafruit_ST7789(int8_t cs, int8_t dc, int8_t mosi, int8_t sclk, int8_t rst = -1);
  Adafruit_ST7789(SPIClass *spi, int8_t cs, int8_t dc, int8_t rst);

  void init(uint16_t width = 240, uint16_t height = 240, uint8_t spiMode = SPI_MODE0);
  void setSPISpeed(uint32_t frequency);
  void setRotation(uint8_t rotation) override;
  void enableDisplay(bool enable);
  void enableSleep(bool enable);
  void invertDisplay(bool invert) override;

  void drawPixel(int16_t x, int16_t y, uint16_t color) override;
  void writePixel(int16_t x, int16_t y, uint16_t color) override;
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override;
  void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override;
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override;
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override;
  void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override;
  void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override;
  void fillScreen(uint16_t color) override;
  void startWrite() override;
  void endWrite() override;

  uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
  }

 private:
  void fill(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void spendTransfer(uint32_t pixels);

  bool initialized = false;
  uint32_t spiFrequency = 24000000;
  int writeDepth = 0;
};
