#include "Adafruit_ST7789.h"

#include "sim_api.h"

SPIClass SPI;

// The library is constructed with the full controller size; init() sets the actual panel size
Adafruit_ST7789::Adafruit_ST7789(int8_t, int8_t, int8_t) : Adafruit_GFX(240, 320) {}
Adafruit_ST7789::Adafruit_ST7789(int8_t, int8_t, int8_t, int8_t, int8_t) : Adafruit_GFX(240, 320) {}
Adafruit_ST7789::Adafruit_ST7789(SPIClass *, int8_t, int8_t, int8_t) : Adafruit_GFX(240, 320) {}

void Adafruit_ST7789::init(uint16_t width, uint16_t height, uint8_t) {
  // The T-Display panel is 135 x 240. Other sizes would show a part of the 240 x 320 controller memory, which
  // the simulator does not model.
  WIDTH = width;
  HEIGHT = height;
  delay(150 + 10 + 10 + 10);  // Reset, sleep out and display on delays of the Adafruit init sequence
  initialized = true;
  setRotation(0);
}

void Adafruit_ST7789::setSPISpeed(uint32_t frequency) { spiFrequency = frequency ? frequency : 1; }

void Adafruit_ST7789::setRotation(uint8_t m) {
  rotation = m & 3;
  if (rotation & 1) {
    _width = HEIGHT;
    _height = WIDTH;
  } else {
    _width = WIDTH;
    _height = HEIGHT;
  }
  spendTransfer(0);
}

void Adafruit_ST7789::enableDisplay(bool) { spendTransfer(0); }
void Adafruit_ST7789::enableSleep(bool) { spendTransfer(0); }
void Adafruit_ST7789::invertDisplay(bool) { spendTransfer(0); }

// A transfer sets the address window (about 11 bytes of commands and parameters) and sends 16 bits per pixel
void Adafruit_ST7789::spendTransfer(uint32_t pixels) {
  const uint64_t bits = (11 + 2ULL * pixels) * 8;
  simhal::spend(1 + bits * 1000000ULL / spiFrequency);
}

// Logical coordinates (after rotation) to panel memory
void Adafruit_ST7789::fill(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (w < 0) {
    x += w + 1;
    w = -w;
  }
  if (h < 0) {
    y += h + 1;
    h = -h;
  }
  int x0 = x < 0 ? 0 : x;
  int y0 = y < 0 ? 0 : y;
  int x1 = x + w > _width ? _width : x + w;
  int y1 = y + h > _height ? _height : y + h;
  if (x0 >= x1 || y0 >= y1) return;

  spendTransfer((uint32_t)(x1 - x0) * (y1 - y0));
  if (!initialized) return;

  uint16_t *pixels = simhal::panelPixels();
  const int pw = simhal::panelWidth;
  const int ph = simhal::panelHeight;
  for (int ly = y0; ly < y1; ++ly) {
    for (int lx = x0; lx < x1; ++lx) {
      int px, py;
      switch (rotation) {
        case 0:
          px = lx;
          py = ly;
          break;
        case 1:
          px = pw - 1 - ly;
          py = lx;
          break;
        case 2:
          px = pw - 1 - lx;
          py = ph - 1 - ly;
          break;
        default:
          px = ly;
          py = ph - 1 - lx;
          break;
      }
      if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT && px < pw && py < ph) pixels[py * pw + px] = color;
    }
  }
  simhal::panelChanged();
}

void Adafruit_ST7789::drawPixel(int16_t x, int16_t y, uint16_t color) { fill(x, y, 1, 1, color); }
void Adafruit_ST7789::writePixel(int16_t x, int16_t y, uint16_t color) { fill(x, y, 1, 1, color); }
void Adafruit_ST7789::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) { fill(x, y, w, h, color); }
void Adafruit_ST7789::writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) { fill(x, y, w, h, color); }
void Adafruit_ST7789::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) { fill(x, y, w, 1, color); }
void Adafruit_ST7789::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) { fill(x, y, 1, h, color); }
void Adafruit_ST7789::writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) { fill(x, y, w, 1, color); }
void Adafruit_ST7789::writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) { fill(x, y, 1, h, color); }
void Adafruit_ST7789::fillScreen(uint16_t color) { fill(0, 0, _width, _height, color); }
void Adafruit_ST7789::startWrite() { ++writeDepth; }
void Adafruit_ST7789::endWrite() {
  if (writeDepth > 0) --writeDepth;
}
