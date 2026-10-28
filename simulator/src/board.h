// The electronics: ESP32 GPIO of the T-Display and everything wired to it on the mixer board
// (eagle/untitled.sch): the 74HC595 shift register, two L298N motor drivers, two DR21A01 relay boards, the
// PH-4502C pH and Keyestudio TDS boards (powered through BSS123 low-side switches), the EC12E rotary encoder,
// the abort button, the float switches and the LEDs.
#pragma once

#include <stdint.h>

#include <random>
#include <string>

#include "json.h"
#include "params.h"
#include "world.h"

namespace sim {

// ESP32 pins on the board (schematic net names)
namespace pins {
const uint8_t BUTTON_IO0 = 0;   // T-Display left button
const uint8_t BUTTON_IO35 = 35;  // T-Display right button
const uint8_t TFT_BL = 4;
const uint8_t ADC_EN = 14;
const uint8_t SR_SER = 2;
const uint8_t SR_SCK = 15;
const uint8_t SR_RCK = 33;
const uint8_t SR_SCL = 25;  // !SRCLR
const uint8_t ABORT_SW = 13;
const uint8_t LED_ERROR = 12;
const uint8_t ENC_A = 17;
const uint8_t ENC_B = 22;
const uint8_t ENC_SW = 21;
const uint8_t RELAY1 = 27;
const uint8_t RELAY2 = 26;
const uint8_t TDS3V = 32;
const uint8_t PH3V = 36;
const uint8_t TEMP3V = 39;
const uint8_t FLOAT_RES = 37;
const uint8_t FLOAT_MIXER = 38;
}  // namespace pins

// 74HC595 outputs (QA = bit 0 ... QH = bit 7)
namespace sr {
const int PH_ON = 0;      // QA
const int PPUMP1_EN = 1;  // QB
const int PPUMP2_EN = 2;  // QC
const int PPUMP4_EN = 3;  // QD
const int PPUMP_B = 4;    // QE
const int PPUMP_A = 5;    // QF
const int PPUMP3_EN = 6;  // QG
const int TDS_ON = 7;     // QH
}  // namespace sr

enum Button { IO0, IO35, ENCODER_BUTTON, ABORT_BUTTON, BUTTON_COUNT };

struct Pin {
  uint8_t mode = 0;  // 0 = not configured (input, no pulls)
  uint8_t out = 0;
  int level = 0;
  void (*isr)() = nullptr;
  int isrMode = 0;
};

// Sensor board on the probe power switch (BSS123 between the board's GND and the main ground)
struct SensorBoard {
  bool powered = false;
  double poweredAt = 0;   // Seconds
  double output = 0;      // Settled output voltage (mV) for the current solution
  double settled = 0;     // Output including the warm-up, mV at the board output
};

class Board {
 public:
  Board(Params &params, World &world);
  void powerOn();

  // GPIO (firmware side)
  void pinMode(uint8_t pin, uint8_t mode);
  void digitalWrite(uint8_t pin, uint8_t level);
  int digitalRead(uint8_t pin);
  uint32_t analogMilliVolts(uint8_t pin);
  int analogRaw(uint8_t pin);
  void attachInterrupt(uint8_t pin, void (*handler)(), int mode);
  void detachInterrupt(uint8_t pin);

  // Simulator side
  void setButton(Button button, bool pressed);
  bool buttonPressed(Button button) const { return buttons[button]; }
  void setEncoderContacts(bool aClosed, bool bClosed);
  void update(double now);  // Called after each world step: float switches and sensor warm up
  Actuators actuators() const;

  uint8_t shiftRegister() const { return srLatch; }
  bool srOutput(int bit) const { return (srLatch >> bit) & 1; }
  bool relayEnergized(int relay) const;  // 0 = relay 1 (GPIO27), 1 = relay 2 (GPIO26)
  int l298nOutput(int driver, int channel) const;  // -1, 0, 1
  int pinLevel(uint8_t pin) const { return pin < 40 ? pinState[pin].level : 0; }

  Json snapshot() const;

  double now = 0;  // Seconds since power on (virtual time)

 private:
  void updateInput(uint8_t pin);
  int resolveLevel(uint8_t pin) const;
  void shiftRegisterPinChanged(uint8_t pin, int level);
  double sensorMilliVolts(uint8_t pin);
  void updateSensorBoard(SensorBoard &board, bool powered, double target);
  double noise(double sigma);
  bool motorsRunning() const;

  Params &params;
  World &world;
  Pin pinState[40];
  bool buttons[BUTTON_COUNT] = {false, false, false, false};
  bool encoderA = false;  // Contact closed (pulls the pin low)
  bool encoderB = false;

  uint8_t srShift = 0;
  uint8_t srLatch = 0;
  int srClockLevel = 0;
  int srLatchLevel = 0;

  SensorBoard phBoard;
  SensorBoard tdsBoard;
  double phDrift = 0;  // Random walk of a disconnected or dry pH probe

  std::mt19937 random;
  std::normal_distribution<double> gaussian{0, 1};
  std::uniform_real_distribution<double> uniform{0, 1};
};

}  // namespace sim
