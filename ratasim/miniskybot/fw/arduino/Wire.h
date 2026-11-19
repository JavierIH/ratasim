// Arduino's Wire library (the ATmega328P's TWI, I2C at 100 kHz on A4 and A5), simulated: its bus has the SRF02 on it
// (mchip.cpp). Each byte on the bus takes its time (9 clocks).
#pragma once
#include "Arduino.h"

class TwoWire {
 public:
  void begin(void);
  void setClock(uint32_t hz);
  void beginTransmission(uint8_t address);
  void beginTransmission(int address) { beginTransmission((uint8_t)address); }
  uint8_t endTransmission(bool stop = true);
  size_t write(uint8_t data);
  uint8_t requestFrom(uint8_t address, uint8_t quantity);
  uint8_t requestFrom(int address, int quantity) { return requestFrom((uint8_t)address, (uint8_t)quantity); }
  int available(void);
  int read(void);
};
extern TwoWire Wire;
