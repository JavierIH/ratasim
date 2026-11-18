// I2C: nothing on Gamusino's bus (its LSM6DSR is on SPI); here for the LSM6DSR library's I2C constructor.
#pragma once
#include "Arduino.h"

class TwoWire {
 public:
  void begin(void) {}
  void beginTransmission(uint8_t address) { (void)address; }
  uint8_t endTransmission(bool stop = true) { (void)stop; return 2; }   // address not acknowledged
  size_t write(uint8_t data) { (void)data; return 1; }
  uint8_t requestFrom(uint8_t address, uint8_t n, bool stop = true) { (void)address, (void)n, (void)stop; return 0; }
  int available(void) { return 0; }
  int read(void) { return -1; }
};
extern TwoWire Wire;
