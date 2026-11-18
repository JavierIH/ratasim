// The ESP32's SPI (arduino-esp32's SPIClass): each byte goes to the device whose chip select is low (gchip.cpp) and
// takes its 8 clock periods.
#pragma once
#include "Arduino.h"

#define FSPI 1
#define HSPI 2
#define VSPI 3
#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3
#define SPI_LSBFIRST 0
#define SPI_MSBFIRST 1

class SPISettings {
 public:
  SPISettings(uint32_t clock = 1000000, uint8_t bit_order = SPI_MSBFIRST, uint8_t data_mode = SPI_MODE0)
      : clock(clock), bit_order(bit_order), data_mode(data_mode) {}
  uint32_t clock;
  uint8_t bit_order, data_mode;
};

class SPIClass {
 public:
  explicit SPIClass(uint8_t bus = HSPI) : bus(bus) {}
  void begin(int8_t sck = -1, int8_t miso = -1, int8_t mosi = -1, int8_t ss = -1) { (void)sck, (void)miso, (void)mosi, (void)ss; }
  void end(void) {}
  void beginTransaction(SPISettings settings);
  void endTransaction(void) {}
  void setFrequency(uint32_t freq) { clock = freq; }
  uint8_t transfer(uint8_t data);
  void transfer(void *data, uint32_t size) {
    uint8_t *b = (uint8_t *)data;
    for (uint32_t i = 0; i < size; i++) b[i] = transfer(b[i]);
  }

 private:
  uint8_t bus;
  uint32_t clock = 1000000;
};
extern SPIClass SPI;
