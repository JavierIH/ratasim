// Adafruit NeoPixel's API (its ESP32 driver sends through the RMT peripheral): the colours shown go to the simulator
// (gchip.cpp), and show() takes the time of the bits on the wire (800 kHz, 24 bits a LED, then the reset).
#pragma once
#include "Arduino.h"

#define NEO_GRB ((1 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_RGB ((0 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_KHZ800 0x0000
#define NEO_KHZ400 0x0100

class Adafruit_NeoPixel {
 public:
  Adafruit_NeoPixel(uint16_t n, int16_t pin = 6, uint16_t type = NEO_GRB + NEO_KHZ800);
  ~Adafruit_NeoPixel();
  void begin(void) {}
  void show(void);
  void setPin(int16_t p) { pin = p; }
  void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
  void setPixelColor(uint16_t n, uint32_t c) { setPixelColor(n, (uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c); }
  void fill(uint32_t c = 0, uint16_t first = 0, uint16_t count = 0);
  void setBrightness(uint8_t b) { brightness = b; }
  void clear(void);
  uint16_t numPixels(void) const { return n; }
  uint8_t getBrightness(void) const { return brightness; }
  uint32_t getPixelColor(uint16_t i) const;
  static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) { return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b; }

 private:
  uint16_t n;
  int16_t pin;
  uint8_t brightness = 255;
  uint8_t *rgb;
};
