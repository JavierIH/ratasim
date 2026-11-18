// The Arduino core for the ESP32 (arduino-esp32 2.x, the one its maker builds with), simulated on the host: what
// Gamusino's sketch and its libraries use. The chip behind it: gchip.cpp.
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <cmath>

using std::max;
using std::min;

typedef bool boolean;
typedef uint8_t byte;

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x01
#define OUTPUT 0x03
#define PULLUP 0x04
#define INPUT_PULLUP 0x05
#define PULLDOWN 0x08
#define INPUT_PULLDOWN 0x09
#define LSBFIRST 0
#define MSBFIRST 1
#define CHANGE 0x03
#define FALLING 0x02
#define RISING 0x01

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define IRAM_ATTR
#define digitalPinToInterrupt(p) (p)

// Time: simulated, advanced by the sketch's own waits and by what its calls cost (gchip.cpp)
unsigned long millis(void);
unsigned long micros(void);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
void yield(void);

// Pins, the ADC (12 bits), the LEDC PWM
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
uint16_t analogRead(uint8_t pin);
void analogReadResolution(uint8_t bits);
double ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
void ledcAttachPin(uint8_t pin, uint8_t channel);
void ledcDetachPin(uint8_t pin);
void ledcWrite(uint8_t channel, uint32_t duty);
void attachInterrupt(uint8_t pin, void (*isr)(void), int mode);
void detachInterrupt(uint8_t pin);

// Serial: Arduino's Print, enough of it
class Print {
 public:
  virtual size_t write(uint8_t c) = 0;
  size_t write(const char *s) { return write((const uint8_t *)s, strlen(s)); }
  size_t write(const uint8_t *b, size_t n) {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
  size_t print(const char *s) { return write(s); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(int v, int base = 10) { return print((long)v, base); }
  size_t print(unsigned v, int base = 10) { return print((unsigned long)v, base); }
  size_t print(long v, int base = 10);
  size_t print(unsigned long v, int base = 10);
  size_t print(long long v, int base = 10) { return print((long)v, base); }
  size_t print(double v, int digits = 2);
  size_t println(void) { return write("\r\n"); }
  template <typename T>
  size_t println(T v) {
    size_t n = print(v);
    return n + println();
  }
  template <typename T>
  size_t println(T v, int f) {
    size_t n = print(v, f);
    return n + println();
  }
  size_t printf(const char *fmt, ...);
};

class HardwareSerial : public Print {
 public:
  void begin(unsigned long baud, ...) { (void)baud; }
  void end(void) {}
  int available(void);
  int read(void);
  int peek(void);
  void flush(void) {}
  size_t write(uint8_t c) override;
  using Print::write;
  operator bool() const { return true; }
};
extern HardwareSerial Serial;

