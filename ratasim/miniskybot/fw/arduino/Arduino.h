// The Arduino core for the ATmega328P (ArduinoCore-avr 1.8) simulated on the host: what Miniskybot's program uses
// on its Skymega board. The chip behind it: mchip.cpp. As on the chip, min() and max() are macros. Not as on it: int
// is the host's (32 bits; the program keeps to long for its times, as it would on the chip).
#pragma once
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef bool boolean;
typedef uint8_t byte;

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#define PI 3.1415926535897932384626433832795
#ifdef abs
#undef abs
#endif
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))

// The ATmega328P's pins as Arduino numbers them: D0..D13, A0..A5 (14..19)
#define LED_BUILTIN 13
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

// Time: simulated, advanced by the program's waits and by what its calls cost (mchip.cpp)
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
long map(long x, long in_min, long in_max, long out_min, long out_max);

class Print {
 public:
  virtual size_t write(uint8_t c) = 0;
  size_t write(const char *s) {
    size_t n = 0;
    while (*s) n += write((uint8_t)*s++);
    return n;
  }
  size_t print(const char *s) { return write(s); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(int v, int base = DEC) { return print((long)v, base); }
  size_t print(unsigned v, int base = DEC) { return print((unsigned long)v, base); }
  size_t print(long v, int base = DEC);
  size_t print(unsigned long v, int base = DEC);
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
};

class HardwareSerial : public Print {
 public:
  void begin(unsigned long baud);
  int available(void);
  int read(void);
  void flush(void);
  size_t write(uint8_t c) override;
  using Print::write;
  operator bool() const { return true; }
};
extern HardwareSerial Serial;
