// The Arduino core for the ATmega328P (ArduinoCore-avr 1.8, the Nano's), simulated on the host: what
// mazerunner-core uses. The chip behind it: uchip.cpp. As on the chip, min() and max() are macros (the firmware
// mixes ints and floats in them).
#pragma once
#include <math.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avr/interrupt.h"
#include "avr/io.h"
#include "avr/pgmspace.h"

typedef bool boolean;
typedef uint8_t byte;

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define CHANGE 1
#define FALLING 2
#define RISING 3
#define DEFAULT 1
#define EXTERNAL 0
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#ifdef abs
#undef abs
#endif
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))
#define interrupts() sei()
#define noInterrupts() cli()

// The Nano's pins: D0..D13, A0..A7 (14..21); external interrupts INT0, INT1 on D2, D3
#define LED_BUILTIN 13
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21
#define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : -1))

// Time: simulated, advanced by the firmware's waits and by what its calls cost (uchip.cpp)
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void yield(void);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogReference(uint8_t mode);
void analogWrite(uint8_t pin, int val);
void attachInterrupt(uint8_t interrupt, void (*isr)(void), int mode);
void detachInterrupt(uint8_t interrupt);

// WCharacter.h
inline bool isPrintable(int c) { return isprint(c); }

// Flash strings: ordinary ones
class __FlashStringHelper;
#define F(s) (s)

// avr-libc's stdio streams (config-ukmarsbot.h's redirectPrintf(), which the firmware does not call)
#define _FDEV_SETUP_WRITE 2
#define fdev_setup_stream(stream, p, g, f) ((void)(stream), (void)(p))

class Print {
 public:
  virtual size_t write(uint8_t c) = 0;
  size_t write(const char *s) { return write((const uint8_t *)s, strlen(s)); }
  size_t write(char c) { return write((uint8_t)c); }
  size_t write(const uint8_t *b, size_t n) {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
  size_t print(const char *s) { return write(s); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(unsigned char v, int base = DEC) { return print((unsigned long)v, base); }
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

class Stream : public Print {
 public:
  virtual int available(void) = 0;
  virtual int read(void) = 0;
  virtual int peek(void) = 0;
};

class HardwareSerial : public Stream {
 public:
  void begin(unsigned long baud, ...);
  void end(void) {}
  int available(void) override;
  int read(void) override;
  int peek(void) override;
  void flush(void);
  size_t write(uint8_t c) override;
  using Print::write;
  operator bool() const { return true; }
};
extern HardwareSerial Serial;
