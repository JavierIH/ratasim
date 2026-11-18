// ESP32Encoder's API (madhephaestus; on the chip, its PCNT units): the counts of the quadrature encoder on these
// pins, from the simulator's wheel (gchip.cpp), at the time they are read.
#pragma once
#include "Arduino.h"

enum class puType { up, down, none };
enum encType { single, half, full };

class ESP32Encoder {
 public:
  void attachFullQuad(int a, int b) { attach(a, b, full); }
  void attachHalfQuad(int a, int b) { attach(a, b, half); }
  void attachSingleEdge(int a, int b) { attach(a, b, single); }
  int64_t getCount(void);
  int64_t clearCount(void) { return setCount(0); }
  int64_t setCount(int64_t value);
  int64_t pauseCount(void) { return 0; }
  int64_t resumeCount(void) { return 0; }
  void detach(void) { pin_a = -1; }
  bool isAttached(void) { return pin_a >= 0; }
  static puType useInternalWeakPullResistors;

 private:
  void attach(int a, int b, encType type) {
    pin_a = a, pin_b = b, this->type = type;
  }
  int pin_a = -1, pin_b = -1;
  encType type = full;
  int64_t offset = 0;
};
