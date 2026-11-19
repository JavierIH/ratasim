// Arduino's Servo library (1.1, the AVR's: Timer1 sends each attached servo its pulse every 20 ms), simulated: the
// pulse each pin sends, read by the simulator (mchip.cpp, msim_servo_us), which moves the servo by it.
#pragma once
#include "Arduino.h"

#define MIN_PULSE_WIDTH 544
#define MAX_PULSE_WIDTH 2400
#define DEFAULT_PULSE_WIDTH 1500

class Servo {
 public:
  uint8_t attach(int pin);
  uint8_t attach(int pin, int min, int max);
  void detach(void);
  void write(int value);                  // an angle (0..180), or a pulse in us from 544 up (as the library)
  void writeMicroseconds(int value);
  int read(void);
  int readMicroseconds(void);
  bool attached(void);

 private:
  int pin_ = -1, min_ = MIN_PULSE_WIDTH, max_ = MAX_PULSE_WIDTH;
};
