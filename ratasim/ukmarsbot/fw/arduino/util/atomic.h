// avr-libc's ATOMIC_BLOCK: the global interrupt flag cleared for the block and restored after it, however the block
// is left. What the block costs the chip (the flag's save and restore) is what lets the simulated time pass in the
// firmware's polling loops (uchip.cpp).
#pragma once
#include "../avr/interrupt.h"
struct SimAtomic {
  uint8_t saved, once = 1;
  SimAtomic();
  ~SimAtomic();
};
#define ATOMIC_RESTORESTATE 1
#define ATOMIC_FORCEON 2
#define ATOMIC_BLOCK(type) for (SimAtomic sim_atomic_; sim_atomic_.once; sim_atomic_.once = 0)
