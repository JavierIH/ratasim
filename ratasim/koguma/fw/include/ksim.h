// Forced into every file of KOGUMA's firmware (Makefile: -include ksim.h). Its main loop waits on what its 1 kHz
// interrupt changes in bare loops (`while (g_timCount < n) {}`, `while (G_Motor_X < x) {}`), as on its chip, where
// the interrupt comes in the middle of them; here time passes in each pass of each loop, so that it comes: every
// `while` and `for` takes a pass of its chip (kchip.c: ksim_pass, KSIM_PASS_NS of simulated time), its files
// unchanged. The C library's headers come first, so their own loops are left as they are.
#ifndef KSIM_H
#define KSIM_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void ksim_pass(void);

#define while(...) while (ksim_pass(), (__VA_ARGS__))
#define for(...) for (__VA_ARGS__) if (ksim_pass(), 0) {} else

#endif
