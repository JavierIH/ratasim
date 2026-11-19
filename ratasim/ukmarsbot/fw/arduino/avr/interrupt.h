// Interrupt vectors as functions the simulated chip calls (uchip.cpp); sei()/cli() its global interrupt flag.
#pragma once
#include "io.h"
#define ISR_NOBLOCK
#define ISR_BLOCK
#define ISR(vector, ...) extern "C" void vector(void)
void sim_sei(void);
void sim_cli(void);
#define sei() sim_sei()
#define cli() sim_cli()
