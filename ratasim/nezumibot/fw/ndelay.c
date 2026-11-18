// Replaces NezumiBot's delay.c (and setup.c: the chip needs no configuration here): the same clock, but its waits
// give the time back to the simulator (nchip.c).
#include "delay.h"

void nsim_poll(void);
void nsim_wait_tick(void);
uint32_t nsim_cycles(void);

static volatile uint32_t clock_ticks;

void setup(void) {}

void clock_tick(void) { clock_ticks++; }

uint32_t read_cycle_counter(void) { return dwt_read_cycle_counter(); }

void delay(uint32_t ms) {
  uint32_t awake = clock_ticks + ms;
  while (awake > clock_ticks) nsim_wait_tick();
}

uint32_t get_clock_ticks(void) {
  nsim_poll();
  return clock_ticks;
}

void delay_us(uint32_t us) {
  uint32_t start = nsim_cycles();
  uint32_t n = (uint32_t)(SYSCLK_FREQUENCY_HZ / MICROSECONDS_PER_SECOND) * us;
  while (nsim_cycles() - start <= n) nsim_wait_tick();
}
