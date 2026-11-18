// Replaces ZoroBot3's delay.c: the same clock, but its waits give the time back to the simulator (zchip.c).
#include "delay.h"

void zsim_poll(void);
void zsim_wait_tick(void);
uint32_t zsim_cycles(void);

static volatile uint32_t clock_ticks;

void clock_tick(void) {
  clock_ticks++;
}

uint32_t read_cycle_counter(void) {
  return dwt_read_cycle_counter();
}

uint32_t get_us_counter(void) {
  return dwt_read_cycle_counter() / (SYSCLK_FREQUENCY_HZ / MICROSECONDS_PER_SECOND);
}

void delay(uint32_t ms) {
  uint32_t awake = clock_ticks + ms;
  while (awake > clock_ticks) zsim_wait_tick();
}

uint32_t get_clock_ticks(void) {
  zsim_poll();
  return clock_ticks;
}

void delay_us(uint32_t us) {
  uint32_t start = zsim_cycles();
  uint32_t n = (uint32_t)(SYSCLK_FREQUENCY_HZ / MICROSECONDS_PER_SECOND) * us;
  while (zsim_cycles() - start <= n) zsim_wait_tick();
}
