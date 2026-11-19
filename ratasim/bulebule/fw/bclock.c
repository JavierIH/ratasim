// Replaces mmlib's clock.c: the same clock, stopwatch and sleeps, but its waits give the time back to the simulator
// (bchip.c): on the chip they spin on a variable the SysTick interrupt changes.
#include "mmlib/clock.h"

void bsim_poll(void);
void bsim_wait_tick(void);

static volatile uint32_t clock_ticks;
static volatile uint32_t stopwatch_counter;

uint32_t get_clock_ticks(void) {
  bsim_poll();
  return clock_ticks;
}

void clock_tick(void) { clock_ticks++; }

void sleep_ticks(uint32_t ticks) {
  uint32_t awake = clock_ticks + ticks;

  while (awake > clock_ticks) bsim_wait_tick();
}

void sleep_seconds(float seconds) { sleep_ticks((uint32_t)(seconds * SYSTICK_FREQUENCY_HZ)); }

void stopwatch_start(void) { stopwatch_counter = read_cycle_counter(); }

float stopwatch_stop(void) {
  return (float)(read_cycle_counter() - stopwatch_counter) / (float)SYSCLK_FREQUENCY_HZ;
}

void sleep_us(uint32_t us) {
  uint32_t initial_cycles = read_cycle_counter();
  uint32_t sleep_cycles = (uint32_t)(SYSCLK_FREQUENCY_HZ * ((float)us / (float)MICROSECONDS_PER_SECOND));

  while (read_cycle_counter() - initial_cycles <= sleep_cycles) bsim_wait_tick();
}

void sleep_us_after(uint32_t cycle_counter, uint32_t us) {
  uint32_t sleep_cycles = (uint32_t)(SYSCLK_FREQUENCY_HZ * ((float)us / (float)MICROSECONDS_PER_SECOND));

  while (read_cycle_counter() - cycle_counter <= sleep_cycles) bsim_wait_tick();
}

void each(uint32_t period, void (*function)(void), uint32_t during) {
  uint32_t ticks_initial;
  uint32_t ticks_current;

  ticks_initial = get_clock_ticks();
  ticks_current = ticks_initial;
  while (ticks_current - ticks_initial < during) {
    if (ticks_current % period == 0) function();
    sleep_ticks(1);
    ticks_current = get_clock_ticks();
  }
}

bool wait_until(bool (*function)(void), uint32_t timeout) {
  uint32_t ticks_initial;

  ticks_initial = get_clock_ticks();
  while (get_clock_ticks() - ticks_initial < timeout) {
    if (function()) return true;
  }
  return false;
}
