// NezumiBot's chip (STM32F401) simulated: its main() in a coroutine, its SysTick (clock, encoders, the sensors'
// filter) once per simulated millisecond, and what its code touches through libopencm3 (ratasim/opencm3): the
// motors' PWM (TIM2: OC3 left, OC1 right) and the TB6612's direction and standby pins, the encoders' counters (TIM4
// left, TIM3 right), the sensors' ADC1 sequence that DMA keeps copying into sensors_raw, USART1. Python
// (nezumibot/robot.py) drives it with the nsim_* functions at the end.
//
// Time: main() yields every YIELD_CALLS calls of what its waiting loops poll (get_clock_ticks, get_sensor_bool)
// and in delay(); computation takes no simulated time.
#define _GNU_SOURCE
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucontext.h>
#include "fakecm3.h"

#define YIELD_CALLS 8
#define STACK_BYTES (1024 * 1024)
enum { NSIM_RUNNING = 0, NSIM_HALTED = 2 };

extern int firmware_main(void);
extern void sys_tick_handler(void);
extern volatile uint16_t *get_sensors_raw(void);

static ucontext_t sim_ctx, fw_ctx;
static char *fw_stack;
static uint32_t tick, cycles;
static uint8_t in_isr, state;
static unsigned calls;

static void yield(void) {
  calls = 0;
  swapcontext(&fw_ctx, &sim_ctx);
}
void nsim_poll(void) {
  if (!in_isr && ++calls >= YIELD_CALLS) yield();
}
void nsim_wait_tick(void) {
  if (!in_isr) yield();
}
uint32_t nsim_cycles(void) { return cycles; }
static void fw_entry(void) {
  firmware_main();
  state = NSIM_HALTED;
  for (;;) yield();
}

// main()'s menu polls the sensors' booleans in a loop
bool __real_get_sensor_bool(int index);
bool __wrap_get_sensor_bool(int index) {
  nsim_poll();
  return __real_get_sensor_bool(index);
}

// ---- GPIO ----
static uint16_t odr[4];
void gpio_set(uint32_t port, uint16_t pins) { odr[port & 3] |= pins; }
void gpio_clear(uint32_t port, uint16_t pins) { odr[port & 3] &= (uint16_t)~pins; }
void gpio_toggle(uint32_t port, uint16_t pins) { odr[port & 3] ^= pins; }
uint16_t gpio_get(uint32_t port, uint16_t pins) { return odr[port & 3] & pins; }

// ---- Timers: PWM compare values, encoders' counters ----
static uint32_t oc[9][4];
static uint16_t cnt[9];
void timer_set_oc_value(uint32_t tim, enum tim_oc_id id, uint32_t value) {
  if (tim < 9 && (unsigned)id < 4) oc[tim][id] = value;
}
uint32_t timer_get_counter(uint32_t tim) { return tim < 9 ? cnt[tim] : 0; }
void timer_set_counter(uint32_t tim, uint32_t count) {
  if (tim < 9) cnt[tim] = (uint16_t)count;
}
uint32_t dwt_read_cycle_counter(void) {
  nsim_poll();
  return cycles;
}

// ---- USART1 (printf) ----
static char tx[1 << 16];
static uint32_t tx_len;
void usart_send_blocking(uint32_t usart, uint16_t data) {
  (void)usart;
  if (tx_len < sizeof(tx)) tx[tx_len++] = (char)data;
}
int nsim_printf(const char *fmt, ...) {
  char b[512];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  for (int i = 0; i < n && i < (int)sizeof(b) - 1; i++) usart_send_blocking(USART1, (uint8_t)b[i]);
  return n;
}
static uint32_t scratch;
volatile uint32_t *zsim_mmio32(uint32_t addr) {
  (void)addr;
  scratch = 0;
  return &scratch;
}

// ---- Simulator API ----
int nsim_boot(void) {
  fw_stack = malloc(STACK_BYTES);
  if (!fw_stack) return 0;
  getcontext(&fw_ctx);
  fw_ctx.uc_stack.ss_sp = fw_stack;
  fw_ctx.uc_stack.ss_size = STACK_BYTES;
  fw_ctx.uc_link = NULL;
  makecontext(&fw_ctx, fw_entry, 0);
  swapcontext(&sim_ctx, &fw_ctx);
  return 1;
}

uint8_t nsim_step(void) {
  if (state != NSIM_RUNNING) return state;
  tick++;
  cycles += 84000u;
  in_isr = 1;
  sys_tick_handler();
  in_isr = 0;
  swapcontext(&sim_ctx, &fw_ctx);
  return state;
}

uint32_t nsim_time_ms(void) { return tick; }
void nsim_set_encoders(int32_t left, int32_t right) {   // ticks, forward positive (encoders.c negates the left)
  cnt[4] = (uint16_t)(-left);
  cnt[3] = (uint16_t)right;
}
void nsim_set_sensors(uint16_t fl, uint16_t l, uint16_t r, uint16_t fr) {   // ADC counts, DMA's order (sensors.c)
  volatile uint16_t *raw = get_sensors_raw();
  raw[0] = fl;
  raw[1] = l;
  raw[2] = r;
  raw[3] = fr;
}
static int16_t duty(uint32_t pwm, int fwd, int back) {
  if (fwd == back) return 0;    // both high or both low: brake
  return (int16_t)(fwd ? (int32_t)pwm : -(int32_t)pwm);
}
void nsim_get_motors(int16_t *left, int16_t *right, uint8_t *enabled) {   // duty in 1/1024, forward positive
  *left = duty(oc[2][TIM_OC3], (odr[GPIOB] & GPIO1) != 0, (odr[GPIOB] & GPIO2) != 0);
  *right = duty(oc[2][TIM_OC1], (odr[GPIOA] & GPIO6) != 0, (odr[GPIOA] & GPIO7) != 0);
  *enabled = (odr[GPIOB] & GPIO0) != 0;
}
uint32_t nsim_uart_tx(char *buffer, uint32_t size) {
  uint32_t n = tx_len < size ? tx_len : size;
  memcpy(buffer, tx, n);
  memmove(tx, tx + n, tx_len - n);
  tx_len -= n;
  return n;
}
