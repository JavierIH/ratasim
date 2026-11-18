// Perenquen's chip (dsPIC33EP512MC806, 60 MIPS) simulated: its main() in a coroutine, its registers in host memory
// (gen_sfr.py), and the peripherals its code uses, as its code uses them:
//
//   Timer1      every SCHEDULER_UNIT (250 us) when on, its interrupt (_T1Interrupt: aversive's scheduler, which runs
//               the control system every millisecond) when enabled and above the CPU's priority level (SR.IPL)
//   ADC1        manual sampling (sensor.c): the input sampled while SAMP is set, converted when it is cleared, DONE
//               and ADC1BUF0 set then; each phototransistor's input lit or dark by its emitter's pin (LATD)
//   QEI1, QEI2  the encoders' 32-bit counters (POSxCNTL, POSxHLD)
//   PWM1, PWM2  the high-speed PWM's duty registers (actuator.c: SDCx forward, PDCx backwards, of PTPER)
//   oscillator  the clock switch it asks for (OSCCON) done at once, the PLL locked
//   LEDs        LATG3, LATF2, LATF3
//
// The UART is puart.c. Python (perenquen/robot.py) drives the chip with the psim_* functions at the end.
//
// Time: one psim_step() is a millisecond: four Timer1 periods, each its interrupt then main() until it waits.
// main() gives the time back when it polls (every YIELD_CALLS of its locks of the interrupts, IRQ_LOCK, which every
// waiting loop of aversive does) and in __delay32(); computation takes no simulated time.
#define _GNU_SOURCE
#define __dsPIC33EP512MC806__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucontext.h>
#include "p33Exxxx.h"

#define FCY 60000000UL
#define TIMER_US 250
#define CYCLES_PER_TICK (FCY / 1000000UL * TIMER_US)
#define YIELD_CALLS 8
#define STACK_BYTES (1024 * 1024)
#define T1_DEFAULT_IPL 4
enum { PSIM_RUNNING = 0, PSIM_RESET = 1, PSIM_HALTED = 2 };

extern int firmware_main(void);
extern void _T1Interrupt(void);
void puart_service(uint32_t bytes);     // puart.c
void puart_reset(void);

static ucontext_t sim_ctx, fw_ctx;
static char *fw_stack;
static uint64_t cycles;
static uint32_t ticks;
static uint8_t in_isr, state;
static unsigned calls;

static void yield(void) {
  calls = 0;
  swapcontext(&fw_ctx, &sim_ctx);
}
static unsigned ipl(void) { return ((volatile SRBITS *)&sim_SR)->IPL; }

// main()'s waiting loops: called where it polls (outside interrupts, with them enabled)
void psim_poll(void) {
  if (!in_isr && ipl() < 7 && ++calls >= YIELD_CALLS) yield();
}

void __delay32(unsigned long n) {
  if (in_isr) return;
  uint64_t until = cycles + n;
  while (cycles < until) yield();
}

static void fw_entry(void) {
  firmware_main();
  state = PSIM_HALTED;
  for (;;) yield();
}

__attribute__((force_align_arg_pointer)) void sim_cpu_reset(void) {   // the RESET instruction
  state = PSIM_RESET;
  for (;;) yield();
}

// ---- oscillator ----
void sim_write_osccon(int high, unsigned value) {
  volatile OSCCONBITS *o = (volatile OSCCONBITS *)&sim_OSCCON;
  if (high) {
    o->NOSC = value & 7;
  } else if (value & 1) {          // OSWEN: the switch is done, the PLL locked
    o->COSC = o->NOSC;
    o->LOCK = 1;
  }
}

// ---- ADC1 ----
// Analog inputs (counts 0..1023): each phototransistor's with its emitter lit and dark, the rest as they are.
static uint16_t analog[32], analog_lit[32];
static int emitter_pin[32];        // LATD bit that lights the input's emitter, -1: none
static uint8_t sampling;
static uint16_t sample;

static uint16_t input(unsigned ch) {
  ch &= 31;
  if (emitter_pin[ch] >= 0 && (sim_LATD.w >> emitter_pin[ch]) & 1) return analog_lit[ch];
  return analog[ch];
}

static void adc_touch(void) {
  volatile AD1CON1BITS *a = (volatile AD1CON1BITS *)&sim_AD1CON1;
  if (!a->ADON) return;
  if (a->SAMP) {                   // sampling: the input now (until SAMP is cleared)
    sampling = 1;
    a->DONE = 0;
    sample = input(((volatile AD1CHS0BITS *)&sim_AD1CHS0)->CH0SA);
  } else if (sampling) {           // SAMP cleared: converted
    sampling = 0;
    sim_ADC1BUF0.w = sample;
    a->DONE = 1;
  }
}

volatile void *sim_touch(int reg) {
  switch (reg) {
    case SIM_REG_SR:
      psim_poll();
      return &sim_SR;
    case SIM_REG_AD1CON1:
      adc_touch();
      return &sim_AD1CON1;
  }
  return NULL;
}

// ---- Timer1 ----
static void timer1(void) {
  volatile T1CONBITS *t = (volatile T1CONBITS *)&sim_T1CON;
  if (!t->TON) return;
  ((volatile IFS0BITS *)&sim_IFS0)->T1IF = 1;
}

static void interrupts(void) {
  volatile IFS0BITS *f = (volatile IFS0BITS *)&sim_IFS0;
  volatile IEC0BITS *e = (volatile IEC0BITS *)&sim_IEC0;
  unsigned prio = ((volatile IPC0BITS *)&sim_IPC0)->T1IP;
  if (!prio) prio = T1_DEFAULT_IPL;
  if (f->T1IF && e->T1IE && prio > ipl()) {
    volatile SRBITS *sr = (volatile SRBITS *)&sim_SR;
    unsigned saved = sr->IPL;
    sr->IPL = prio;
    in_isr = 1;
    _T1Interrupt();                // clears T1IF; aversive's scheduler runs its events in it
    in_isr = 0;
    sr->IPL = saved;
  }
}

// ---- Simulator API ----
int psim_boot(void) {
  fw_stack = malloc(STACK_BYTES);
  if (!fw_stack) return 0;
  for (int i = 0; i < 32; i++) emitter_pin[i] = -1;
  ((volatile SRBITS *)&sim_SR)->IPL = 0;
  getcontext(&fw_ctx);
  fw_ctx.uc_stack.ss_sp = fw_stack;
  fw_ctx.uc_stack.ss_size = STACK_BYTES;
  fw_ctx.uc_link = NULL;
  makecontext(&fw_ctx, fw_entry, 0);
  swapcontext(&sim_ctx, &fw_ctx);
  return 1;
}

uint8_t psim_step(void) {
  for (int q = 0; q < 1000 / TIMER_US; q++) {
    if (state != PSIM_RUNNING) return state;
    cycles += CYCLES_PER_TICK;
    timer1();
    interrupts();
    puart_service(921600 / 10 / (1000 / TIMER_US));     // its Bluetooth link's 921600 baud
    swapcontext(&sim_ctx, &fw_ctx);
  }
  ticks++;
  return state;
}

uint32_t psim_time_ms(void) { return ticks; }

// the IR receivers' inputs: the emitter's LATD bit, the input lit and dark (counts)
void psim_set_ir(uint8_t ch, int8_t pin, uint16_t lit, uint16_t dark) {
  ch &= 31;
  emitter_pin[ch] = pin;
  analog_lit[ch] = lit > 1023 ? 1023 : lit;
  analog[ch] = dark > 1023 ? 1023 : dark;
}
void psim_set_adc(uint8_t ch, uint16_t counts) { analog[ch & 31] = counts > 1023 ? 1023 : counts; }

void psim_set_encoders(int32_t left, int32_t right) {   // counts, forward positive: QEI2 left, QEI1 right
  sim_POS1CNTL.w = (uint16_t)right;
  sim_POS1HLD.w = (uint16_t)((uint32_t)right >> 16);
  sim_POS2CNTL.w = (uint16_t)left;
  sim_POS2HLD.w = (uint16_t)((uint32_t)left >> 16);
}

// duty in 1/PTPER, forward positive (actuator.c: generator 1 right, 2 left; SDC forward, PDC backwards)
void psim_get_pwm(int16_t *left, int16_t *right, uint16_t *period) {
  int on = ((volatile PTCONBITS *)&sim_PTCON)->PTEN;
  *right = on ? (int16_t)((int)sim_SDC1.w - (int)sim_PDC1.w) : 0;
  *left = on ? (int16_t)((int)sim_SDC2.w - (int)sim_PDC2.w) : 0;
  *period = sim_PTPER.w;
}

uint8_t psim_leds(void) {   // bit 0 LED1 (G3), bit 2 LED3 (F2), bit 3 LED4 (F3)
  return (uint8_t)(((sim_LATG.w >> 3) & 1) | (((sim_LATF.w >> 2) & 1) << 2) | (((sim_LATF.w >> 3) & 1) << 3));
}
