// UKMARSBOT's Arduino Nano (ATmega328P at 16 MHz) simulated under the Arduino core (arduino/): mazerunner-core's
// setup() and loop() in a coroutine, its interrupts called as the chip calls them, and the peripherals its code
// drives through their registers. Python (ukmarsbot/robot.py) drives it with the usim_* functions at the end.
//
// Time: simulated, in nanoseconds. It moves when the firmware waits (delay) and by what its calls cost on the chip
// (below: a pass of loop(), an ATOMIC block, a register read, each interrupt's body); the rest of its computation
// takes none. Events happen at their times inside that: Timer2's compare match, an ADC conversion's end, an
// encoder's edge. Each sets its interrupt flag; with the global interrupt flag set, the interrupt runs there, in
// the middle of whatever the firmware was doing, by its priority (its vector's number). An interrupt clears the
// global flag while it runs, but for systick's (ISR_NOBLOCK: the others nest in it). When the time passes what the
// simulator asked for, the firmware yields to it.
//
// Its peripherals:
//   Timer2      CTC on OCR2A at its prescaler (TCCR2B): TIMER2_COMPA_vect, systick, 500 Hz
//   ADC         a conversion started by ADSC (13 ADC clocks at its prescaler, ADPS) samples ADMUX's channel when
//               it starts: a phototransistor's input lit if its emitter's pin is high, else dark; the
//               function switches and the battery as they are; ADC_vect at its end if ADIE
//   INT0, INT1  each encoder's XOR-ed channels (D2, D3, its 74HC86) and its B channel (D4, D5): a quadrature
//               sequence at the edges of each wheel's counts, an interrupt at each (attachInterrupt, CHANGE)
//   ports       PORTB/C/D drive the pins (digitalWrite, the firmware's fast_write_pin); PINx read them, the
//               encoders' inputs from their wheels
//   PWM         analogWrite's duty on D9 and D10 (Timer1: the motors), read by the simulator with D7 and D8
//   USART       115200 baud: what it prints, buffered for the simulator at that rate (its 64-byte buffer full,
//               Serial.write waits, as on the chip); what the simulator types, read back
#include <ucontext.h>

#include "Arduino.h"
#include "util/atomic.h"

void setup(void);
void loop(void);
extern "C" void TIMER2_COMPA_vect(void);
extern "C" void ADC_vect(void);

// ---- What the chip's work costs (ns), at 16 MHz -----------------------------------------------------------------
#define LOOP_PASS_NS 2000          // loop() returning to main() and called again (serialEventRun)
#define ATOMIC_NS 2000             // an ATOMIC block (SREG saved, cli, restored) and the code around it in a poll
#define REG_READ_NS 250            // a register read (polling ADSC)
#define CLOCK_READ_NS 1000         // millis(), micros()
#define INT_ISR_NS 5000            // an encoder's interrupt
#define ADC_ISR_NS 5000            // the ADC's interrupt ("each interrupt action takes about 5us", adc.h)
#define SYSTICK_ISR_NS 300000      // systick's (adc.h, systick.h: 10% of its 2 ms at rest, 30-40% moving)
#define CHAR_NS 86806              // a character at 115200 baud
#define TX_BUFFER 64
#define CPU_HZ 16000000ull
#define STACK_BYTES (1024 * 1024)
#define NEVER (~0ull)
enum { USIM_RUNNING = 0, USIM_HALTED = 2 };

// ---- The coroutine and the clock --------------------------------------------------------------------------------
static ucontext_t sim_ctx, fw_ctx;
static char *fw_stack;
static uint8_t in_fw, state, sreg_i, isr_depth;
static uint64_t now_ns, until_ns;
static uint8_t regs[SIM_REGS];
SimReg8 sim_regs[SIM_REGS];
SimAdc16 sim_adc16;

static void events(void);
static void dispatch(void);
static uint64_t next_event(void);

static void yield_if_due(void) {
  if (in_fw && now_ns >= until_ns) swapcontext(&fw_ctx, &sim_ctx);
}
// The firmware's work takes ns: the events in it happen, their interrupts run (their time on top).
static void advance(uint64_t ns) {
  uint64_t remaining = ns;
  for (;;) {
    const uint64_t t = next_event();
    if (t > now_ns + remaining) {
      now_ns += remaining;
      remaining = 0;
      yield_if_due();
      if (next_event() > now_ns) return;
      continue;
    }
    if (t > now_ns) {
      remaining -= t - now_ns;
      now_ns = t;
    }
    events();
    dispatch();
    yield_if_due();
  }
}
static void wait_until(uint64_t t) {
  while (now_ns < t) advance(t - now_ns);
}
static void fw_entry(void) {
  in_fw = 1;
  setup();
  for (;;) {
    loop();
    advance(LOOP_PASS_NS);
  }
}

unsigned long millis(void) {
  advance(CLOCK_READ_NS);
  return (uint32_t)(now_ns / 1000000u);
}
unsigned long micros(void) {
  advance(CLOCK_READ_NS);
  return (uint32_t)(now_ns / 1000u);
}
void delay(unsigned long ms) { wait_until(now_ns + (uint64_t)ms * 1000000u); }
void delayMicroseconds(unsigned int us) { wait_until(now_ns + (uint64_t)us * 1000u); }
void yield(void) { advance(LOOP_PASS_NS); }

// ---- Interrupts -------------------------------------------------------------------------------------------------
static void (*int_isr[2])(void);
static uint8_t int_flag[2], t2_flag, adc_flag;

void sim_sei(void) {
  sreg_i = 1;
  dispatch();
}
void sim_cli(void) { sreg_i = 0; }
SimAtomic::SimAtomic() {
  advance(ATOMIC_NS);
  saved = sreg_i;
  sreg_i = 0;
}
SimAtomic::~SimAtomic() {
  sreg_i = saved;
  if (sreg_i) dispatch();
}

static void run_isr(void (*isr)(void), uint64_t cost, int noblock) {
  const uint8_t saved = sreg_i;
  sreg_i = noblock;
  isr_depth++;
  isr();
  advance(cost);
  isr_depth--;
  sreg_i = saved;     // RETI
}
// Pending interrupts by priority (the vector's number: INT0 1, INT1 2, TIMER2_COMPA 7, ADC 21)
static void dispatch(void) {
  while (sreg_i) {
    if (int_flag[0] && int_isr[0]) {
      int_flag[0] = 0;
      run_isr(int_isr[0], INT_ISR_NS, 0);
    } else if (int_flag[1] && int_isr[1]) {
      int_flag[1] = 0;
      run_isr(int_isr[1], INT_ISR_NS, 0);
    } else if (t2_flag && (regs[SIM_TIMSK2] & _BV(OCIE2A))) {
      t2_flag = 0;
      run_isr(TIMER2_COMPA_vect, SYSTICK_ISR_NS, 1);
    } else if (adc_flag && (regs[SIM_ADCSRA] & _BV(ADIE))) {
      adc_flag = 0;
      run_isr(ADC_vect, ADC_ISR_NS, 0);
    } else {
      return;
    }
  }
}

// ---- Pins -------------------------------------------------------------------------------------------------------
#define PINS 22
static int16_t pwm[PINS];      // analogWrite's value while a pin's PWM is on, else -1

static int port_of(uint8_t pin, uint8_t *bit) {   // D0-7 PORTD, D8-13 PORTB, A0-A5 PORTC
  if (pin <= 7) return *bit = pin, SIM_PORTD;
  if (pin <= 13) return *bit = pin - 8, SIM_PORTB;
  if (pin <= 19) return *bit = pin - 14, SIM_PORTC;
  return -1;
}
static uint8_t level(uint8_t pin) {
  uint8_t b;
  const int p = port_of(pin, &b);
  return p < 0 ? 0 : (regs[p] >> b) & 1;
}
void pinMode(uint8_t pin, uint8_t mode) {
  uint8_t b;
  const int p = port_of(pin, &b);
  if (p < 0) return;
  if (mode == OUTPUT) regs[p + (SIM_DDRB - SIM_PORTB)] |= 1 << b;
  else regs[p + (SIM_DDRB - SIM_PORTB)] &= ~(1 << b);
}
void digitalWrite(uint8_t pin, uint8_t val) {
  uint8_t b;
  const int p = port_of(pin, &b);
  if (p < 0) return;
  pwm[pin] = -1;   // turnOffPWM
  if (val) regs[p] |= 1 << b;
  else regs[p] &= ~(1 << b);
}
void analogWrite(uint8_t pin, int val) {
  pinMode(pin, OUTPUT);
  if (val <= 0) digitalWrite(pin, LOW);
  else if (val >= 255) digitalWrite(pin, HIGH);
  else if (pin < PINS) pwm[pin] = (int16_t)val;
}
static uint8_t encoder_level(uint8_t pin, uint8_t *out);
int digitalRead(uint8_t pin) {
  uint8_t v;
  if (encoder_level(pin, &v)) return v;
  return level(pin);
}

// ---- Encoders: each wheel's counts a quadrature sequence on its pins ------------------------------------------------
struct Wheel {
  uint8_t clk_pin, b_pin, irq;
  int64_t pos;            // edges so far
  double c0, rate;        // counts at t0, counts a nanosecond (forward positive, as its firmware counts them)
  uint64_t t0;
};
static Wheel wheels[2] = {{2, 4, 0, 0, 0, 0, 0}, {3, 5, 1, 0, 0, 0, 0}};   // left, right (config-ukmarsbot.h)
static const uint8_t GRAY_A[4] = {0, 0, 1, 1}, GRAY_B[4] = {0, 1, 1, 0};  // A, B: +1 a step in its decoder

static int64_t wheel_target(const Wheel &w, uint64_t t) {
  return (int64_t)floor(w.c0 + w.rate * ((double)t - (double)w.t0));
}
static uint64_t wheel_next(const Wheel &w) {
  if (wheel_target(w, now_ns) != w.pos) return now_ns;
  if (w.rate == 0) return NEVER;
  const double edge = w.rate > 0 ? (double)(w.pos + 1) : (double)w.pos - 1e-9;
  const double t = (double)w.t0 + (edge - w.c0) / w.rate;
  if (t > 1.8e19) return NEVER;
  const uint64_t ti = t <= (double)now_ns ? now_ns + 1 : (uint64_t)ceil(t);
  return ti;
}
static uint8_t encoder_level(uint8_t pin, uint8_t *out) {
  for (const Wheel &w : wheels) {
    const int i = (int)(w.pos & 3);
    if (pin == w.clk_pin) return *out = GRAY_A[i] ^ GRAY_B[i], 1;
    if (pin == w.b_pin) return *out = GRAY_B[i], 1;
  }
  return 0;
}
void attachInterrupt(uint8_t interrupt, void (*isr)(void), int mode) {
  (void)mode;   // CHANGE: every edge of the XOR-ed channels
  if (interrupt < 2) int_isr[interrupt] = isr;
}
void detachInterrupt(uint8_t interrupt) {
  if (interrupt < 2) int_isr[interrupt] = NULL;
}

// ---- Timer2 -----------------------------------------------------------------------------------------------------
static uint64_t t2_next = NEVER;
static uint64_t t2_period(void) {
  static const uint16_t presc[8] = {0, 1, 8, 32, 64, 128, 256, 1024};
  const uint16_t p = presc[regs[SIM_TCCR2B] & 7];
  if (!p) return 0;
  return (uint64_t)(regs[SIM_OCR2A] + 1) * p * 1000000000ull / CPU_HZ;
}
static void timer2_config(void) {
  const uint64_t period = t2_period();
  if (!period) t2_next = NEVER;
  else if (t2_next == NEVER) t2_next = now_ns + period;
}

// ---- ADC --------------------------------------------------------------------------------------------------------
static uint16_t analog_dark[8], analog_lit[8];
static int8_t emitter_pin[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
static uint8_t adc_busy;
static uint16_t adc_sample, adc_result;
static uint64_t adc_done = NEVER;

static uint16_t analog_input(uint8_t ch) {
  ch &= 7;
  if (emitter_pin[ch] >= 0 && level((uint8_t)emitter_pin[ch])) return analog_lit[ch];
  return analog_dark[ch];
}
static void adc_start(void) {
  static const uint8_t presc[8] = {2, 2, 4, 8, 16, 32, 64, 128};
  adc_busy = 1;
  adc_sample = analog_input(regs[SIM_ADMUX] & 0x0F);
  adc_done = now_ns + 13ull * presc[regs[SIM_ADCSRA] & 7] * 1000000000ull / CPU_HZ;
}
int analogRead(uint8_t pin) {
  if (pin >= 14) pin -= 14;
  regs[SIM_ADMUX] = (regs[SIM_ADMUX] & 0xF0) | (pin & 7);
  adc_start();
  wait_until(adc_done);
  return adc_result;
}
void analogReference(uint8_t mode) { regs[SIM_ADMUX] = (regs[SIM_ADMUX] & 0x3F) | (mode << 6); }
uint16_t sim_adc_read(void) { return adc_result; }

// ---- Registers --------------------------------------------------------------------------------------------------
uint8_t sim_reg_read(int reg) {
  switch (reg) {
    case SIM_PINB:
    case SIM_PINC:
    case SIM_PIND: {
      const int port = reg - SIM_PINB + SIM_PORTB;
      uint8_t v = regs[port];
      for (uint8_t bit = 0; bit < 8; bit++) {
        const uint8_t pin = port == SIM_PORTD ? bit : port == SIM_PORTB ? bit + 8 : bit + 14;
        uint8_t e;
        if (pin < PINS && encoder_level(pin, &e)) v = (uint8_t)((v & ~(1 << bit)) | (e << bit));
      }
      return v;
    }
    case SIM_ADCSRA:
      if (isr_depth == 0) advance(REG_READ_NS);    // polling ADSC: the time passes
      return (uint8_t)(regs[reg] | (adc_busy ? _BV(ADSC) : 0) | (adc_flag ? _BV(ADIF) : 0));
    case SIM_SREG:
      return (uint8_t)(sreg_i << SREG_I);
  }
  return regs[reg];
}
void sim_reg_write(int reg, uint8_t value) {
  switch (reg) {
    case SIM_PINB:
    case SIM_PINC:
    case SIM_PIND:       // writing PINx toggles PORTx
      regs[reg - SIM_PINB + SIM_PORTB] ^= value;
      return;
    case SIM_ADCSRA:
      if (value & _BV(ADIF)) adc_flag = 0;          // a one written clears the flag (and a read-modify-write does)
      regs[reg] = value & ~(_BV(ADSC) | _BV(ADIF));
      if ((value & _BV(ADSC)) && !adc_busy && (value & _BV(ADEN))) adc_start();
      dispatch();
      return;
    case SIM_SREG:
      sreg_i = (value >> SREG_I) & 1;
      if (sreg_i) dispatch();
      return;
  }
  regs[reg] = value;
  if (reg == SIM_TCCR2B || reg == SIM_OCR2A || reg == SIM_TCCR2A) timer2_config();
  if (reg == SIM_TIMSK2) dispatch();
}

// ---- Events -----------------------------------------------------------------------------------------------------
static uint64_t next_event(void) {
  uint64_t t = t2_next;
  if (adc_busy && adc_done < t) t = adc_done;
  for (const Wheel &w : wheels) {
    const uint64_t e = wheel_next(w);
    if (e < t) t = e;
  }
  return t;
}
static void events(void) {
  if (t2_next <= now_ns) {
    t2_flag = 1;
    const uint64_t period = t2_period();
    t2_next = period ? t2_next + period : NEVER;
  }
  if (adc_busy && adc_done <= now_ns) {
    adc_busy = 0;
    adc_done = NEVER;
    adc_result = adc_sample;
    adc_flag = 1;
  }
  for (Wheel &w : wheels) {
    const int64_t target = wheel_target(w, now_ns);
    if (target != w.pos) {          // one edge at a time: a quadrature step, an interrupt
      w.pos += target > w.pos ? 1 : -1;
      int_flag[w.irq] = 1;
    }
  }
}

// ---- Serial -----------------------------------------------------------------------------------------------------
HardwareSerial Serial;
static char tx[1 << 17];
static uint32_t tx_len;
static uint64_t tx_free;           // when the USART has sent all it was given
static char rx[4096];
static uint32_t rx_head, rx_len;
void HardwareSerial::begin(unsigned long baud, ...) { (void)baud; }
size_t HardwareSerial::write(uint8_t c) {
  if (tx_free > now_ns + TX_BUFFER * CHAR_NS && isr_depth == 0) wait_until(tx_free - TX_BUFFER * CHAR_NS);
  tx_free = (tx_free > now_ns ? tx_free : now_ns) + CHAR_NS;
  if (tx_len < sizeof(tx)) tx[tx_len++] = (char)c;
  return 1;
}
void HardwareSerial::flush(void) { wait_until(tx_free); }
int HardwareSerial::available(void) {
  advance(CLOCK_READ_NS);
  return (int)rx_len;
}
int HardwareSerial::peek(void) { return rx_len ? (uint8_t)rx[rx_head] : -1; }
int HardwareSerial::read(void) {
  if (!rx_len) return -1;
  const int c = (uint8_t)rx[rx_head];
  rx_head = (rx_head + 1) % sizeof(rx), rx_len--;
  return c;
}
static size_t print_number(Print &p, unsigned long v, int base, bool negative) {
  char b[72], *s = b + sizeof(b) - 1;
  *s = 0;
  if (base < 2) base = 10;
  do {
    const int d = (int)(v % base);
    *--s = (char)(d < 10 ? '0' + d : 'A' + d - 10);
    v /= base;
  } while (v);
  if (negative) *--s = '-';
  return p.write(s);
}
size_t Print::print(long v, int base) {
  if (base == 10 && v < 0) return print_number(*this, (unsigned long)(-v), 10, true);
  if (base != 10) v = (long)(uint32_t)v;       // as on the chip: 32-bit longs
  return print_number(*this, (unsigned long)v, base, false);
}
size_t Print::print(unsigned long v, int base) { return print_number(*this, v, base, false); }
size_t Print::print(double v, int digits) {   // as Arduino's Print::printFloat
  if (isnan(v)) return write("nan");
  if (isinf(v)) return write("inf");
  if (v > 4294967040.0 || v < -4294967040.0) return write("ovf");
  char b[64];
  snprintf(b, sizeof(b), "%.*f", digits, v);
  return write(b);
}

// ---- Simulator API ----------------------------------------------------------------------------------------------
extern "C" {
int usim_boot(void) {
  for (int i = 0; i < SIM_REGS; i++) sim_regs[i].id = i;
  memset(pwm, -1, sizeof(pwm));
  // the Arduino core's init(): timers 1 and 2 for its PWM at 64, the ADC on at 128; interrupts on
  regs[SIM_TCCR1A] = _BV(WGM10), regs[SIM_TCCR1B] = _BV(CS11) | _BV(CS10);
  regs[SIM_TCCR2A] = _BV(WGM20), regs[SIM_TCCR2B] = _BV(CS22);
  regs[SIM_ADCSRA] = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  timer2_config();
  sreg_i = 1;
  fw_stack = (char *)malloc(STACK_BYTES);
  if (!fw_stack) return 0;
  getcontext(&fw_ctx);
  fw_ctx.uc_stack.ss_sp = fw_stack;
  fw_ctx.uc_stack.ss_size = STACK_BYTES;
  fw_ctx.uc_link = NULL;
  makecontext(&fw_ctx, fw_entry, 0);
  return 1;
}
// Runs the firmware until this time (ns since it was switched on).
uint8_t usim_run_until(uint64_t t_ns) {
  if (state != USIM_RUNNING) return state;
  until_ns = t_ns;
  if (now_ns < until_ns) swapcontext(&sim_ctx, &fw_ctx);
  return state;
}
uint64_t usim_time_ns(void) { return now_ns; }
// A wheel's encoder (0 left, 1 right): its counts now, forward positive, and how fast they change (counts a ns).
void usim_set_encoder(int side, double counts, double counts_per_ns) {
  if (side < 0 || side > 1) return;
  Wheel &w = wheels[side];
  w.c0 = counts, w.rate = counts_per_ns, w.t0 = now_ns;
}
// An analog input (channel 0..7): its counts with the emitter on that pin lit, and dark (pin -1: no emitter).
void usim_set_analog(int ch, int emitter, int lit, int dark) {
  if (ch < 0 || ch > 7) return;
  emitter_pin[ch] = (int8_t)emitter;
  analog_lit[ch] = (uint16_t)(lit < 0 ? 0 : lit > 1023 ? 1023 : lit);
  analog_dark[ch] = (uint16_t)(dark < 0 ? 0 : dark > 1023 ? 1023 : dark);
}
// A pin's output: its PWM duty (0..1) while on, else its level.
double usim_pin(int pin) {
  if (pin < 0 || pin >= PINS) return 0;
  if (pwm[pin] >= 0) return pwm[pin] / 255.0;
  return level((uint8_t)pin);
}
uint32_t usim_serial_tx(char *buffer, uint32_t size) {
  const uint32_t n = tx_len < size ? tx_len : size;
  memcpy(buffer, tx, n);
  memmove(tx, tx + n, tx_len - n);
  tx_len -= n;
  return n;
}
void usim_serial_rx(const char *data, uint32_t n) {
  for (uint32_t i = 0; i < n && rx_len < sizeof(rx); i++, rx_len++) rx[(rx_head + rx_len) % sizeof(rx)] = data[i];
}
}
