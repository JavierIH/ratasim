// Miniskybot's Skymega board (an ATmega328P at 16 MHz) simulated under the Arduino core (arduino/): the program's
// setup() and loop() in a coroutine, and what it drives: its pins, its two servos' pulses (Servo), the I2C bus with
// the SRF02 ultrasonic ranger on it (Wire), its serial port. Python (miniskybot/robot.py) drives it with the msim_*
// functions at the end.
//
// Time: simulated, in nanoseconds. It moves when the program waits (delay) and by what its calls cost on the chip (a
// pass of loop(), a clock read, each byte on the I2C bus); the rest of its computation takes none. When the time
// passes what the simulator asked for, the program yields to it.
//
// The SRF02 (Devantech's data sheet, its I2C mode): address 0xE0 (0x70 in Arduino's 7 bits); a command written to
// register 0 (0x50 inches, 0x51 cm, 0x52 us) starts a ranging, which takes 66 ms; meanwhile it answers nothing on
// the bus (its address not acknowledged, as the data sheet says), then registers 2 and 3 hold its result, high
// byte first. Register 0 reads its software revision, 1 reads 0x80. Its echo (the distance it hears, mm, or none)
// comes from the simulator, asked for when a ranging starts (msim_ranging).
#include <ucontext.h>

#include "Arduino.h"
#include "Servo.h"
#include "Wire.h"

void setup(void);
void loop(void);

// ---- What the chip's work costs (ns), at 16 MHz -----------------------------------------------------------------
#define LOOP_PASS_NS 2000          // loop() returning to main() and called again
#define CLOCK_READ_NS 1000         // millis(), micros()
#define PIN_NS 500                 // digitalRead(), digitalWrite(), pinMode()
#define I2C_BYTE_NS 90000          // a byte and its acknowledge at 100 kHz
#define I2C_START_STOP_NS 20000    // a start and a stop condition
#define CHAR_NS 86806              // a character at 115200 baud
#define TX_BUFFER 64
#define STACK_BYTES (1024 * 1024)
enum { MSIM_RUNNING = 0, MSIM_HALTED = 2 };

// ---- The coroutine and the clock --------------------------------------------------------------------------------
static ucontext_t sim_ctx, fw_ctx;
static char *fw_stack;
static uint8_t in_fw, state;
static uint64_t now_ns, until_ns;

static void advance(uint64_t ns) {      // a wait past what the simulator asked for: it runs in between
  const uint64_t target = now_ns + ns;
  while (in_fw && target >= until_ns) {
    if (now_ns < until_ns) now_ns = until_ns;
    swapcontext(&fw_ctx, &sim_ctx);
  }
  now_ns = target;
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
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ---- Pins -------------------------------------------------------------------------------------------------------
#define PINS 20
static uint8_t out_level[PINS], mode[PINS], input_level[PINS];
static int16_t servo_us[PINS];        // the pulse a pin sends to its servo (us), 0: none

void pinMode(uint8_t pin, uint8_t m) {
  advance(PIN_NS);
  if (pin < PINS) mode[pin] = m;
}
void digitalWrite(uint8_t pin, uint8_t val) {
  advance(PIN_NS);
  if (pin < PINS) out_level[pin] = val ? 1 : 0;
}
int digitalRead(uint8_t pin) {
  advance(PIN_NS);
  if (pin >= PINS) return 0;
  if (mode[pin] == OUTPUT) return out_level[pin];
  return input_level[pin];
}

// ---- Servo --------------------------------------------------------------------------------------------------------
uint8_t Servo::attach(int pin) { return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); }
uint8_t Servo::attach(int pin, int lo, int hi) {
  if (pin < 0 || pin >= PINS) return 0;
  pinMode((uint8_t)pin, OUTPUT);
  pin_ = pin, min_ = lo, max_ = hi;
  if (!servo_us[pin]) servo_us[pin] = DEFAULT_PULSE_WIDTH;
  return 1;
}
void Servo::detach(void) {
  if (pin_ >= 0) servo_us[pin_] = 0;
  pin_ = -1;
}
void Servo::write(int value) {
  if (value < MIN_PULSE_WIDTH) {      // an angle, as the library takes it
    value = constrain(value, 0, 180);
    value = (int)map(value, 0, 180, min_, max_);
  }
  writeMicroseconds(value);
}
void Servo::writeMicroseconds(int value) {
  if (pin_ < 0) return;
  servo_us[pin_] = (int16_t)constrain(value, min_, max_);
}
int Servo::read(void) { return (int)map(readMicroseconds() + 1, min_, max_, 0, 180); }
int Servo::readMicroseconds(void) { return pin_ >= 0 ? servo_us[pin_] : 0; }
bool Servo::attached(void) { return pin_ >= 0; }

// ---- I2C: the SRF02 -----------------------------------------------------------------------------------------------
#define SRF02_ADDRESS 0x70
#define SRF02_RANGING_NS 66000000ull
#define SRF02_REVISION 6
TwoWire Wire;
static uint8_t tx_addr, tx_data[32], tx_n, rx_data[32], rx_n, rx_i;
static uint8_t srf_reg, srf_command, srf_range[2], srf_asked;
static uint64_t srf_busy_until;
static double srf_echo_mm = -1;       // what it hears (mm), < 0: nothing

static int srf_listening(void) { return now_ns >= srf_busy_until; }
static void srf_finish(void) {        // a ranging's result in its registers, in the units its command asked for
  if (srf_command == 0) return;
  double v = 0;
  if (srf_echo_mm >= 0) {
    if (srf_command == 0x50) v = srf_echo_mm / 25.4;
    else if (srf_command == 0x51) v = srf_echo_mm / 10.0;
    else v = srf_echo_mm * 2.0 / 0.343;   // the echo's time, us (sound at 343 m/s)
  }
  const unsigned r = (unsigned)lround(v > 65535 ? 65535 : v);
  srf_range[0] = (uint8_t)(r >> 8), srf_range[1] = (uint8_t)r;
  srf_command = 0;
}
static uint8_t srf_read_reg(uint8_t reg) {
  srf_finish();
  switch (reg) {
    case 0: return SRF02_REVISION;
    case 1: return 0x80;
    case 2: return srf_range[0];
    case 3: return srf_range[1];
    default: return 0;
  }
}

void TwoWire::begin(void) {}
void TwoWire::setClock(uint32_t hz) { (void)hz; }
void TwoWire::beginTransmission(uint8_t address) { tx_addr = address, tx_n = 0; }
size_t TwoWire::write(uint8_t data) {
  if (tx_n >= sizeof(tx_data)) return 0;
  tx_data[tx_n++] = data;
  return 1;
}
uint8_t TwoWire::endTransmission(bool stop) {
  (void)stop;
  advance(I2C_START_STOP_NS + (uint64_t)(1 + tx_n) * I2C_BYTE_NS);
  if (tx_addr != SRF02_ADDRESS || !srf_listening()) return 2;     // its address not acknowledged
  srf_finish();
  if (tx_n >= 1) srf_reg = tx_data[0];
  for (uint8_t i = 1; i < tx_n; i++, srf_reg++) {
    if (srf_reg == 0 && tx_data[i] >= 0x50 && tx_data[i] <= 0x52) {   // a ranging: the echo asked of the simulator
      srf_command = tx_data[i];
      srf_busy_until = now_ns + SRF02_RANGING_NS;
      srf_asked = 1;
      break;
    }
  }
  return 0;
}
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  rx_n = rx_i = 0;
  if (quantity > sizeof(rx_data)) quantity = sizeof(rx_data);
  if (address != SRF02_ADDRESS || !srf_listening()) {
    advance(I2C_START_STOP_NS + I2C_BYTE_NS);
    return 0;
  }
  advance(I2C_START_STOP_NS + (uint64_t)(1 + quantity) * I2C_BYTE_NS);
  for (uint8_t i = 0; i < quantity; i++) rx_data[rx_n++] = srf_read_reg(srf_reg++);
  return rx_n;
}
int TwoWire::available(void) { return rx_n - rx_i; }
int TwoWire::read(void) { return rx_i < rx_n ? rx_data[rx_i++] : -1; }

// ---- Serial -----------------------------------------------------------------------------------------------------
HardwareSerial Serial;
static char tx[1 << 16];
static uint32_t tx_len;
static uint64_t tx_free;           // when the USART has sent all it was given
void HardwareSerial::begin(unsigned long baud) { (void)baud; }
size_t HardwareSerial::write(uint8_t c) {
  if (tx_free > now_ns + TX_BUFFER * CHAR_NS) wait_until(tx_free - TX_BUFFER * CHAR_NS);
  tx_free = (tx_free > now_ns ? tx_free : now_ns) + CHAR_NS;
  if (tx_len < sizeof(tx)) tx[tx_len++] = (char)c;
  return 1;
}
void HardwareSerial::flush(void) { wait_until(tx_free); }
int HardwareSerial::available(void) {
  advance(CLOCK_READ_NS);
  return 0;
}
int HardwareSerial::read(void) { return -1; }
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
  return print_number(*this, (unsigned long)v, base, false);
}
size_t Print::print(unsigned long v, int base) { return print_number(*this, v, base, false); }
size_t Print::print(double v, int digits) {
  char b[64];
  snprintf(b, sizeof(b), "%.*f", digits, v);
  return write(b);
}

// ---- Simulator API ----------------------------------------------------------------------------------------------
extern "C" {
int msim_boot(void) {
  for (int i = 0; i < PINS; i++) input_level[i] = 1;      // inputs pulled up (the button's, I2C's 4K7)
  fw_stack = (char *)malloc(STACK_BYTES);
  if (!fw_stack) return 0;
  getcontext(&fw_ctx);
  fw_ctx.uc_stack.ss_sp = fw_stack;
  fw_ctx.uc_stack.ss_size = STACK_BYTES;
  fw_ctx.uc_link = NULL;
  makecontext(&fw_ctx, fw_entry, 0);
  return 1;
}
// Runs the program until this time (ns since it was switched on).
uint8_t msim_run_until(uint64_t t_ns) {
  if (state != MSIM_RUNNING) return state;
  until_ns = t_ns;
  if (now_ns < until_ns) swapcontext(&sim_ctx, &fw_ctx);
  return state;
}
uint64_t msim_time_ns(void) { return now_ns; }
// An input pin's level (the button's: low when pressed).
void msim_set_input(int pin, int level) {
  if (pin >= 0 && pin < PINS) input_level[pin] = level ? 1 : 0;
}
// An output pin's level.
int msim_pin(int pin) { return pin >= 0 && pin < PINS ? out_level[pin] : 0; }
// The pulse a servo's pin sends (us), 0: none.
int msim_servo_us(int pin) { return pin >= 0 && pin < PINS ? servo_us[pin] : 0; }
// 1 when the SRF02 started a ranging since the last call: the simulator then gives it its echo (msim_set_echo).
int msim_ranging(void) {
  const int r = srf_asked;
  srf_asked = 0;
  return r;
}
void msim_set_echo(double mm) { srf_echo_mm = mm; }
uint32_t msim_serial_tx(char *buffer, uint32_t size) {
  const uint32_t n = tx_len < size ? tx_len : size;
  memcpy(buffer, tx, n);
  memmove(tx, tx + n, tx_len - n);
  tx_len -= n;
  return n;
}
}
