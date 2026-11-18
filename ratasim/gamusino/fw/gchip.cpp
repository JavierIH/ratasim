// Gamusino's ESP32, simulated under the Arduino core (arduino/): its sketch's setup() and loop() in a coroutine, as
// arduino-esp32's loop task runs them, and the peripherals they touch. Python (gamusino/robot.py) drives it with the
// gsim_* functions at the end.
//
// Time: simulated, in nanoseconds. It moves when the sketch waits (delay, delayMicroseconds) and by what its calls
// cost on the chip (below: reading the clock, a pass of loop(), the bytes on SPI, the NeoPixels' bits, an ADC
// conversion); the rest of its computation takes none. When the time passes what the simulator asked for, the
// sketch yields to it (in the middle of whatever it was doing, as an interrupt would find it).
//
// Its peripherals:
//   pins and LEDC   levels and duties (channels of 2^bits counts at their frequency), read by the simulator
//   ADC             12 bits, the values the simulator sets per pin
//   SPI             each byte to the device whose chip select is low: the LSM6DSR, a register file whose outputs
//                   are the simulator's angular rates and accelerations, latched at its output data rate, with noise
//   encoders        ESP32Encoder: each encoder's counts at the time they are read, from its wheel's angle and speed
//   NeoPixels       the colours last shown, as the LEDs light them (brightness applied)
//   Serial          its output buffered for the simulator; input from it
#include <ucontext.h>

#include <stdarg.h>

#include "Adafruit_NeoPixel.h"
#include "Arduino.h"
#include "ESP32Encoder.h"
#include "SPI.h"
#include "Wire.h"

void setup(void);
void loop(void);

// ---- What the chip's calls cost (ns): arduino-esp32 at 240 MHz -------------------------------------------------
#define CLOCK_READ_NS 500          // micros(), millis()
#define LOOP_PASS_NS 500           // the loop task between two loop() calls
#define SPI_TRANSACTION_NS 2000    // beginTransaction, the chip select, the driver's set-up
#define ADC_READ_NS 10000          // analogRead()
#define NEOPIXEL_BIT_NS 1250       // 800 kHz
#define NEOPIXEL_RESET_NS 50000
#define STACK_BYTES (1024 * 1024)
enum { GSIM_RUNNING = 0, GSIM_HALTED = 2 };

// ---- The coroutine and the clock -------------------------------------------------------------------------------
static ucontext_t sim_ctx, fw_ctx;
static char *fw_stack;
static uint8_t in_fw, state;
static uint64_t now_ns, until_ns;

static void advance(uint64_t ns) {
  now_ns += ns;
  if (in_fw && now_ns >= until_ns) swapcontext(&fw_ctx, &sim_ctx);
}
static void wait_until(uint64_t t) {
  while (now_ns < t) {
    uint64_t step = (until_ns > now_ns && until_ns < t ? until_ns : t) - now_ns;
    advance(step ? step : t - now_ns);
  }
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
unsigned long micros(void) {   // 32 bits, as on the chip
  advance(CLOCK_READ_NS);
  return (uint32_t)(now_ns / 1000u);
}
void delay(uint32_t ms) { wait_until(now_ns + (uint64_t)ms * 1000000u); }
void delayMicroseconds(uint32_t us) { wait_until(now_ns + (uint64_t)us * 1000u); }
void yield(void) { advance(LOOP_PASS_NS); }

// ---- Noise ------------------------------------------------------------------------------------------------------
static uint64_t rng = 0x9E3779B97F4A7C15ull;
static double uniform(void) {
  rng ^= rng << 13, rng ^= rng >> 7, rng ^= rng << 17;
  return ((rng >> 11) + 0.5) / 9007199254740992.0;
}
static double gauss(void) { return sqrt(-2.0 * log(uniform())) * cos(TWO_PI * uniform()); }

// ---- Pins, LEDC, ADC --------------------------------------------------------------------------------------------
#define PINS 40
#define LEDC_CHANNELS 16
static uint8_t level[PINS], mode_[PINS];
static int8_t pin_channel[PINS];
static uint16_t adc[PINS];
static uint8_t ledc_bits[LEDC_CHANNELS];
static uint32_t ledc_duty[LEDC_CHANNELS];
static double ledc_freq[LEDC_CHANNELS];
static void spi_select(uint8_t pin, uint8_t high);

void pinMode(uint8_t pin, uint8_t mode) {
  if (pin < PINS) mode_[pin] = mode;
}
void digitalWrite(uint8_t pin, uint8_t val) {
  if (pin >= PINS) return;
  level[pin] = val != 0;
  spi_select(pin, val != 0);
}
int digitalRead(uint8_t pin) { return pin < PINS ? level[pin] : 0; }
uint16_t analogRead(uint8_t pin) {
  advance(ADC_READ_NS);
  return pin < PINS ? adc[pin] : 0;
}
void analogReadResolution(uint8_t bits) { (void)bits; }
double ledcSetup(uint8_t ch, double freq, uint8_t bits) {
  if (ch >= LEDC_CHANNELS || bits < 1 || bits > 20) return 0;
  ledc_bits[ch] = bits, ledc_freq[ch] = freq;
  return freq;
}
void ledcAttachPin(uint8_t pin, uint8_t ch) {
  if (pin < PINS && ch < LEDC_CHANNELS) pin_channel[pin] = (int8_t)ch;
}
void ledcDetachPin(uint8_t pin) {
  if (pin < PINS) pin_channel[pin] = -1;
}
void ledcWrite(uint8_t ch, uint32_t duty) {
  if (ch < LEDC_CHANNELS) ledc_duty[ch] = duty;
}
// Interrupts on pins: none of its pins change by themselves here (its encoders count in PCNT units).
void attachInterrupt(uint8_t pin, void (*isr)(void), int mode) { (void)pin, (void)isr, (void)mode; }
void detachInterrupt(uint8_t pin) { (void)pin; }

// ---- SPI and the LSM6DSR ----------------------------------------------------------------------------------------
SPIClass SPI(VSPI);
TwoWire Wire;
static uint32_t spi_clock = 1000000;
static int imu_cs = -1;
static uint8_t imu_selected, imu_first, imu_addr, imu_reading, regs[128];
static double imu_gyro_dps[3], imu_accel_g[3], imu_gyro_bias[3], imu_gyro_noise, imu_accel_noise;
static int16_t imu_out[7];             // temperature, gyro x y z, accel x y z: the output registers
static uint64_t imu_sample = ~0ull;

static double gyro_mdps_lsb(void) {   // CTRL2_G: FS_4000 (bit 0), FS_125 (bit 1), FS_G (bits 3:2)
  const uint8_t v = regs[0x11];
  if (v & 1) return 140.0;
  if (v & 2) return 4.375;
  static const double fs[4] = {8.75, 17.5, 35.0, 70.0};
  return fs[(v >> 2) & 3];
}
static double accel_mg_lsb(void) {    // CTRL1_XL: FS_XL (bits 3:2): 2, 16, 4, 8 g
  static const double fs[4] = {0.061, 0.488, 0.122, 0.244};
  return fs[(regs[0x10] >> 2) & 3];
}
static double odr_hz(uint8_t ctrl) {  // ODR bits 7:4: 12.5 Hz times 2^(n-1), 1.6 Hz for the accelerometer's 11
  const uint8_t n = ctrl >> 4;
  if (n == 0) return 0;
  if (n == 11) return 1.6;
  return 12.5 * (1 << (n - 1));
}
static int16_t saturate(double v) { return (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : lround(v)); }
static void imu_latch(void) {         // a new sample at each period of the gyro's output data rate
  const double odr = odr_hz(regs[0x11]) > 0 ? odr_hz(regs[0x11]) : odr_hz(regs[0x10]);
  const uint64_t k = odr > 0 ? (uint64_t)(now_ns * 1e-9 * odr) : 0;
  if (k == imu_sample) return;
  imu_sample = k;
  imu_out[0] = 0;   // 25 C
  for (int i = 0; i < 3; i++) {
    imu_out[1 + i] = odr_hz(regs[0x11]) > 0 ? saturate((imu_gyro_dps[i] + imu_gyro_bias[i] + imu_gyro_noise * gauss()) * 1000.0 / gyro_mdps_lsb()) : 0;
    imu_out[4 + i] = odr_hz(regs[0x10]) > 0 ? saturate((imu_accel_g[i] + imu_accel_noise * gauss()) * 1000.0 / accel_mg_lsb()) : 0;
  }
}
static uint8_t imu_read(uint8_t a) {
  if (a == 0x0F) return 0x6B;          // WHO_AM_I
  if (a == 0x1E) return 0x07;          // STATUS_REG: new data
  if (a >= 0x20 && a <= 0x2D) {        // OUT_TEMP, OUTX..Z_G, OUTX..Z_A
    if (a == 0x20 || a == 0x22 || a == 0x28) imu_latch();   // block data update: the sample of the first byte read
    const uint16_t v = (uint16_t)imu_out[(a - 0x20) / 2];
    return (a & 1) ? (uint8_t)(v >> 8) : (uint8_t)v;
  }
  return regs[a & 0x7F];
}
static void spi_select(uint8_t pin, uint8_t high) {
  if ((int)pin != imu_cs) return;
  imu_selected = !high;
  imu_first = 1;
}
void SPIClass::beginTransaction(SPISettings s) {
  spi_clock = s.clock ? s.clock : clock;
  advance(SPI_TRANSACTION_NS);
}
uint8_t SPIClass::transfer(uint8_t data) {
  advance(8000000000ull / spi_clock);
  if (!imu_selected) return 0xFF;
  if (imu_first) {
    imu_first = 0;
    imu_addr = data & 0x7F;
    imu_reading = (data & 0x80) != 0;
    return 0;
  }
  uint8_t out = 0;
  if (imu_reading) out = imu_read(imu_addr);
  else regs[imu_addr] = data;
  imu_addr = (imu_addr + 1) & 0x7F;   // IF_INC
  return out;
}

// ---- Encoders (PCNT) --------------------------------------------------------------------------------------------
puType ESP32Encoder::useInternalWeakPullResistors = puType::down;
struct Wheel {
  int pin_a;
  double counts, rate;   // full-quadrature counts at t0, counts a nanosecond
  uint64_t t0;
};
static Wheel wheels[4];
static int nwheels;
int64_t ESP32Encoder::getCount(void) {
  for (int i = 0; i < nwheels; i++) {
    if (wheels[i].pin_a != pin_a) continue;
    const double c = wheels[i].counts + wheels[i].rate * (double)(now_ns - wheels[i].t0);
    const double scale = type == full ? 1.0 : type == half ? 0.5 : 0.25;
    return (int64_t)floor(c * scale) - offset;
  }
  return -offset;
}
int64_t ESP32Encoder::setCount(int64_t value) {
  offset = 0;
  offset = getCount() - value;
  return value;
}

// ---- NeoPixels --------------------------------------------------------------------------------------------------
static uint8_t shown[16][3];
static uint16_t nshown;
Adafruit_NeoPixel::Adafruit_NeoPixel(uint16_t n, int16_t pin, uint16_t type) : n(n), pin(pin) {
  (void)type;
  rgb = (uint8_t *)calloc(n ? n : 1, 3);
}
Adafruit_NeoPixel::~Adafruit_NeoPixel() { free(rgb); }
void Adafruit_NeoPixel::setPixelColor(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
  if (i >= n) return;
  rgb[3 * i] = r, rgb[3 * i + 1] = g, rgb[3 * i + 2] = b;
}
void Adafruit_NeoPixel::fill(uint32_t c, uint16_t first, uint16_t count) {
  const uint16_t end = count ? first + count : n;
  for (uint16_t i = first; i < end && i < n; i++) setPixelColor(i, c);
}
void Adafruit_NeoPixel::clear(void) { memset(rgb, 0, 3 * n); }
uint32_t Adafruit_NeoPixel::getPixelColor(uint16_t i) const {
  return i < n ? Color(rgb[3 * i], rgb[3 * i + 1], rgb[3 * i + 2]) : 0;
}
void Adafruit_NeoPixel::show(void) {   // its brightness scales what is sent (Adafruit scales the stored values)
  nshown = n < 16 ? n : 16;
  for (uint16_t i = 0; i < nshown; i++)
    for (int c = 0; c < 3; c++) shown[i][c] = (uint8_t)((rgb[3 * i + c] * (brightness + 1u)) >> 8);
  advance((uint64_t)n * 24 * NEOPIXEL_BIT_NS + NEOPIXEL_RESET_NS);
}

// ---- Serial -----------------------------------------------------------------------------------------------------
HardwareSerial Serial;
static char tx[1 << 17];
static uint32_t tx_len;
static char rx[4096];
static uint32_t rx_head, rx_len;
size_t HardwareSerial::write(uint8_t c) {
  if (tx_len < sizeof(tx)) tx[tx_len++] = (char)c;
  return 1;
}
int HardwareSerial::available(void) { return (int)rx_len; }
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
size_t Print::printf(const char *fmt, ...) {
  char b[1024];
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  return write((const uint8_t *)b, n < (int)sizeof(b) ? (size_t)n : sizeof(b) - 1);
}

// ---- Simulator API ----------------------------------------------------------------------------------------------
extern "C" {
int gsim_boot(uint64_t seed) {
  rng ^= seed * 0x2545F4914F6CDD1Dull;
  memset(pin_channel, -1, sizeof(pin_channel));
  fw_stack = (char *)malloc(STACK_BYTES);
  if (!fw_stack) return 0;
  getcontext(&fw_ctx);
  fw_ctx.uc_stack.ss_sp = fw_stack;
  fw_ctx.uc_stack.ss_size = STACK_BYTES;
  fw_ctx.uc_link = NULL;
  makecontext(&fw_ctx, fw_entry, 0);
  return 1;
}
// Runs the sketch until this time (ns since it was switched on).
uint8_t gsim_run_until(uint64_t t_ns) {
  if (state != GSIM_RUNNING) return state;
  until_ns = t_ns;
  if (now_ns < until_ns) swapcontext(&sim_ctx, &fw_ctx);
  return state;
}
uint64_t gsim_time_ns(void) { return now_ns; }
void gsim_set_encoder(int pin_a, double counts, double counts_per_ns) {   // full-quadrature counts, now
  int i = 0;
  while (i < nwheels && wheels[i].pin_a != pin_a) i++;
  if (i == nwheels) {
    if (nwheels == 4) return;
    nwheels++;
  }
  wheels[i] = Wheel{pin_a, counts, counts_per_ns, now_ns};
}
void gsim_imu(int cs_pin, double gyro_bias_x, double gyro_bias_y, double gyro_bias_z, double gyro_noise_dps,
              double accel_noise_g) {
  imu_cs = cs_pin;
  imu_gyro_bias[0] = gyro_bias_x, imu_gyro_bias[1] = gyro_bias_y, imu_gyro_bias[2] = gyro_bias_z;
  imu_gyro_noise = gyro_noise_dps, imu_accel_noise = accel_noise_g;
}
void gsim_set_imu(double gx, double gy, double gz, double ax, double ay, double az) {   // dps, g: the sensor's axes
  imu_gyro_dps[0] = gx, imu_gyro_dps[1] = gy, imu_gyro_dps[2] = gz;
  imu_accel_g[0] = ax, imu_accel_g[1] = ay, imu_accel_g[2] = az;
}
void gsim_set_adc(int pin, uint16_t value) {
  if (pin >= 0 && pin < PINS) adc[pin] = value;
}
void gsim_set_pin(int pin, int high) {
  if (pin >= 0 && pin < PINS) level[pin] = high != 0;
}
// A pin's output: its LEDC duty (0..1) if a channel drives it, else its level.
double gsim_pin(int pin) {
  if (pin < 0 || pin >= PINS) return 0;
  const int ch = pin_channel[pin];
  if (ch < 0) return level[pin];
  const double d = (double)ledc_duty[ch] / (double)(1u << ledc_bits[ch]);
  return d > 1 ? 1 : d;
}
uint32_t gsim_led(int i) {   // the colour a NeoPixel shows, 0xRRGGBB
  return i >= 0 && i < nshown ? ((uint32_t)shown[i][0] << 16) | ((uint32_t)shown[i][1] << 8) | shown[i][2] : 0;
}
uint32_t gsim_serial_tx(char *buffer, uint32_t size) {
  const uint32_t n = tx_len < size ? tx_len : size;
  memcpy(buffer, tx, n);
  memmove(tx, tx + n, tx_len - n);
  tx_len -= n;
  return n;
}
void gsim_serial_rx(const char *data, uint32_t n) {
  for (uint32_t i = 0; i < n && rx_len < sizeof(rx); i++, rx_len++) rx[(rx_head + rx_len) % sizeof(rx)] = data[i];
}
}
