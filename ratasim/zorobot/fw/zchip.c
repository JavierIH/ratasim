// ZoroBot3's chip, simulated: its main() in a coroutine, its SysTick at 16 kHz (sm_emitter_adc() every tick, its
// clock, buttons, battery, encoders, sensors, gyro and control_loop() each once in 16) each simulated millisecond, and the peripherals its code touches through
// libopencm3 (fakecm3.h). Python (zorobot/robot.py) drives it with the zsim_* functions at the end.
//
// Time: main() yields every YIELD_CALLS calls of the functions its waiting loops poll (get_clock_ticks, the
// cycle counter and the getters wrapped in zwrap.c) and in delay(); computation takes no simulated time.
#define _GNU_SOURCE
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucontext.h>
#include "fakecm3.h"

#define YIELD_CALLS 8
#define STACK_BYTES (1024 * 1024)
#define AMBIENT_COUNTS 150.0f       // the receivers with every emitter off
#define EEPROM_ADDRESS 0x080E0000u  // flash sector 11 (include/eeprom.h)
#define EEPROM_BYTES 4096u
enum { ZSIM_RUNNING = 0, ZSIM_HALTED = 2 };

extern int firmware_main(void);
extern void sys_tick_handler(void);
extern void control_loop(void);
extern void sm_emitter_adc(void);
extern volatile uint16_t *get_aux_adc_raw(void);

static ucontext_t sim_ctx, fw_ctx;
static char *fw_stack;
static uint32_t tick, cycles;
static uint8_t in_isr, state;
static unsigned calls;

static void yield(void) {
  calls = 0;
  swapcontext(&fw_ctx, &sim_ctx);
}
void zsim_poll(void) {
  if (!in_isr && ++calls >= YIELD_CALLS) yield();
}
void zsim_wait_tick(void) {
  if (!in_isr) yield();
}
uint32_t zsim_cycles(void) {
  return cycles;
}
static void fw_entry(void) {
  firmware_main();
  state = ZSIM_HALTED;
  for (;;) yield();
}

// ---- GPIO ----
static uint16_t odr[4];
static void spi_cs(int high);
void gpio_set(uint32_t port, uint16_t pins) {
  odr[port & 3] |= pins;
  if (port == GPIOA && (pins & GPIO15)) spi_cs(1);
}
void gpio_clear(uint32_t port, uint16_t pins) {
  odr[port & 3] &= (uint16_t)~pins;
  if (port == GPIOA && (pins & GPIO15)) spi_cs(0);
}
void gpio_toggle(uint32_t port, uint16_t pins) {
  odr[port & 3] ^= pins;
}
uint16_t gpio_get(uint32_t port, uint16_t pins) {
  return odr[port & 3] & pins;
}

// ---- Timers: PWM compare values (motors TIM8, fan and RGB TIM1) and the encoders' counters (TIM3, TIM4) ----
static uint32_t oc[9][4];
static uint16_t cnt[9];
void timer_set_oc_value(uint32_t tim, enum tim_oc_id id, uint32_t value) {
  if (tim < 9 && (unsigned)id < 4) oc[tim][id] = value;
}
uint32_t timer_get_counter(uint32_t tim) {
  return tim < 9 ? cnt[tim] : 0;
}
void timer_set_counter(uint32_t tim, uint32_t count) {
  if (tim < 9) cnt[tim] = (uint16_t)count;
}
uint32_t rcc_apb1_frequency = 42000000, rcc_apb2_frequency = 84000000, rcc_ahb_frequency = 168000000;
const struct rcc_clock_scale rcc_hse_8mhz_3v3[1];
uint32_t dwt_read_cycle_counter(void) {
  zsim_poll();
  return cycles;
}

// ---- ADC2 injected group: the wall sensors, sampled with whatever emitters are lit ----
static float ir_signal[4];
static uint16_t injected[4];
static const uint16_t EMITTER[4] = {GPIO0, GPIO3, GPIO2, GPIO1};   // FL, FR, SL, SR (sensors.c)
static uint32_t rng = 12345u;
static float noise(void) {    // +-1.5 counts
  rng = rng * 1664525u + 1013904223u;
  return ((float)(rng >> 8) / 16777216.0f - 0.5f) * 3.0f;
}
void adc_start_conversion_injected(uint32_t adc) {
  (void)adc;
  for (int k = 0; k < 4; k++) {
    float v = AMBIENT_COUNTS + ((odr[GPIOA] & EMITTER[k]) ? ir_signal[k] : 0.0f) + noise();
    injected[k] = (uint16_t)(v < 0 ? 0 : v > 4095 ? 4095 : v);
  }
}
uint32_t adc_read_injected(uint32_t adc, uint8_t reg) {
  (void)adc;
  return reg >= 1 && reg <= 4 ? injected[reg - 1] : 0;
}

// ---- SPI3: the LSM6DSR (chip select PA15): a register file, the gyro's Z rate from the simulator ----
static uint8_t spi_on, spi_first, spi_addr, spi_reading, regs[128];
static uint16_t spi_rx;
static float gyro_dps;
static void spi_cs(int high) {
  spi_on = !high;
  spi_first = 1;
}
static float sensitivity_mdps(void) {   // CTRL2_G: FS_4000 bit 0, FS_G bits 3:2
  const uint8_t v = regs[0x11];
  if (v & 1) return 140.0f;
  static const float fs[4] = {8.75f, 17.5f, 35.0f, 70.0f};
  return fs[(v >> 2) & 3];
}
static uint8_t reg_read(uint8_t a) {
  if (a == 0x0F) return 0x6B;   // WHO_AM_I
  if (a == 0x26 || a == 0x27) {
    float raw = gyro_dps * 1000.0f / sensitivity_mdps();
    int16_t r = (int16_t)(raw > 32767 ? 32767 : raw < -32768 ? -32768 : raw);
    return a == 0x26 ? (uint8_t)(r & 0xFF) : (uint8_t)((uint16_t)r >> 8);
  }
  return regs[a & 0x7F];
}
void spi_send(uint32_t spi, uint16_t data) {
  (void)spi;
  if (!spi_on) {
    spi_rx = 0;
    return;
  }
  if (spi_first) {
    spi_addr = data & 0x7F;
    spi_reading = (data & 0x80) != 0;
    spi_first = 0;
    spi_rx = 0;
  } else {
    if (spi_reading) spi_rx = reg_read(spi_addr);
    else regs[spi_addr] = (uint8_t)data;
    spi_addr = (spi_addr + 1) & 0x7F;
  }
}
uint16_t spi_read(uint32_t spi) {
  (void)spi;
  return spi_rx;
}
uint16_t spi_xfer(uint32_t spi, uint16_t data) {   // libopencm3's: a word out, the word in (its firmware since 2026)
  spi_send(spi, data);
  return spi_read(spi);
}

// ---- USART3 (printf) ----
static char tx[1 << 17];
static uint32_t tx_len;
void usart_send_blocking(uint32_t usart, uint16_t data) {
  (void)usart;
  if (tx_len < sizeof(tx)) tx[tx_len++] = (char)data;
}
void usart_send(uint32_t usart, uint16_t data) {
  usart_send_blocking(usart, data);
}
int zsim_printf(const char *fmt, ...) {
  char b[1024];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  for (int i = 0; i < n && i < (int)sizeof(b) - 1; i++) usart_send_blocking(USART3, (uint8_t)b[i]);
  return n;
}

// ---- Flash (the EEPROM sector) and other memory-mapped words (the chip's UID) ----
static uint32_t eeprom[EEPROM_BYTES / 4];
static uint32_t flash_writes;
static uint32_t uid[3] = {0x00470030u, 0x31395111u, 0x36383133u};   // neither B's nor C's: robot version A
static uint32_t scratch;
volatile uint32_t *zsim_mmio32(uint32_t addr) {
  if (addr >= EEPROM_ADDRESS && addr < EEPROM_ADDRESS + EEPROM_BYTES) return &eeprom[(addr - EEPROM_ADDRESS) / 4];
  if (addr >= 0x1FFF7A10u && addr < 0x1FFF7A1Cu) return &uid[(addr - 0x1FFF7A10u) / 4];
  scratch = 0;
  return &scratch;
}
void flash_unlock(void) {}
void flash_lock(void) {}
void flash_erase_sector(uint8_t sector, uint32_t program_size) {
  (void)program_size;
  if (sector == 11) memset(eeprom, 0xFF, sizeof(eeprom));
  flash_writes++;
}
void flash_program_word(uint32_t address, uint32_t data) {
  if (address >= EEPROM_ADDRESS && address < EEPROM_ADDRESS + EEPROM_BYTES) eeprom[(address - EEPROM_ADDRESS) / 4] = data;
  flash_writes++;
}

// ---- Simulator API ----
int zsim_boot(void) {
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

uint8_t zsim_step(void) {
  if (state != ZSIM_RUNNING) return state;
  tick++;
  cycles += 168000u;
  in_isr = 1;
  for (int i = 0; i < 16; i++) sys_tick_handler();   // SysTick at 16 kHz: sm_emitter_adc() each tick, the rest in turn
  in_isr = 0;
  swapcontext(&sim_ctx, &fw_ctx);
  return state;
}

uint32_t zsim_time_ms(void) { return tick; }
void zsim_set_encoders(int32_t left, int32_t right) {   // ticks, forward positive (encoders.c negates the left)
  cnt[4] = (uint16_t)(-left);
  cnt[3] = (uint16_t)right;
}
void zsim_set_ir(uint8_t sensor, float signal_counts) {    // emitter-on minus emitter-off counts
  if (sensor < 4) ir_signal[sensor] = signal_counts;
}
void zsim_set_gyro(float dps_ccw) { gyro_dps = dps_ccw; }
void zsim_set_aux(uint16_t battery, uint16_t button) {
  volatile uint16_t *a = get_aux_adc_raw();
  a[0] = battery;
  a[1] = a[2] = 0;
  a[3] = button;
}
void zsim_get_motors(int16_t *left, int16_t *right, uint8_t *enabled) {   // duty in 1/1024, forward positive
  *left = (int16_t)((int32_t)oc[8][TIM_OC3] - (int32_t)oc[8][TIM_OC4]);
  *right = (int16_t)((int32_t)oc[8][TIM_OC1] - (int32_t)oc[8][TIM_OC2]);
  *enabled = (odr[GPIOB] & GPIO15) != 0;
}
uint32_t zsim_oc(uint8_t tim, uint8_t ch) { return tim < 9 && ch < 4 ? oc[tim][ch] : 0; }
uint16_t zsim_gpio(uint8_t port) { return odr[port & 3]; }
uint32_t zsim_uart_tx(char *buffer, uint32_t size) {
  uint32_t n = tx_len < size ? tx_len : size;
  memcpy(buffer, tx, n);
  memmove(tx, tx + n, tx_len - n);
  tx_len -= n;
  return n;
}
void zsim_flash_load(const uint8_t *data, uint32_t size) {
  memset(eeprom, 0xFF, sizeof(eeprom));
  memcpy(eeprom, data, size < sizeof(eeprom) ? size : sizeof(eeprom));
}
const uint8_t *zsim_flash_image(void) { return (const uint8_t *)eeprom; }
uint32_t zsim_flash_writes(void) { return flash_writes; }
