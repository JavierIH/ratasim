// Bulebule's chip (STM32F103C8) simulated: its main() in a coroutine and, every simulated millisecond, its
// interrupts as the chip runs them: TIM1's update 16 times (its emitters' state machine, detection.c: each
// phototransistor read dark, its emitter lit, read lit), DMA1's transfer complete (USART3's TX), then SysTick
// (mmlib's clock, sensors, gyro, encoders, control, logging); then main() until it waits. What its code touches
// through libopencm3 (fakef1.h): the emitters' pins and the ADC's injected group they light, ADC2's battery reading,
// the encoders' counters (TIM2 left, TIM4 right), the motor driver's inputs (TIM3: OC1/OC2 left, OC3/OC4 right),
// the MPU-6500 on SPI2 (its chip select PB12), USART3 through DMA, the flash page that keeps its maze, the user
// button (PA12) and its LEDs. Python (bulebule/robot.py) drives it with the bsim_* functions at the end.
//
// Time: main() yields every YIELD_CALLS calls of what its waiting loops poll (the cycle counter, the encoders'
// distance, the ideal speed, the front distances, the button: bwrap.c) and in mmlib's sleeps (bclock.c);
// computation takes no simulated time.
//
// Addresses: its code hands DMA its buffers as 32-bit addresses ((uint32_t)data): they are statics of this library,
// so their upper half is the library's. Its maze lives in flash at 0x0800FC00, a page mapped there in this process.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <ucontext.h>
#include "fakef1.h"

#define YIELD_CALLS 8
#define STACK_BYTES (1024 * 1024)
#define CYCLES_PER_MS 72000u
#define TIM1_PER_MS 16                  // its emitters' timer: 16 kHz
#define FLASH_PAGE 0x0800FC00u          // FLASH_EEPROM_ADDRESS_MAZE
#define FLASH_PAGE_BYTES 1024u
enum { BSIM_RUNNING = 0, BSIM_HALTED = 2 };

extern int firmware_main(void);
extern void sys_tick_handler(void);
extern void tim1_up_isr(void);
extern void dma1_channel2_isr(void);
extern void dma1_channel3_isr(void);
extern void usart3_isr(void);

static ucontext_t sim_ctx, fw_ctx;
static char *fw_stack;
static uint32_t tick, cycles;
static uint8_t in_isr, state;
static unsigned calls;

static void yield(void) {
  calls = 0;
  swapcontext(&fw_ctx, &sim_ctx);
}
void bsim_poll(void) {
  if (!in_isr && ++calls >= YIELD_CALLS) yield();
}
void bsim_wait_tick(void) {
  if (!in_isr) yield();
}
uint32_t bsim_cycles(void) { return cycles; }
static void fw_entry(void) {
  firmware_main();
  state = BSIM_HALTED;
  for (;;) yield();
}

uint32_t rcc_apb1_frequency = 36000000, rcc_apb2_frequency = 72000000, rcc_ahb_frequency = 72000000;
uint32_t dwt_read_cycle_counter(void) {
  bsim_poll();
  return cycles;
}
static bool systick_on;
void systick_interrupt_enable(void) { systick_on = true; }
void systick_interrupt_disable(void) { systick_on = false; }

// ---- GPIO: outputs, the button's input ----
static uint16_t odr[3], idr[3];
static void mpu_select(bool selected);
void gpio_set(uint32_t port, uint16_t pins) {
  odr[port % 3] |= pins;
  if (port == GPIOB && (pins & GPIO12)) mpu_select(false);
}
void gpio_clear(uint32_t port, uint16_t pins) {
  odr[port % 3] &= (uint16_t)~pins;
  if (port == GPIOB && (pins & GPIO12)) mpu_select(true);
}
void gpio_toggle(uint32_t port, uint16_t pins) { odr[port % 3] ^= pins; }
uint16_t gpio_get(uint32_t port, uint16_t pins) { return (uint16_t)((odr[port % 3] | idr[port % 3]) & pins); }

// ---- Timers: TIM1's update interrupt, TIM3's compare values, the encoders' counters ----
static uint32_t oc[5][4];
static uint16_t cnt[5];
static uint32_t dier[5], sr[5];
void timer_set_oc_value(uint32_t tim, enum tim_oc_id id, uint32_t value) {
  if (tim < 5 && (unsigned)id < 4) oc[tim][id] = value;
}
uint32_t timer_get_counter(uint32_t tim) { return tim < 5 ? cnt[tim] : 0; }
void timer_enable_irq(uint32_t tim, uint32_t irq) { dier[tim % 5] |= irq; }
void timer_disable_irq(uint32_t tim, uint32_t irq) { dier[tim % 5] &= ~irq; }
bool timer_get_flag(uint32_t tim, uint32_t flag) { return (sr[tim % 5] & flag) != 0; }
void timer_clear_flag(uint32_t tim, uint32_t flag) { sr[tim % 5] &= ~flag; }
void rcc_periph_reset_pulse(uint32_t rst) {
  if (rst == RST_TIM1) dier[1] = sr[1] = 0;       // the speaker takes TIM1: the emitters stop until it is done
}

// ---- ADC: ADC1's injected group (the phototransistors: rank k is sensor k-1, detection.c), ADC2 (battery) ----
// Each sensor's input dark and the light its own emitter adds (Python sets them every ms); the emitters' pins as
// detection.c lights them: left side PA9, right side PB8, front left PA8, front right PB9.
static uint16_t ir_dark[4], ir_lit[4], injected[4], battery;
static uint32_t adc_sr[3];
static bool emitter_on(int k) {
  static const uint8_t port[4] = {GPIOA, GPIOB, GPIOA, GPIOB};
  static const uint16_t pin[4] = {GPIO9, GPIO8, GPIO8, GPIO9};
  return (odr[port[k]] & pin[k]) != 0;
}
volatile uint32_t *bsim_adc_sr(uint32_t adc) { return &adc_sr[adc % 3]; }
void adc_start_conversion_injected(uint32_t adc) {
  if (adc != ADC1) return;
  for (int k = 0; k < 4; k++) {
    uint32_t v = ir_dark[k] + (emitter_on(k) ? ir_lit[k] : 0u);
    injected[k] = (uint16_t)(v > 4095 ? 4095 : v);
  }
  adc_sr[1] |= ADC_SR_JEOC;
}
uint32_t adc_read_injected(uint32_t adc, uint8_t reg) {
  return (adc == ADC1 && reg >= 1 && reg <= 4) ? injected[reg - 1] : 0;
}
void adc_start_conversion_direct(uint32_t adc) { adc_sr[adc % 3] |= ADC_SR_EOC; }
uint32_t adc_read_regular(uint32_t adc) { return adc == ADC2 ? battery : 0; }

// ---- MPU-6500 on SPI2: its address byte (bit 7: read), then one data byte ----
#define MPU_GYRO_CONFIG 27
#define MPU_GYRO_ZOUT_H 71
#define MPU_GYRO_ZOUT_L 72
#define MPU_Z_OFFS_USR_H 23
#define MPU_Z_OFFS_USR_L 24
#define MPU_WHOAMI 117
static uint8_t mpu_reg[128], mpu_addr, mpu_phase;
static uint16_t spi_rx;
static int16_t gyro_z, gyro_latched;       // the gyro's output now (LSB at its range), its last read
static void mpu_select(bool selected) {
  if (!selected) mpu_phase = 0;
}
static uint8_t mpu_read(uint8_t a) {
  switch (a) {
  case MPU_GYRO_ZOUT_H: {
    // its user offset (its register map: 4 / 2^FS_SEL LSB per unit; its firmware writes -2x its mean at 2000 dps)
    int32_t offs = (int16_t)(mpu_reg[MPU_Z_OFFS_USR_H] << 8 | mpu_reg[MPU_Z_OFFS_USR_L]);
    int32_t v = gyro_z + offs * 4 / (1 << ((mpu_reg[MPU_GYRO_CONFIG] >> 3) & 3));
    gyro_latched = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
    return (uint8_t)((uint16_t)gyro_latched >> 8);
  }
  case MPU_GYRO_ZOUT_L: return (uint8_t)gyro_latched;
  case MPU_WHOAMI: return 0x70;
  default: return mpu_reg[a & 127];
  }
}
void spi_send(uint32_t spi, uint16_t data) {
  (void)spi;
  if (mpu_phase == 0) {
    mpu_addr = (uint8_t)data;
    mpu_phase = 1;
    spi_rx = 0;
  } else {
    if (mpu_addr & 0x80) {
      spi_rx = mpu_read(mpu_addr & 0x7F);
    } else {
      mpu_reg[mpu_addr & 0x7F] = (uint8_t)data;
      if ((mpu_addr & 0x7F) == 107 && (data & 0x80)) {      // PWR_MGMT_1's reset: its registers to defaults
        memset(mpu_reg, 0, sizeof(mpu_reg));
      }
      spi_rx = 0;
    }
    mpu_addr = (uint8_t)((mpu_addr & 0x80) | ((mpu_addr + 1) & 0x7F));
  }
}
uint16_t spi_read(uint32_t spi) {
  (void)spi;
  return spi_rx;
}

// ---- DMA1 and USART3 ----
struct dma_ch { uint32_t addr; uint16_t n, left; bool from_memory, enabled, tcie, tcif; };
static struct dma_ch dma[8];
static uint32_t usart_sr, usart_dr, usart_cr1;
static bool tx_dma, rx_dma;
static char tx[1 << 16];
static uint32_t tx_len;
static bool tx_done_pending;
static char anchor;

// A 32-bit address its code took of one of its statics: the library's upper half put back on it.
static char *host_pointer(uint32_t addr) {
  char *p = (char *)(((uintptr_t)&anchor & ~(uintptr_t)0xFFFFFFFFu) | addr);
  Dl_info a, b;
  if (!dladdr(&anchor, &a) || !dladdr(p, &b) || a.dli_fbase != b.dli_fbase) {
    fprintf(stderr, "bchip: DMA address %08x is not one of the firmware's statics\n", addr);
    return NULL;
  }
  return p;
}
void dma_channel_reset(uint32_t d, uint8_t ch) { (void)d; memset(&dma[ch & 7], 0, sizeof(dma[0])); }
void dma_set_memory_address(uint32_t d, uint8_t ch, uint32_t a) { (void)d; dma[ch & 7].addr = a; }
void dma_set_number_of_data(uint32_t d, uint8_t ch, uint16_t n) { (void)d; dma[ch & 7].n = dma[ch & 7].left = n; }
void dma_set_read_from_memory(uint32_t d, uint8_t ch) { (void)d; dma[ch & 7].from_memory = true; }
void dma_set_read_from_peripheral(uint32_t d, uint8_t ch) { (void)d; dma[ch & 7].from_memory = false; }
void dma_enable_channel(uint32_t d, uint8_t ch) { (void)d; dma[ch & 7].enabled = true; }
void dma_disable_channel(uint32_t d, uint8_t ch) { (void)d; dma[ch & 7].enabled = false; }
void dma_enable_transfer_complete_interrupt(uint32_t d, uint8_t ch) { (void)d; dma[ch & 7].tcie = true; }
void dma_disable_transfer_complete_interrupt(uint32_t d, uint8_t ch) { (void)d; dma[ch & 7].tcie = false; }
bool dma_get_interrupt_flag(uint32_t d, uint8_t ch, uint32_t f) { (void)d; return (f & DMA_TCIF) && dma[ch & 7].tcif; }
void dma_clear_interrupt_flags(uint32_t d, uint8_t ch, uint32_t f) {
  (void)d;
  if (f & DMA_TCIF) dma[ch & 7].tcif = false;
}
volatile uint32_t *bsim_usart_reg(uint32_t usart, uint32_t offset) {
  (void)usart;
  if (offset == 0x04) {               // reading DR after SR clears IDLE
    usart_sr &= ~USART_SR_IDLE;
    return &usart_dr;
  }
  return offset == 0x0C ? &usart_cr1 : &usart_sr;
}
void usart_enable_tx_dma(uint32_t usart) {    // channel 2 sends its buffer; complete at the next millisecond
  (void)usart;
  tx_dma = true;
  struct dma_ch *c = &dma[2];
  if (!c->enabled || !c->from_memory || !c->left) return;
  char *p = host_pointer(c->addr);
  for (uint32_t i = 0; p && i < c->left && tx_len < sizeof(tx); i++) tx[tx_len++] = p[i];
  c->left = 0;
  tx_done_pending = true;
}
void usart_disable_tx_dma(uint32_t usart) { (void)usart; tx_dma = false; }
void usart_enable_rx_dma(uint32_t usart) { (void)usart; rx_dma = true; }
void usart_disable_rx_dma(uint32_t usart) { (void)usart; rx_dma = false; }
int _putchar(char c) {
  if (tx_len < sizeof(tx)) tx[tx_len++] = c;
  return c;
}

// ---- Flash: its maze's page at its own address ----
static uint8_t *flash;
static uint32_t flash_erases;
static int flash_map(void) {
  uintptr_t base = FLASH_PAGE & ~0xFFFu;
  void *p = mmap((void *)base, 4096, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
  if (p == MAP_FAILED) {
    // already mapped: another of its boots in this process (its maze shared with it)
    unsigned char vec;
    if (mincore((void *)base, 4096, &vec) != 0) return 0;
  } else if ((uintptr_t)p != base) {
    munmap(p, 4096);
    return 0;
  } else {
    memset((void *)FLASH_PAGE, 0xFF, FLASH_PAGE_BYTES);
  }
  flash = (uint8_t *)(uintptr_t)FLASH_PAGE;
  return 1;
}
void flash_unlock(void) {}
void flash_erase_page(uint32_t a) {
  if (a >= FLASH_PAGE && a < FLASH_PAGE + FLASH_PAGE_BYTES) {
    memset(flash, 0xFF, FLASH_PAGE_BYTES);
    flash_erases++;
  }
}
uint32_t flash_get_status_flags(void) { return FLASH_SR_EOP; }
void flash_program_word(uint32_t a, uint32_t data) {
  if (a >= FLASH_PAGE && a + 4 <= FLASH_PAGE + FLASH_PAGE_BYTES) memcpy(flash + (a - FLASH_PAGE), &data, 4);
}

// ---- Simulator API ----
int bsim_boot(void) {
  if (!flash_map()) return 0;
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

uint8_t bsim_step(void) {
  if (state != BSIM_RUNNING) return state;
  tick++;
  in_isr = 1;
  for (int i = 0; i < TIM1_PER_MS; i++) {
    cycles += CYCLES_PER_MS / TIM1_PER_MS;
    if (dier[1] & TIM_DIER_UIE) {
      sr[1] |= TIM_SR_UIF;
      tim1_up_isr();
    }
  }
  if (tx_done_pending) {
    tx_done_pending = false;
    dma[2].tcif = true;
    if (dma[2].tcie) dma1_channel2_isr();
  }
  if (systick_on) sys_tick_handler();
  in_isr = 0;
  swapcontext(&sim_ctx, &fw_ctx);
  return state;
}

uint32_t bsim_time_ms(void) { return tick; }
void bsim_set_encoders(int32_t left, int32_t right) {      // counts, forward positive
  cnt[TIM2] = (uint16_t)left;
  cnt[TIM4] = (uint16_t)right;
}
void bsim_set_ir(uint8_t k, uint16_t dark, uint16_t lit) {   // k: detection.h's sensor id
  if (k < 4) ir_dark[k] = dark, ir_lit[k] = lit;
}
void bsim_set_gyro(int16_t z) { gyro_z = z; }                // LSB at its range, counter-clockwise positive
void bsim_set_battery(uint16_t counts) { battery = counts; }
void bsim_set_button(uint8_t pressed) {
  if (pressed) idr[GPIOA] |= GPIO12;
  else idr[GPIOA] &= (uint16_t)~GPIO12;
}
// The motor driver's inputs (DRV8833-like, slow decay): each side's two compare values; their difference, its
// duty in 1/1024 (forward positive); both low: the driver off (coasting).
void bsim_get_motors(int16_t *left, int16_t *right, uint8_t *on_left, uint8_t *on_right) {
  *left = (int16_t)((int32_t)oc[3][TIM_OC1] - (int32_t)oc[3][TIM_OC2]);
  *right = (int16_t)((int32_t)oc[3][TIM_OC3] - (int32_t)oc[3][TIM_OC4]);
  *on_left = (oc[3][TIM_OC1] | oc[3][TIM_OC2]) != 0;
  *on_right = (oc[3][TIM_OC3] | oc[3][TIM_OC4]) != 0;
}
uint8_t bsim_leds(void) {            // bit 0 left (PB4), bit 1 right (PB5), bit 2 the Blue Pill's (PC13, active low)
  return (uint8_t)(((odr[GPIOB] >> 4) & 1) | ((odr[GPIOB] >> 4) & 2) | ((odr[GPIOC] & GPIO13) ? 0 : 4));
}
uint8_t bsim_speaker(void) { return !(dier[1] & TIM_DIER_UIE); }   // TIM1 given to the speaker
uint8_t *bsim_flash(void) { return flash; }
uint32_t bsim_flash_bytes(void) { return FLASH_PAGE_BYTES; }
uint32_t bsim_flash_erases(void) { return flash_erases; }      // its page erased (before each save)

// Bytes arriving on USART3 (Bluetooth): DMA channel 3 writes them into its receive buffer, then the line goes idle.
void bsim_uart_rx(const char *data, uint32_t n) {
  struct dma_ch *c = &dma[3];
  in_isr = 1;
  for (uint32_t i = 0; i < n; i++) {
    if (!rx_dma || !c->enabled || !c->left) break;
    char *p = host_pointer(c->addr + (c->n - c->left));
    if (!p) break;
    *p = data[i];
    if (--c->left == 0) {
      c->tcif = true;
      if (c->tcie) dma1_channel3_isr();
    }
  }
  usart_sr |= USART_SR_IDLE;
  usart3_isr();
  in_isr = 0;
}
uint32_t bsim_uart_tx(char *buffer, uint32_t size) {
  uint32_t n = tx_len < size ? tx_len : size;
  memcpy(buffer, tx, n);
  memmove(tx, tx + n, tx_len - n);
  tx_len -= n;
  return n;
}
