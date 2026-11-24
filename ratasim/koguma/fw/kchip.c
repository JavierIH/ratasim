// KOGUMA's STM32F446RE (180 MHz) simulated: its main() in a coroutine on a clock of simulated time, its interrupts
// and the part of ST's HAL its firmware calls (include/stm32f4xx_hal.h), its peripherals as host memory:
//
//   SysTick    every millisecond: HAL's tick (HAL_Delay, HAL_GetTick), priority 15
//   TIM6       1 MHz, its update every 1000 counts: its control interrupt (stm32f4xx_it.c), priority 0; its counter
//              read and set as it runs (wait_ms sets it to 0: the next interrupt a millisecond later)
//   ADC1+DMA2  its 5 channels' scan (battery, FR, L, R, FL), 3 + 12 cycles each at 11.25 MHz, then the DMA's
//              interrupt (priority 0, after TIM6's: the same priority does not preempt) and its callback. Each
//              photodiode's reading with its emitter's pin lit or dark: PA1 lights FL and R, PA0 L and FR
//   SPI3       its two AS5047P encoders (CS PC5 left, PA15 right): a frame's answer is the previous frame's
//              command's. Its SPI3 runs in mode 3, the AS5047P in mode 1: its MCU reads each word one bit late,
//              the angle's 13 upper bits (8192 a turn), as its firmware takes them (x 360 / 8192)
//   SPI1       its LSM6DSR (CS PA4): its registers, its gyro and accelerometer sampled at their 1.66 kHz
//   TIM3       its MP6550 drivers' PWM, 50 kHz: CCR1/CCR2 left, CCR3/CCR4 right (both high: brake; a channel
//              stopped: its input low, the motor coasting)
//   TIM2       its fan's PWM (CCR1 of ARR + 1); TIM4 its buzzer (channel 4, 1 MHz / (ARR + 1))
//   USART2     its printf, 115200 baud: each byte its 87 us (its _write sends them one by one, waiting)
//   GPIO       its LEDs and emitters (output data), its switch S2 on PB3 (pressed: low)
//
// Time passes where its firmware waits on the chip: in each pass of a loop (ksim.h: ksim_pass), in its HAL calls
// (what they take on the chip), in HAL_Delay. When it reaches an interrupt's time, the interrupt runs there, in the
// middle of what main() was doing; when it reaches what the simulator asked for, the firmware gives the time back
// to it (ksim_run_until). Computation outside loops takes none.
#define _GNU_SOURCE
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucontext.h>

#include "stm32f4xx_hal.h"

#define NEVER UINT64_MAX
#define PASS_NS 50ull                 // a loop's pass (a few cycles at 180 MHz)
#define CALL_NS 40ull                 // a HAL call that only reads or writes a register
#define SPI_CALL_NS 400ull            // a blocking SPI transfer's own time, besides its bytes'
#define SPI1_BYTE_NS 711ull           // 8 bits at 11.25 MHz (APB2's 90 MHz / 8)
#define SPI3_BYTE_NS 356ull           // 8 bits at 22.5 MHz (APB1's 45 MHz / 2)
#define UART_BYTE_NS 86806ull         // 10 bits at 115200 baud
#define ADC_CLOCK_NS (1e9 / 11.25e6)  // PCLK2 / 8
#define SYSTICK_NS 1000000ull
#define GYRO_ODR_NS 602410ull         // 1.66 kHz
#define STACK_BYTES (8 << 20)
enum { KSIM_RUNNING = 0, KSIM_HALTED = 2 };

GPIO_TypeDef ksim_gpio[8];
TIM_TypeDef ksim_tim[7];
ADC_TypeDef ksim_adc1;
SPI_TypeDef ksim_spi[4];
USART_TypeDef ksim_usart2;
DMA_Stream_TypeDef ksim_dma2_stream0;
RCC_TypeDef ksim_rcc;
DWT_Type ksim_dwt;
CoreDebug_Type ksim_coredebug;
uint32_t SystemCoreClock = 180000000u;

extern int koguma_main(void);
extern void SysTick_Handler(void);
extern void TIM6_DAC_IRQHandler(void);
extern void DMA2_Stream0_IRQHandler(void);

// ---- Time, interrupts ----------------------------------------------------------------------------------------------
enum { IRQ_SYSTICK, IRQ_TIM6, IRQ_DMA, NIRQ };     // in their exception numbers' order (ties: the lower first)
static uint64_t now_ns, until_ns, next_check;
static uint64_t irq_at[NIRQ] = {NEVER, NEVER, NEVER};
static uint8_t irq_pending[NIRQ], irq_on[NIRQ], irq_prio[NIRQ] = {15, 0, 0};
static int running_prio = 256;              // 256: main()
static int irq_masked;
static volatile uint32_t uw_tick;
static ucontext_t sim_ctx, fw_ctx;
static char *fw_stack;
static uint8_t in_fw, state;

static uint64_t tim_base_cyc[7];            // each timer's count origin, in its clock's cycles (90 MHz)
static uint64_t cyc_now(void) { return now_ns * 9 / 100; }
static uint64_t cyc_to_ns(uint64_t c) { return (c * 100 + 8) / 9; }
static int tim_index(TIM_TypeDef *t) { return (int)(t - ksim_tim); }

static void tim_sync(int i) {
  TIM_TypeDef *t = &ksim_tim[i];
  if (!(t->CR1 & TIM_CR1_CEN)) return;
  uint64_t c = cyc_now();
  uint64_t ticks = c > tim_base_cyc[i] ? (c - tim_base_cyc[i]) / (t->PSC + 1) : 0;
  t->CNT = (uint32_t)(ticks % ((uint64_t)t->ARR + 1));
}
static void timers_sync(void) {
  tim_sync(2);
  tim_sync(3);
  tim_sync(6);
}

static void tim6_schedule(void) {
  TIM_TypeDef *t = TIM6;
  if (!(t->CR1 & TIM_CR1_CEN) || !(t->DIER & TIM_DIER_UIE)) {
    irq_at[IRQ_TIM6] = NEVER;
    return;
  }
  uint64_t period = ((uint64_t)t->PSC + 1) * ((uint64_t)t->ARR + 1);
  uint64_t c = cyc_now() - tim_base_cyc[6];
  irq_at[IRQ_TIM6] = cyc_to_ns(tim_base_cyc[6] + (c / period + 1) * period);
}

static void recheck(void) {
  next_check = until_ns;
  for (int i = 0; i < NIRQ; i++)
    if (irq_at[i] < next_check) next_check = irq_at[i];
}

static void dma_transfer(void);

// what has come due: pending (an interrupt's flag holds one, however many times it came while masked)
static void latch(void) {
  for (int i = 0; i < NIRQ; i++) {
    if (irq_at[i] > now_ns) continue;
    irq_pending[i] = 1;
    if (i == IRQ_SYSTICK) {
      while (irq_at[i] <= now_ns) irq_at[i] += SYSTICK_NS;
    } else if (i == IRQ_TIM6) {
      TIM6->SR |= TIM_SR_UIF;
      tim6_schedule();
    } else {
      irq_at[i] = NEVER;
      dma_transfer();
    }
  }
}

static void run_irq(int i) {
  int saved = running_prio;
  running_prio = irq_prio[i];
  irq_pending[i] = 0;
  timers_sync();
  if (i == IRQ_SYSTICK) SysTick_Handler();
  else if (i == IRQ_TIM6) TIM6_DAC_IRQHandler();
  else DMA2_Stream0_IRQHandler();
  running_prio = saved;
}

static void service(void) {
  for (;;) {
    latch();
    if (irq_masked) return;
    int best = -1;
    for (int i = 0; i < NIRQ; i++)
      if (irq_pending[i] && irq_on[i] && irq_prio[i] < running_prio && (best < 0 || irq_prio[i] < irq_prio[best]))
        best = i;
    if (best < 0) return;
    run_irq(best);
  }
}

static void maybe_yield(void) {
  while (in_fw && now_ns >= until_ns) swapcontext(&fw_ctx, &sim_ctx);
}

static void checkpoint(void) {
  service();
  maybe_yield();
  recheck();
}

// time passes (what the chip spends); the interrupts that come in it run where they come
static void advance(uint64_t ns) {
  uint64_t end = now_ns + ns;
  for (;;) {
    if (now_ns >= next_check) checkpoint();
    if (now_ns >= end) break;
    now_ns = next_check < end ? next_check : end;
  }
  if (running_prio < 256) timers_sync();
  ksim_dwt.CYCCNT = (uint32_t)(now_ns * 9 / 50);
}

void ksim_pass(void) {
  now_ns += PASS_NS;
  if (now_ns >= next_check) checkpoint();
  if (running_prio < 256) timers_sync();
  ksim_dwt.CYCCNT = (uint32_t)(now_ns * 9 / 50);
}

void __disable_irq(void) { irq_masked = 1; }
void __enable_irq(void) {
  irq_masked = 0;
  service();
  recheck();
}

// ---- Noise -----------------------------------------------------------------------------------------------------------
static uint64_t rng = 0x9E3779B97F4A7C15ull;
static double uniform(void) {
  rng ^= rng << 13;
  rng ^= rng >> 7;
  rng ^= rng << 17;
  return ((rng >> 11) + 0.5) / 9007199254740992.0;
}
static double gauss(void) { return sqrt(-2.0 * log(uniform())) * cos(2 * M_PI * uniform()); }

// ---- The world, from the simulator ----------------------------------------------------------------------------------
static struct { double angle, rate; uint64_t t0; } encoder[2];      // its shaft (rad), rad a ns: 0 left, 1 right
static double ir_signal[4], ir_dark[4] = {40, 40, 40, 40};         // ADC counts, its index: FL, R, L, FR
static double ir_noise = 2.0;
static double gyro_dps, accel_g[3] = {0, 0, 1}, gyro_noise_dps = 0.1;
static double battery_v = 16.0;
static int switch_pressed;

// ---- GPIO -------------------------------------------------------------------------------------------------------------
static void gpio_changed(GPIO_TypeDef *port, uint16_t pins, uint32_t before);

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) {
  for (int pin = 0; pin < 16; pin++) {
    if (!(init->Pin & (1u << pin))) continue;
    port->MODER = (port->MODER & ~(3u << 2 * pin)) | ((init->Mode & 3u) << 2 * pin);
    port->PUPDR = (port->PUPDR & ~(3u << 2 * pin)) | ((init->Pull & 3u) << 2 * pin);
  }
}
void HAL_GPIO_DeInit(GPIO_TypeDef *port, uint32_t pins) { (void)port, (void)pins; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
  advance(CALL_NS);
  if (port == GPIOB) port->IDR = (port->IDR & ~(uint32_t)GPIO_PIN_3) | (switch_pressed ? 0 : GPIO_PIN_3);
  return (port->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState s) {
  advance(CALL_NS);
  uint32_t before = port->ODR;
  if (s) port->ODR |= pins;
  else port->ODR &= ~(uint32_t)pins;
  gpio_changed(port, pins, before);
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pins) {
  advance(CALL_NS);
  uint32_t before = port->ODR;
  port->ODR ^= pins;
  gpio_changed(port, pins, before);
}

// ---- Core, clocks, NVIC ---------------------------------------------------------------------------------------------
HAL_StatusTypeDef HAL_Init(void) {
  HAL_MspInit();
  irq_on[IRQ_SYSTICK] = 1;
  irq_at[IRQ_SYSTICK] = now_ns + SYSTICK_NS;
  recheck();
  return HAL_OK;
}
__attribute__((weak)) void HAL_MspInit(void) {}
void HAL_IncTick(void) { uw_tick++; }
uint32_t HAL_GetTick(void) {
  advance(CALL_NS / 2);
  return uw_tick;
}
void HAL_Delay(uint32_t ms) {
  uint32_t t0 = HAL_GetTick();
  uint32_t wait = ms < HAL_MAX_DELAY ? ms + 1 : ms;
  while (uw_tick - t0 < wait) advance(irq_at[IRQ_SYSTICK] > now_ns ? irq_at[IRQ_SYSTICK] - now_ns : PASS_NS);
}
static int irq_slot(IRQn_Type irq) {
  return irq == SysTick_IRQn ? IRQ_SYSTICK : irq == TIM6_DAC_IRQn ? IRQ_TIM6 : irq == DMA2_Stream0_IRQn ? IRQ_DMA : -1;
}
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub) {
  (void)sub;
  int i = irq_slot(irq);
  if (i >= 0) irq_prio[i] = (uint8_t)preempt;
}
void HAL_NVIC_EnableIRQ(IRQn_Type irq) {
  int i = irq_slot(irq);
  if (i >= 0) irq_on[i] = 1;
}
void HAL_NVIC_DisableIRQ(IRQn_Type irq) {
  int i = irq_slot(irq);
  if (i >= 0) irq_on[i] = 0;
}
void HAL_NVIC_SetPriorityGrouping(uint32_t group) { (void)group; }
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *init) { (void)init; return HAL_OK; }
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *init, uint32_t latency) {
  (void)init, (void)latency;
  SystemCoreClock = 180000000u;
  return HAL_OK;
}
uint32_t HAL_RCC_GetHCLKFreq(void) { return SystemCoreClock; }
HAL_StatusTypeDef HAL_PWREx_EnableOverDrive(void) { return HAL_OK; }

// ---- DMA, ADC1 --------------------------------------------------------------------------------------------------------
static ADC_HandleTypeDef *adc_handle;
static uint8_t adc_rank[16], adc_ranks;
static uint16_t *dma_dst;
static uint32_t dma_len;
static uint64_t adc_single_done;

HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma) { (void)hdma; return HAL_OK; }
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma) { (void)hdma; return HAL_OK; }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma) {
  if (hdma->Parent) HAL_ADC_ConvCpltCallback((ADC_HandleTypeDef *)hdma->Parent);
}

// a channel's conversion (counts): the battery through its divider (47k / 10k, 3.3 V), each photodiode dark
// or lit by its emitter
static uint16_t adc_sample(int ch) {
  double v;
  if (ch == 9) {
    v = battery_v * 10.0 / 57.0 / 3.3 * 4095.0;
  } else if (ch >= 10 && ch <= 13) {
    static const int sensor[4] = {3, 2, 1, 0};                     // ch10 FR, ch11 L, ch12 R, ch13 FL
    static const uint16_t emitter[4] = {GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_1};
    int s = sensor[ch - 10];
    v = ir_dark[s] + ((GPIOA->ODR & emitter[ch - 10]) ? ir_signal[s] : 0.0) + ir_noise * gauss();
  } else {
    v = 0;
  }
  return (uint16_t)(v < 0 ? 0 : v > 4095 ? 4095 : lround(v));
}

static uint64_t adc_conversion_ns(int ch) {
  static const int cycles[8] = {3, 15, 28, 56, 84, 112, 144, 480};
  uint32_t smp = ch >= 10 ? (ADC1->SMPR1 >> 3 * (ch - 10)) & 7 : (ADC1->SMPR2 >> 3 * ch) & 7;
  return (uint64_t)((cycles[smp] + 12) * ADC_CLOCK_NS);
}

static void dma_transfer(void) {
  for (uint32_t k = 0; k < dma_len && dma_dst; k++) dma_dst[k] = adc_sample(adc_rank[k % (adc_ranks ? adc_ranks : 1)]);
}

HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc) {
  adc_handle = hadc;
  HAL_ADC_MspInit(hadc);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *c) {
  (void)hadc;
  if (c->Rank >= 1 && c->Rank <= 16) {
    adc_rank[c->Rank - 1] = (uint8_t)c->Channel;
    if (c->Rank > adc_ranks) adc_ranks = (uint8_t)c->Rank;
    if (c->Channel >= 10) ADC1->SMPR1 = (ADC1->SMPR1 & ~(7u << 3 * (c->Channel - 10))) | (c->SamplingTime << 3 * (c->Channel - 10));
    else ADC1->SMPR2 = (ADC1->SMPR2 & ~(7u << 3 * c->Channel)) | (c->SamplingTime << 3 * c->Channel);
  }
  return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  advance(CALL_NS * 10);
  adc_single_done = now_ns + adc_conversion_ns(adc_rank[0]);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t timeout) {
  (void)hadc, (void)timeout;
  if (now_ns < adc_single_done) advance(adc_single_done - now_ns);
  return HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  return adc_sample(adc_rank[0]);
}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc) { (void)hadc; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *data, uint32_t length) {
  (void)hadc;
  advance(CALL_NS * 25);                    // HAL's setting up of the ADC and the stream
  dma_dst = (uint16_t *)data;               // its stream moves half-words (DMA_MDATAALIGN_HALFWORD)
  dma_len = length;
  uint64_t t = 0;
  for (uint32_t k = 0; k < length; k++) t += adc_conversion_ns(adc_rank[k % (adc_ranks ? adc_ranks : 1)]);
  irq_at[IRQ_DMA] = now_ns + t;
  recheck();
  return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  advance(CALL_NS * 5);
  irq_at[IRQ_DMA] = NEVER;
  recheck();
  return HAL_OK;
}

// ---- SPI: its encoders and its IMU ----------------------------------------------------------------------------------
#define ENC_L_PORT GPIOC
#define ENC_L_PIN GPIO_PIN_5
#define ENC_R_PORT GPIOA
#define ENC_R_PIN GPIO_PIN_15
#define IMU_PORT GPIOA
#define IMU_PIN GPIO_PIN_4

static struct { uint16_t command, previous, answer, word; int bytes; } as5047[2];
static struct { uint8_t reg[128]; uint8_t addr, read; int bytes; uint64_t sample; int16_t out[6]; } imu;

static double encoder_angle(int i) {
  return encoder[i].angle + encoder[i].rate * (double)(now_ns - encoder[i].t0);
}

// its answer to a frame: to the previous frame's command (a read of ANGLECOM, 0x3FFF: its angle now)
static uint16_t as5047_answer(int i) {
  uint16_t cmd = as5047[i].previous;
  if ((cmd & 0x3FFF) != 0x3FFF || !(cmd & 0x4000)) return 0;
  double turns = encoder_angle(i) / (2 * M_PI);
  uint16_t angle = (uint16_t)((uint32_t)floor((turns - floor(turns)) * 16384.0) & 0x3FFF);
  uint16_t word = angle, parity = 0;
  for (int b = 0; b < 15; b++) parity ^= (word >> b) & 1;
  word |= (uint16_t)(parity << 15);
  return (uint16_t)(word >> 1);             // read one bit late (its SPI3 in mode 3)
}

static void imu_sample(void) {
  uint64_t k = now_ns / GYRO_ODR_NS;
  if (k == imu.sample) return;
  imu.sample = k;
  // its gyro reads 1 % low (its firmware multiplies it by 1.01); 140 mdps an LSB (+-4000 dps)
  double raw = (gyro_dps + gyro_noise_dps * gauss()) / 1.01 / 0.140;
  imu.out[2] = (int16_t)(raw > 32767 ? 32767 : raw < -32768 ? -32768 : lround(raw));
  imu.out[0] = imu.out[1] = 0;
  for (int a = 0; a < 3; a++) {                                    // 0.244 mg an LSB (+-8 g)
    double g = accel_g[a] / 0.000244;
    imu.out[3 + a] = (int16_t)(g > 32767 ? 32767 : g < -32768 ? -32768 : lround(g));
  }
}

static uint8_t imu_byte(uint8_t out) {
  if (imu.bytes++ == 0) {
    imu.addr = out & 0x7F;
    imu.read = out & 0x80;
    return 0xFF;
  }
  uint8_t a = imu.addr++ & 0x7F, r = 0;
  if (!imu.read) {
    imu.reg[a] = out;
  } else if (a == 0x0F) {
    r = 0x6B;                               // WHO_AM_I
  } else if (a >= 0x22 && a <= 0x2D) {
    imu_sample();
    int16_t v = imu.out[(a - 0x22) / 2];
    r = (a & 1) ? (uint8_t)((uint16_t)v >> 8) : (uint8_t)(v & 0xFF);
  } else {
    r = imu.reg[a];
  }
  return r;
}

static void gpio_changed(GPIO_TypeDef *port, uint16_t pins, uint32_t before) {
  uint32_t now = port->ODR;
  struct { GPIO_TypeDef *p; uint16_t pin; int enc; } cs[3] = {{ENC_L_PORT, ENC_L_PIN, 0}, {ENC_R_PORT, ENC_R_PIN, 1}, {IMU_PORT, IMU_PIN, 2}};
  for (int k = 0; k < 3; k++) {
    if (port != cs[k].p || !(pins & cs[k].pin)) continue;
    int fell = (before & cs[k].pin) && !(now & cs[k].pin), rose = !(before & cs[k].pin) && (now & cs[k].pin);
    if (k == 2) {
      if (fell) imu.bytes = 0;
    } else if (fell) {
      as5047[k].bytes = 0;
      as5047[k].command = 0;
      as5047[k].answer = as5047_answer(k);
    } else if (rose && as5047[k].bytes >= 2) {
      as5047[k].previous = as5047[k].command;
    }
  }
}

static uint8_t spi_byte(SPI_HandleTypeDef *h, uint8_t out) {
  if (h->Instance == SPI1) return !(IMU_PORT->ODR & IMU_PIN) ? imu_byte(out) : 0xFF;
  for (int k = 0; k < 2; k++) {
    GPIO_TypeDef *p = k ? ENC_R_PORT : ENC_L_PORT;
    uint16_t pin = k ? ENC_R_PIN : ENC_L_PIN;
    if (p->ODR & pin) continue;
    int n = as5047[k].bytes++;
    if (n < 2) {
      as5047[k].command = (uint16_t)(as5047[k].command | (uint16_t)out << (8 * (1 - n)));
      return (uint8_t)(as5047[k].answer >> (8 * (1 - n)));
    }
    return 0;
  }
  return 0xFF;
}

static void spi_time(SPI_HandleTypeDef *h, uint16_t n) {
  advance(SPI_CALL_NS + n * (h->Instance == SPI1 ? SPI1_BYTE_NS : SPI3_BYTE_NS));
}

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi) {
  HAL_SPI_MspInit(hspi);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout) {
  (void)timeout;
  spi_time(h, n);
  for (uint16_t i = 0; i < n; i++) spi_byte(h, data[i]);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout) {
  (void)timeout;                            // a master's receive sends what the buffer holds
  spi_time(h, n);
  for (uint16_t i = 0; i < n; i++) data[i] = spi_byte(h, data[i]);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t n, uint32_t timeout) {
  (void)timeout;
  spi_time(h, n);
  for (uint16_t i = 0; i < n; i++) rx[i] = spi_byte(h, tx[i]);
  return HAL_OK;
}

// ---- TIM ----------------------------------------------------------------------------------------------------------------
static void tim_start(int i) {
  TIM_TypeDef *t = &ksim_tim[i];
  if (t->CR1 & TIM_CR1_CEN) return;
  t->CR1 |= TIM_CR1_CEN;
  tim_base_cyc[i] = cyc_now() - (uint64_t)t->CNT * (t->PSC + 1);
}
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *h) {
  h->Instance->PSC = h->Init.Prescaler;
  h->Instance->ARR = h->Init.Period;
  HAL_TIM_Base_MspInit(h);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *h) {
  h->Instance->PSC = h->Init.Prescaler;
  h->Instance->ARR = h->Init.Period;
  HAL_TIM_PWM_MspInit(h);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *h, TIM_OC_InitTypeDef *c, uint32_t ch) {
  *(&h->Instance->CCR1 + (ch >> 2)) = c->Pulse;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *h, TIM_MasterConfigTypeDef *c) {
  (void)h, (void)c;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch) {
  advance(CALL_NS * 3);
  h->Instance->CCER |= 1u << ch;
  tim_start(tim_index(h->Instance));
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h, uint32_t ch) {
  advance(CALL_NS * 3);
  h->Instance->CCER &= ~(1u << ch);
  if (!(h->Instance->CCER & 0x1111u)) h->Instance->CR1 &= ~TIM_CR1_CEN;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h) {
  int i = tim_index(h->Instance);
  h->Instance->DIER |= TIM_DIER_UIE;
  tim_start(i);
  if (i == 6) tim6_schedule();
  recheck();
  return HAL_OK;
}
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *h) {
  if (h->Instance->SR & TIM_SR_UIF) {
    h->Instance->SR &= ~TIM_SR_UIF;
    HAL_TIM_PeriodElapsedCallback(h);
  }
}
__attribute__((weak)) void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h) { (void)h; }
void ksim_tim_set_counter(TIM_TypeDef *t, uint32_t cnt) {
  int i = tim_index(t);
  t->CNT = cnt;
  tim_base_cyc[i] = cyc_now() - (uint64_t)cnt * (t->PSC + 1);
  if (i == 6) tim6_schedule();
  recheck();
}
void ksim_tim_set_autoreload(TIM_HandleTypeDef *h, uint32_t arr) {
  TIM_TypeDef *t = h->Instance;
  int i = tim_index(t);
  tim_sync(i);
  t->ARR = arr;
  h->Init.Period = arr;
  if (t->CNT > arr) t->CNT = 0;
  tim_base_cyc[i] = cyc_now() - (uint64_t)t->CNT * (t->PSC + 1);
  if (i == 6) tim6_schedule();
  recheck();
}

// ---- USART2: its printf ---------------------------------------------------------------------------------------------
static char tx[1 << 20];
static uint32_t tx_len;

static void uart_out(const char *s, size_t n) {
  for (size_t i = 0; i < n; i++)
    if (tx_len < sizeof(tx) && s[i] != '\r') tx[tx_len++] = s[i];
  advance(n * UART_BYTE_NS);
}
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *h) {
  HAL_UART_MspInit(h);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *h) {
  HAL_UART_MspInit(h);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *data, uint16_t n, uint32_t timeout) {
  (void)h, (void)timeout;
  uart_out((const char *)data, n);
  return HAL_OK;
}
int printf(const char *fmt, ...) {
  char buf[4096];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n > 0) uart_out(buf, (size_t)(n < (int)sizeof buf ? n : (int)sizeof buf - 1));
  return n;
}

// ---- Its main() -------------------------------------------------------------------------------------------------------
static void fw_entry(void) {
  in_fw = 1;
  koguma_main();
  state = KSIM_HALTED;
  for (;;) swapcontext(&fw_ctx, &sim_ctx);
}

// ---- Simulator API ----------------------------------------------------------------------------------------------------
int ksim_boot(uint64_t seed) {
  rng ^= seed * 0x2545F4914F6CDD1Dull + 1;
  GPIOB->IDR |= GPIO_PIN_3;
  fw_stack = malloc(STACK_BYTES);
  if (!fw_stack) return 0;
  getcontext(&fw_ctx);
  fw_ctx.uc_stack.ss_sp = fw_stack;
  fw_ctx.uc_stack.ss_size = STACK_BYTES;
  fw_ctx.uc_link = NULL;
  makecontext(&fw_ctx, fw_entry, 0);
  return 1;
}
uint8_t ksim_run_until(uint64_t t_ns) {
  if (state != KSIM_RUNNING) return state;
  until_ns = t_ns;
  recheck();
  if (now_ns < until_ns) swapcontext(&sim_ctx, &fw_ctx);
  return state;
}
uint64_t ksim_time_ns(void) { return now_ns; }

void ksim_set_encoder(int side, double angle_rad, double rad_per_ns) {
  if (side < 0 || side > 1) return;
  encoder[side].angle = angle_rad;
  encoder[side].rate = rad_per_ns;
  encoder[side].t0 = now_ns;
}
void ksim_set_ir(int sensor, double signal, double dark) {
  if (sensor < 0 || sensor > 3) return;
  ir_signal[sensor] = signal;
  ir_dark[sensor] = dark;
}
void ksim_set_noise(double ir_counts, double gyro_dps_) {
  ir_noise = ir_counts;
  gyro_noise_dps = gyro_dps_;
}
void ksim_set_imu(double gz_dps, double ax_g, double ay_g, double az_g) {
  gyro_dps = gz_dps;
  accel_g[0] = ax_g, accel_g[1] = ay_g, accel_g[2] = az_g;
}
void ksim_set_battery(double v) { battery_v = v; }
void ksim_set_switch(int pressed) { switch_pressed = pressed != 0; }

// a motor's duty (+ forward, of its battery) if its driver drives it (brake included), 0 if it coasts
int ksim_motor(int side, double *duty) {
  TIM_TypeDef *t = TIM3;
  uint32_t a = side ? 8 : 0, b = side ? 12 : 4;
  *duty = 0.0;
  if (!(t->CR1 & TIM_CR1_CEN) || !(t->CCER & (1u << a)) || !(t->CCER & (1u << b))) return 0;
  double top = (double)t->ARR + 1;
  double ca = *(&t->CCR1 + (a >> 2)), cb = *(&t->CCR1 + (b >> 2));
  ca = ca > top ? top : ca, cb = cb > top ? top : cb;
  *duty = (cb - ca) / top;
  return 1;
}
double ksim_fan(void) {
  TIM_TypeDef *t = TIM2;
  if (!(t->CR1 & TIM_CR1_CEN) || !(t->CCER & 1u)) return 0.0;
  double d = (double)t->CCR1 / ((double)t->ARR + 1);
  return d > 1 ? 1 : d;
}
double ksim_buzzer(void) {
  TIM_TypeDef *t = TIM4;
  if (!(t->CR1 & TIM_CR1_CEN) || !(t->CCER & (1u << 12)) || !t->CCR4) return 0.0;
  return 90e6 / ((double)t->PSC + 1) / ((double)t->ARR + 1);
}
// LED1..LED8 (PB12-15, PB4-7) in bits 0-7, the front LEDs LEDF1..LEDF4 (PA3, PC15, PC14, PC13) in 8-11, the
// emitters (PA1: FL and R, PA0: L and FR) in 12-13
uint32_t ksim_leds(void) {
  static const uint16_t b[8] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};
  uint32_t out = 0;
  for (int i = 0; i < 8; i++)
    if (GPIOB->ODR & b[i]) out |= 1u << i;
  if (GPIOA->ODR & GPIO_PIN_3) out |= 1u << 8;
  if (GPIOC->ODR & GPIO_PIN_15) out |= 1u << 9;
  if (GPIOC->ODR & GPIO_PIN_14) out |= 1u << 10;
  if (GPIOC->ODR & GPIO_PIN_13) out |= 1u << 11;
  if (GPIOA->ODR & GPIO_PIN_1) out |= 1u << 12;
  if (GPIOA->ODR & GPIO_PIN_0) out |= 1u << 13;
  return out;
}
uint32_t ksim_console(char *buffer, uint32_t size) {
  uint32_t n = tx_len < size ? tx_len : size;
  memcpy(buffer, tx, n);
  memmove(tx, tx + n, tx_len - n);
  tx_len -= n;
  return n;
}
