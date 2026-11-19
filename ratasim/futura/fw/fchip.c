// Futura's chip (STM32F405 at 168 MHz) simulated: its main() in a coroutine, on a clock of CPU cycles, and the
// Standard Peripheral Library functions and registers its firmware template uses (include/stm32f4xx.h):
//
//   SysTick     every millisecond (SysTick_Config), its handler (stm32f4xx_it.c: Millis, the buzzer's time)
//   ADC1        single conversions started by software: the input sampled at the start (each phototransistor's lit
//               or dark by its emitter's pin), EOC 84 + 12 ADC cycles later (21 MHz)
//   TIM2, TIM5  the encoders' 32-bit counters (left, right): each millisecond they count what the wheels turned
//   TIM4        the motors' PWM (CCR1/CCR2 left forward/backwards, CCR3/CCR4 right, of ARR + 1)
//   TIM3        the buzzer's PWM (CCR1: sounding)
//   USART1      its printf (fputc): each byte at its baud rate, TXE while one waits
//   SPI2        the HCMS-2903 display's bytes, at 21 MHz; RS (PB14) and CE (PC6) pins
//   GPIO        output data registers (its LEDs, its emitters)
//
// Time passes where main() waits on the chip: each read of Millis (its delays), of SysTick->VAL (micros()), of a
// flag (ADC, SPI, USART), of what the controller sample's loop polls (fsim_poll()). When that crosses a
// millisecond, main() gives the time back to fsim_step(), which runs SysTick's handler then; computation takes no
// simulated time.
#define _GNU_SOURCE
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucontext.h>
#include "stm32f4xx.h"

#define CPU_HZ 168000000u
#define CYCLES_PER_MS (CPU_HZ / 1000)
#define POLL_CYCLES 12                 // a waiting loop's pass (load, compare, branch)
#define ADC_CYCLES ((84 + 12) * 8)     // 84 + 12 ADC cycles at 21 MHz (ADCCLK = 84 MHz / 4)
#define SPI_BYTE_CYCLES (8 * 8)        // 8 bits at 21 MHz
#define STACK_BYTES (1024 * 1024)
enum { FSIM_RUNNING = 0, FSIM_RESET = 1, FSIM_HALTED = 2 };

extern int firmware_main(void);
extern void SysTick_Handler(void);
extern int fw_fputc(int ch, FILE *f);

uint32_t SystemCoreClock = CPU_HZ;
GPIO_TypeDef fsim_gpio[4];
TIM_TypeDef fsim_tim[6];
ADC_TypeDef fsim_adc1;
SPI_TypeDef fsim_spi2;
USART_TypeDef fsim_usart[2];

static ucontext_t sim_ctx, fw_ctx;
static char *fw_stack;
static uint64_t cycles, next_tick;
static uint32_t ticks, systick_load = CYCLES_PER_MS - 1;
static uint8_t in_isr, state, systick_on;
static volatile u32 millis;
static SysTick_Type systick;

static void yield(void) { swapcontext(&fw_ctx, &sim_ctx); }

// main() spends n cycles; past the millisecond, the time goes back to fsim_step() (SysTick's interrupt)
static void spend(uint32_t n) {
  cycles += n;
  if (in_isr) return;
  while (cycles >= next_tick && state == FSIM_RUNNING) yield();
}

static void fw_entry(void) {
  firmware_main();
  state = FSIM_HALTED;
  for (;;) yield();
}

volatile u32 *fsim_millis(void) {
  spend(POLL_CYCLES);
  return &millis;
}

// a variable a waiting loop polls (controller.h: the controller sample's encoderCount), at a pass's time
volatile int32_t *fsim_poll(volatile int32_t *p) {
  spend(POLL_CYCLES);
  return p;
}

SysTick_Type *fsim_systick(void) {     // its down-counter at this cycle
  spend(POLL_CYCLES);
  uint64_t into = cycles + CYCLES_PER_MS - next_tick;
  systick.LOAD = systick_load;
  systick.VAL = into >= systick_load ? 0 : systick_load - (uint32_t)into;
  systick.CTRL = systick_on ? 7 : 0;
  return &systick;
}

// ---- core: clocks, interrupts (none but SysTick's: its buttons' handlers are empty) ----
void SystemInit(void) {}
void SystemCoreClockUpdate(void) { SystemCoreClock = CPU_HZ; }
uint32_t SysTick_Config(uint32_t ticks_) {
  systick_load = ticks_ - 1;
  systick_on = 1;
  return 0;
}
void NVIC_PriorityGroupConfig(uint32_t group) { (void)group; }
void NVIC_Init(NVIC_InitTypeDef *init) { (void)init; }
void NVIC_SetPriority(IRQn_Type irq, uint32_t priority) { (void)irq; (void)priority; }
void RCC_AHB1PeriphClockCmd(uint32_t p, FunctionalState s) { (void)p; (void)s; }
void RCC_APB1PeriphClockCmd(uint32_t p, FunctionalState s) { (void)p; (void)s; }
void RCC_APB2PeriphClockCmd(uint32_t p, FunctionalState s) { (void)p; (void)s; }
void EXTI_Init(EXTI_InitTypeDef *init) { (void)init; }
ITStatus EXTI_GetITStatus(uint32_t line) { (void)line; return RESET; }
void EXTI_ClearITPendingBit(uint32_t line) { (void)line; }
void SYSCFG_EXTILineConfig(uint8_t port, uint8_t pin) { (void)port; (void)pin; }

// ---- GPIO ----
void GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) {
  for (int pin = 0; pin < 16; pin++) {
    if (!(init->GPIO_Pin & (1u << pin))) continue;
    port->MODER = (port->MODER & ~(3u << 2 * pin)) | ((uint32_t)init->GPIO_Mode << 2 * pin);
    port->PUPDR = (port->PUPDR & ~(3u << 2 * pin)) | ((uint32_t)init->GPIO_PuPd << 2 * pin);
  }
}
void GPIO_PinAFConfig(GPIO_TypeDef *port, uint16_t source, uint8_t af) {
  port->AFR[source >> 3] = (port->AFR[source >> 3] & ~(0xFu << 4 * (source & 7))) | ((uint32_t)af << 4 * (source & 7));
}
void GPIO_SetBits(GPIO_TypeDef *port, uint16_t pins) { port->ODR |= pins; }
void GPIO_ResetBits(GPIO_TypeDef *port, uint16_t pins) { port->ODR &= ~(uint32_t)pins; }
void GPIO_ToggleBits(GPIO_TypeDef *port, uint16_t pins) { port->ODR ^= pins; }
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *port, uint16_t pin) { return (port->IDR & pin) ? 1 : 0; }
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef *port, uint16_t pin) { return (port->ODR & pin) ? 1 : 0; }

// ---- TIM ----
void TIM_TimeBaseInit(TIM_TypeDef *t, TIM_TimeBaseInitTypeDef *init) {
  t->PSC = init->TIM_Prescaler;
  t->ARR = init->TIM_Period;
}
void TIM_OC1Init(TIM_TypeDef *t, TIM_OCInitTypeDef *init) { t->CCR1 = init->TIM_Pulse; }
void TIM_OC2Init(TIM_TypeDef *t, TIM_OCInitTypeDef *init) { t->CCR2 = init->TIM_Pulse; }
void TIM_OC3Init(TIM_TypeDef *t, TIM_OCInitTypeDef *init) { t->CCR3 = init->TIM_Pulse; }
void TIM_OC4Init(TIM_TypeDef *t, TIM_OCInitTypeDef *init) { t->CCR4 = init->TIM_Pulse; }
void TIM_OC1PreloadConfig(TIM_TypeDef *t, uint16_t p) { (void)t; (void)p; }
void TIM_OC2PreloadConfig(TIM_TypeDef *t, uint16_t p) { (void)t; (void)p; }
void TIM_OC3PreloadConfig(TIM_TypeDef *t, uint16_t p) { (void)t; (void)p; }
void TIM_OC4PreloadConfig(TIM_TypeDef *t, uint16_t p) { (void)t; (void)p; }
void TIM_ARRPreloadConfig(TIM_TypeDef *t, FunctionalState s) { (void)t; (void)s; }
void TIM_Cmd(TIM_TypeDef *t, FunctionalState s) {
  if (s) t->CR1 |= TIM_CR1_CEN;
  else t->CR1 &= ~TIM_CR1_CEN;
}
void TIM_SetAutoreload(TIM_TypeDef *t, uint32_t a) { t->ARR = a; }
void TIM_SetCounter(TIM_TypeDef *t, uint32_t c) { t->CNT = c; }
uint32_t TIM_GetCounter(TIM_TypeDef *t) { return t->CNT; }
void TIM_EncoderInterfaceConfig(TIM_TypeDef *t, uint16_t mode, uint16_t ic1, uint16_t ic2) {
  t->SMCR = mode;
  t->CCER = ic1 | (ic2 << 4);
}

// ---- ADC1 ----
// Analog inputs (counts 0..4095): each phototransistor's with its emitter lit and dark, the rest as they are.
static uint16_t analog[19], analog_lit[19];
static GPIO_TypeDef *emitter_port[19];
static uint16_t emitter_pin[19];
static uint8_t adc_channel, adc_on;
static uint64_t adc_done;

void ADC_CommonInit(ADC_CommonInitTypeDef *init) { (void)init; }
void ADC_Init(ADC_TypeDef *adc, ADC_InitTypeDef *init) { (void)adc; (void)init; }
void ADC_ITConfig(ADC_TypeDef *adc, uint16_t it, FunctionalState s) { (void)adc; (void)it; (void)s; }
void ADC_Cmd(ADC_TypeDef *adc, FunctionalState s) { (void)adc; adc_on = s; }
void ADC_RegularChannelConfig(ADC_TypeDef *adc, uint8_t ch, uint8_t rank, uint8_t t) {
  (void)rank; (void)t;
  adc->SQR3 = ch;
  adc_channel = ch < 19 ? ch : 18;
}
void ADC_SoftwareStartConv(ADC_TypeDef *adc) {
  if (!adc_on) return;
  unsigned ch = adc_channel;
  int lit = emitter_port[ch] && (emitter_port[ch]->ODR & emitter_pin[ch]);
  adc->DR = lit ? analog_lit[ch] : analog[ch];
  adc->SR &= ~(uint32_t)ADC_FLAG_EOC;
  adc_done = cycles + ADC_CYCLES;
}
FlagStatus ADC_GetFlagStatus(ADC_TypeDef *adc, uint8_t flag) {
  spend(POLL_CYCLES);
  if (flag == ADC_FLAG_EOC && cycles >= adc_done) adc->SR |= ADC_FLAG_EOC;
  return (adc->SR & flag) ? SET : RESET;
}
uint16_t ADC_GetConversionValue(ADC_TypeDef *adc) {
  adc->SR &= ~(uint32_t)ADC_FLAG_EOC;
  return (uint16_t)adc->DR;
}

// ---- SPI2: the HCMS-2903's bytes ----
static uint64_t spi_free;
static uint8_t display[64];
static unsigned display_n;
void SPI_I2S_DeInit(SPI_TypeDef *s) { memset((void *)s, 0, sizeof *s); }
void SPI_StructInit(SPI_InitTypeDef *init) { memset(init, 0, sizeof *init); }
void SPI_Init(SPI_TypeDef *s, SPI_InitTypeDef *init) { s->CR1 = init->SPI_Mode | init->SPI_Direction; }
void SPI_SSOutputCmd(SPI_TypeDef *s, FunctionalState st) { (void)s; (void)st; }
void SPI_Cmd(SPI_TypeDef *s, FunctionalState st) { (void)s; (void)st; }
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef *s, uint16_t n) { (void)s; (void)n; }
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef *s, uint16_t flag) {
  (void)s;
  spend(POLL_CYCLES);
  int busy = cycles < spi_free;
  if (flag == SPI_I2S_FLAG_TXE) return cycles + SPI_BYTE_CYCLES >= spi_free ? SET : RESET;
  if (flag == SPI_I2S_FLAG_BSY) return busy ? SET : RESET;
  return RESET;
}
void SPI_I2S_SendData(SPI_TypeDef *s, uint16_t data) {
  s->DR = data;
  spi_free = (spi_free > cycles ? spi_free : cycles) + SPI_BYTE_CYCLES;
  // its dot data (RS low, PB14) while CE (PC6) is low: 20 column bytes, a frame from its first
  if (!(fsim_gpio[1].ODR & GPIO_Pin_14) && !(fsim_gpio[2].ODR & GPIO_Pin_6)) {
    if (display_n >= 20) display_n = 0;
    display[display_n++] = (uint8_t)data;
  }
}

// ---- USART1: its printf ----
static uint32_t baud = 9600;
static uint64_t tx_free;
static char tx_buf[1 << 16];
static uint32_t tx_n;
void USART_OverSampling8Cmd(USART_TypeDef *u, FunctionalState s) { (void)u; (void)s; }
void USART_Init(USART_TypeDef *u, USART_InitTypeDef *init) {
  if (u == USART1 && init->USART_BaudRate) baud = init->USART_BaudRate;
}
void USART_ClockInit(USART_TypeDef *u, USART_ClockInitTypeDef *init) { (void)u; (void)init; }
void USART_Cmd(USART_TypeDef *u, FunctionalState s) { (void)u; (void)s; }
static uint32_t byte_cycles(void) { return (uint32_t)((uint64_t)CPU_HZ * 10 / baud); }
void USART_SendData(USART_TypeDef *u, uint16_t data) {
  u->DR = data;
  if (u != USART1) return;
  tx_free = (tx_free > cycles ? tx_free : cycles) + byte_cycles();
  if (tx_n < sizeof tx_buf) tx_buf[tx_n++] = (char)data;
}
uint16_t USART_ReceiveData(USART_TypeDef *u) { return u->DR & 0x1FF; }
FlagStatus USART_GetFlagStatus(USART_TypeDef *u, uint16_t flag) {
  spend(POLL_CYCLES);
  if (u != USART1) return flag == USART_FLAG_RXNE ? RESET : SET;
  if (flag == USART_FLAG_TXE) return cycles + byte_cycles() >= tx_free ? SET : RESET;   // one byte shifting out
  if (flag == USART_FLAG_TC) return cycles >= tx_free ? SET : RESET;
  return RESET;                        // nothing comes in
}

// printf (fsim.h): Keil's library formats and calls fputc for each character; so here
int fsim_printf(const char *fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  for (int i = 0; i < n && i < (int)sizeof buf - 1; i++) fw_fputc((unsigned char)buf[i], stdout);
  return n;
}

// ---- Simulator API ----
int fsim_boot(void) {
  fw_stack = malloc(STACK_BYTES);
  if (!fw_stack) return 0;
  next_tick = CYCLES_PER_MS;
  getcontext(&fw_ctx);
  fw_ctx.uc_stack.ss_sp = fw_stack;
  fw_ctx.uc_stack.ss_size = STACK_BYTES;
  fw_ctx.uc_link = NULL;
  makecontext(&fw_ctx, fw_entry, 0);
  return 1;
}

// One millisecond: main() runs to it, then SysTick's interrupt
uint8_t fsim_step(void) {
  if (state != FSIM_RUNNING) return state;
  swapcontext(&sim_ctx, &fw_ctx);
  if (cycles < next_tick) cycles = next_tick;    // halted
  next_tick += CYCLES_PER_MS;
  ticks++;
  if (systick_on) {
    in_isr = 1;
    SysTick_Handler();
    in_isr = 0;
  }
  return state;
}

uint32_t fsim_time_ms(void) { return ticks; }

// the IR receivers' inputs: the emitter's port (0 A .. 3 D) and pin, the input lit and dark (counts)
void fsim_set_ir(uint8_t ch, int8_t port, uint8_t pin, uint16_t lit, uint16_t dark) {
  if (ch >= 19) return;
  emitter_port[ch] = port >= 0 ? &fsim_gpio[port] : NULL;
  emitter_pin[ch] = (uint16_t)(1u << pin);
  analog_lit[ch] = lit > 4095 ? 4095 : lit;
  analog[ch] = dark > 4095 ? 4095 : dark;
}
void fsim_set_adc(uint8_t ch, uint16_t counts) {
  if (ch < 19) analog[ch] = counts > 4095 ? 4095 : counts;
}

// what the wheels turned since the last call (counts, forward positive): TIM2 left, TIM5 right
void fsim_add_encoders(int32_t left, int32_t right) {
  if (TIM2->CR1 & TIM_CR1_CEN) TIM2->CNT += (uint32_t)left;
  if (TIM5->CR1 & TIM_CR1_CEN) TIM5->CNT += (uint32_t)right;
}

// duties in counts of ARR + 1, forward positive (pwm.c: CCR1 left forward, CCR2 left backwards, CCR3/CCR4 right)
void fsim_get_pwm(int32_t *left, int32_t *right, uint32_t *period) {
  int on = TIM4->CR1 & TIM_CR1_CEN;
  *left = on ? (int32_t)TIM4->CCR1 - (int32_t)TIM4->CCR2 : 0;
  *right = on ? (int32_t)TIM4->CCR3 - (int32_t)TIM4->CCR4 : 0;
  *period = TIM4->ARR + 1;
}

uint32_t fsim_uart_tx(char *out, uint32_t max) {
  uint32_t n = tx_n < max ? tx_n : max;
  memcpy(out, tx_buf, n);
  memmove(tx_buf, tx_buf + n, tx_n - n);
  tx_n -= n;
  return n;
}

// its eleven LEDs (led.h: LED1..LED11), bit k-1 for LEDk
uint16_t fsim_leds(void) {
  static const struct { uint8_t port, pin; } led[11] = {{0, 11}, {0, 12}, {2, 10}, {2, 11}, {2, 12}, {3, 2},
                                                        {0, 8}, {1, 4}, {1, 12}, {0, 2}, {1, 11}};
  uint16_t out = 0;
  for (int i = 0; i < 11; i++)
    if (fsim_gpio[led[i].port].ODR & (1u << led[i].pin)) out |= 1u << i;
  return out;
}

uint8_t fsim_buzzer(void) { return (TIM3->CR1 & TIM_CR1_CEN) && TIM3->CCR1 ? 1 : 0; }

// the display's 20 column bytes (4 characters of 5 columns, 7 dots each)
void fsim_display(uint8_t *out) { memcpy(out, display, 20); }
