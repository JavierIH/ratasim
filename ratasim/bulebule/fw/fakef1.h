// The part of libopencm3 (STM32F1) that Bulebule's firmware uses, for its chip simulated in bchip.c: GPIO, TIM1's
// update interrupt, TIM3's compare values, the encoder counters, the ADC (ADC1's injected group, ADC2's regular
// conversion), DMA1's channels 2 and 3 with USART3, SPI2, the flash, SysTick's interrupt and the cycle counter do
// something there; the rest configures nothing.
#ifndef FAKEF1_H
#define FAKEF1_H

#include <stdbool.h>
#include <stdint.h>

// ---- GPIO ----
#define GPIOA 0u
#define GPIOB 1u
#define GPIOC 2u
#define GPIO0 (1u << 0)
#define GPIO1 (1u << 1)
#define GPIO2 (1u << 2)
#define GPIO3 (1u << 3)
#define GPIO4 (1u << 4)
#define GPIO5 (1u << 5)
#define GPIO6 (1u << 6)
#define GPIO7 (1u << 7)
#define GPIO8 (1u << 8)
#define GPIO9 (1u << 9)
#define GPIO10 (1u << 10)
#define GPIO11 (1u << 11)
#define GPIO12 (1u << 12)
#define GPIO13 (1u << 13)
#define GPIO14 (1u << 14)
#define GPIO15 (1u << 15)
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT_10_MHZ 1
#define GPIO_MODE_OUTPUT_2_MHZ 2
#define GPIO_MODE_OUTPUT_50_MHZ 3
#define GPIO_CNF_INPUT_ANALOG 0
#define GPIO_CNF_INPUT_FLOAT 1
#define GPIO_CNF_INPUT_PULL_UPDOWN 2
#define GPIO_CNF_OUTPUT_PUSHPULL 0
#define GPIO_CNF_OUTPUT_ALTFN_PUSHPULL 2
#define GPIO_TIM1_CH3 GPIO10        // PA10
#define GPIO_TIM3_CH1 GPIO6         // PA6
#define GPIO_TIM3_CH2 GPIO7         // PA7
#define GPIO_TIM3_CH3 GPIO0         // PB0
#define GPIO_TIM3_CH4 GPIO1         // PB1
#define GPIO_USART3_TX GPIO10       // PB10
#define GPIO_USART3_RX GPIO11       // PB11
#define AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON 0
#define AFIO_MAPR_TIM2_REMAP_FULL_REMAP 0
void gpio_set(uint32_t port, uint16_t pins);
void gpio_clear(uint32_t port, uint16_t pins);
void gpio_toggle(uint32_t port, uint16_t pins);
uint16_t gpio_get(uint32_t port, uint16_t pins);
#define gpio_set_mode(...) ((void)0)
#define gpio_primary_remap(...) ((void)0)

// ---- Timers ----
#define TIM1 1u
#define TIM2 2u
#define TIM3 3u
#define TIM4 4u
enum tim_oc_id { TIM_OC1, TIM_OC2, TIM_OC3, TIM_OC4 };
enum tim_ic_id { TIM_IC1, TIM_IC2, TIM_IC3, TIM_IC4 };
enum tim_ic_input { TIM_IC_IN_TI1 = 1, TIM_IC_IN_TI2 = 2 };
enum tim_oc_mode { TIM_OCM_FROZEN, TIM_OCM_PWM1 = 6, TIM_OCM_PWM2 = 7 };
#define TIM_CR1_CKD_CK_INT 0
#define TIM_CR1_CMS_EDGE 0
#define TIM_CR1_DIR_UP 0
#define TIM_DIER_UIE (1u << 0)
#define TIM_SR_UIF (1u << 0)
void timer_set_oc_value(uint32_t tim, enum tim_oc_id oc, uint32_t value);
uint32_t timer_get_counter(uint32_t tim);
void timer_enable_irq(uint32_t tim, uint32_t irq);
void timer_disable_irq(uint32_t tim, uint32_t irq);
bool timer_get_flag(uint32_t tim, uint32_t flag);
void timer_clear_flag(uint32_t tim, uint32_t flag);
#define timer_set_mode(...) ((void)0)
#define timer_set_prescaler(...) ((void)0)
#define timer_set_period(...) ((void)0)
#define timer_enable_preload(...) ((void)0)
#define timer_continuous_mode(...) ((void)0)
#define timer_enable_counter(...) ((void)0)
#define timer_disable_counter(...) ((void)0)
#define timer_set_oc_mode(...) ((void)0)
#define timer_enable_oc_output(...) ((void)0)
#define timer_disable_oc_output(...) ((void)0)
#define timer_enable_break_main_output(...) ((void)0)
#define timer_set_repetition_counter(...) ((void)0)
#define timer_ic_set_input(...) ((void)0)
#define timer_slave_set_mode(...) ((void)0)
#define timer_set_clock_division(...) ((void)0)

// ---- ADC ----
#define ADC1 1u
#define ADC2 2u
#define ADC_CHANNEL0 0
#define ADC_CHANNEL1 1
#define ADC_CHANNEL2 2
#define ADC_CHANNEL3 3
#define ADC_CHANNEL4 4
#define ADC_CHANNEL5 5
#define ADC_CR2_JEXTSEL_JSWSTART 0
#define ADC_SMPR_SMP_13DOT5CYC 0
#define ADC_SR_EOC (1u << 1)
#define ADC_SR_JEOC (1u << 2)
#define ADC_SR(adc) (*bsim_adc_sr(adc))
volatile uint32_t *bsim_adc_sr(uint32_t adc);
void adc_start_conversion_injected(uint32_t adc);
uint32_t adc_read_injected(uint32_t adc, uint8_t reg);
void adc_start_conversion_direct(uint32_t adc);
uint32_t adc_read_regular(uint32_t adc);
static inline bool adc_eoc(uint32_t adc) { (void)adc; return true; }
#define adc_power_off(...) ((void)0)
#define adc_power_on(...) ((void)0)
#define adc_enable_scan_mode(...) ((void)0)
#define adc_disable_scan_mode(...) ((void)0)
#define adc_set_single_conversion_mode(...) ((void)0)
#define adc_enable_external_trigger_injected(...) ((void)0)
#define adc_disable_external_trigger_regular(...) ((void)0)
#define adc_set_right_aligned(...) ((void)0)
#define adc_set_sample_time_on_all_channels(...) ((void)0)
#define adc_set_injected_sequence(...) ((void)0)
#define adc_set_regular_sequence(...) ((void)0)
#define adc_reset_calibration(...) ((void)0)
#define adc_calibrate(...) ((void)0)

// ---- DMA1 (channel 2: USART3's TX, channel 3: its RX) ----
#define DMA1 1u
#define DMA_CHANNEL1 1
#define DMA_CHANNEL2 2
#define DMA_CHANNEL3 3
#define DMA_TCIF (1u << 1)
#define DMA_CCR_PSIZE_8BIT 0
#define DMA_CCR_MSIZE_8BIT 0
#define DMA_CCR_PL_HIGH 0
#define DMA_CCR_PL_VERY_HIGH 0
void dma_channel_reset(uint32_t dma, uint8_t channel);
void dma_set_memory_address(uint32_t dma, uint8_t channel, uint32_t address);
void dma_set_number_of_data(uint32_t dma, uint8_t channel, uint16_t number);
void dma_set_read_from_memory(uint32_t dma, uint8_t channel);
void dma_set_read_from_peripheral(uint32_t dma, uint8_t channel);
void dma_enable_channel(uint32_t dma, uint8_t channel);
void dma_disable_channel(uint32_t dma, uint8_t channel);
void dma_enable_transfer_complete_interrupt(uint32_t dma, uint8_t channel);
void dma_disable_transfer_complete_interrupt(uint32_t dma, uint8_t channel);
bool dma_get_interrupt_flag(uint32_t dma, uint8_t channel, uint32_t flag);
void dma_clear_interrupt_flags(uint32_t dma, uint8_t channel, uint32_t flag);
#define dma_set_peripheral_address(...) ((void)0)
#define dma_enable_memory_increment_mode(...) ((void)0)
#define dma_set_peripheral_size(...) ((void)0)
#define dma_set_memory_size(...) ((void)0)
#define dma_set_priority(...) ((void)0)

// ---- NVIC, SCB, SysTick, RCC, DWT ----
#define NVIC_TIM1_UP_IRQ 25
#define NVIC_SYSTICK_IRQ -1
#define NVIC_DMA1_CHANNEL2_IRQ 12
#define NVIC_DMA1_CHANNEL3_IRQ 13
#define NVIC_USART3_IRQ 39
#define nvic_set_priority(...) ((void)0)
#define nvic_enable_irq(...) ((void)0)
#define systick_set_frequency(...) ((void)0)
#define systick_counter_enable(...) ((void)0)
void systick_interrupt_enable(void);
void systick_interrupt_disable(void);
extern uint32_t rcc_apb1_frequency, rcc_apb2_frequency, rcc_ahb_frequency;
#define RCC_GPIOA 0
#define RCC_GPIOB 0
#define RCC_GPIOC 0
#define RCC_USART3 0
#define RCC_SPI2 0
#define RCC_TIM1 0
#define RCC_TIM2 0
#define RCC_TIM3 0
#define RCC_TIM4 0
#define RCC_AFIO 0
#define RCC_ADC1 0
#define RCC_ADC2 0
#define RCC_DMA1 0
#define RST_TIM1 1
void rcc_periph_reset_pulse(uint32_t rst);
#define rcc_clock_setup_in_hse_8mhz_out_72mhz() ((void)0)
#define rcc_periph_clock_enable(...) ((void)0)
uint32_t dwt_read_cycle_counter(void);
static inline bool dwt_enable_cycle_counter(void) { return true; }

// ---- SPI2, USART3, flash, mutexes ----
#define SPI2 2u
#define SPI_CR1_BAUDRATE_FPCLK_DIV_2 0
#define SPI_CR1_BAUDRATE_FPCLK_DIV_64 0
#define SPI_CR1_CPHA_CLK_TRANSITION_1 0
#define SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE 0
#define SPI_CR1_DFF_8BIT 0
#define SPI_CR1_MSBFIRST 0
void spi_send(uint32_t spi, uint16_t data);
uint16_t spi_read(uint32_t spi);
#define spi_reset(...) ((void)0)
#define spi_init_master(...) ((void)0)
#define spi_enable_software_slave_management(...) ((void)0)
#define spi_set_nss_high(...) ((void)0)
#define spi_enable(...) ((void)0)
#define USART3 3u
#define USART_SR(u) (*bsim_usart_reg(u, 0x00))
#define USART_DR(u) (*bsim_usart_reg(u, 0x04))
#define USART_CR1(u) (*bsim_usart_reg(u, 0x0C))
#define USART3_DR USART_DR(USART3)
#define USART_SR_IDLE (1u << 4)
#define USART_CR1_IDLEIE (1u << 4)
#define USART_STOPBITS_1 0
#define USART_PARITY_NONE 0
#define USART_FLOWCONTROL_NONE 0
#define USART_MODE_TX_RX 0
volatile uint32_t *bsim_usart_reg(uint32_t usart, uint32_t offset);
void usart_enable_tx_dma(uint32_t usart);
void usart_disable_tx_dma(uint32_t usart);
void usart_enable_rx_dma(uint32_t usart);
void usart_disable_rx_dma(uint32_t usart);
#define usart_set_baudrate(...) ((void)0)
#define usart_set_databits(...) ((void)0)
#define usart_set_stopbits(...) ((void)0)
#define usart_set_parity(...) ((void)0)
#define usart_set_flow_control(...) ((void)0)
#define usart_set_mode(...) ((void)0)
#define usart_enable(...) ((void)0)
#define FLASH_SR_EOP (1u << 5)
void flash_unlock(void);
void flash_erase_page(uint32_t page_address);
uint32_t flash_get_status_flags(void);
void flash_program_word(uint32_t address, uint32_t data);
typedef uint32_t mutex_t;
static inline uint32_t mutex_trylock(mutex_t *m) {
  if (*m) return 0;
  *m = 1;
  return 1;
}
static inline void mutex_unlock(mutex_t *m) { *m = 0; }

#endif
