// The part of libopencm3 (STM32F1/F4) that the simulated robots' firmware uses (ZoroBot3: zorobot/fw/zchip.c,
// NezumiBot: nezumibot/fw/nchip.c): GPIO, timers, the ADC's injected group, SPI, USART, flash and the cycle counter
// do something there; the rest configures nothing.
#ifndef FAKECM3_H
#define FAKECM3_H

#include <stdbool.h>
#include <stdint.h>

#define MMIO32(addr) (*zsim_mmio32((uint32_t)(addr)))
volatile uint32_t *zsim_mmio32(uint32_t addr);

// ---- GPIO ----
#define GPIOA 0u
#define GPIOB 1u
#define GPIOC 2u
#define GPIOD 3u
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
#define GPIO_ALL 0xFFFFu
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
enum { GPIO_PUPD_NONE, GPIO_PUPD_PULLUP, GPIO_PUPD_PULLDOWN };
enum { GPIO_OTYPE_PP, GPIO_OTYPE_OD };
enum { GPIO_OSPEED_2MHZ, GPIO_OSPEED_25MHZ, GPIO_OSPEED_50MHZ, GPIO_OSPEED_100MHZ };
#define GPIO_AF0 0
#define GPIO_AF1 1
#define GPIO_AF2 2
#define GPIO_AF3 3
#define GPIO_AF4 4
#define GPIO_AF5 5
#define GPIO_AF6 6
#define GPIO_AF7 7
#define GPIO_AF8 8
void gpio_set(uint32_t port, uint16_t pins);
void gpio_clear(uint32_t port, uint16_t pins);
void gpio_toggle(uint32_t port, uint16_t pins);
uint16_t gpio_get(uint32_t port, uint16_t pins);
static inline void gpio_mode_setup(uint32_t p, uint8_t m, uint8_t u, uint16_t g) { (void)p; (void)m; (void)u; (void)g; }
static inline void gpio_set_af(uint32_t p, uint8_t af, uint16_t g) { (void)p; (void)af; (void)g; }
static inline void gpio_set_output_options(uint32_t p, uint8_t t, uint8_t s, uint16_t g) { (void)p; (void)t; (void)s; (void)g; }

// ---- Timers ----
#define TIM1 1u
#define TIM2 2u
#define TIM3 3u
#define TIM4 4u
#define TIM5 5u
#define TIM8 8u
enum tim_oc_id { TIM_OC1, TIM_OC2, TIM_OC3, TIM_OC4 };
enum tim_ic_id { TIM_IC1, TIM_IC2, TIM_IC3, TIM_IC4 };
enum tim_ic_input { TIM_IC_IN_TI1 = 1, TIM_IC_IN_TI2 = 2 };
enum tim_oc_mode { TIM_OCM_FROZEN, TIM_OCM_PWM1 = 6, TIM_OCM_PWM2 = 7 };
#define TIM_CR1_CKD_CK_INT 0
#define TIM_CR1_CMS_EDGE 0
#define TIM_CR1_DIR_UP 0
#define TIM_DIER_CC1IE (1u << 1)
#define TIM_SR_CC1IF (1u << 1)
#define TIM_SMCR_SMS_EM3 3
void timer_set_oc_value(uint32_t tim, enum tim_oc_id oc, uint32_t value);
uint32_t timer_get_counter(uint32_t tim);
void timer_set_counter(uint32_t tim, uint32_t count);
static inline bool timer_get_flag(uint32_t t, uint32_t f) { (void)t; (void)f; return false; }
static inline void timer_clear_flag(uint32_t t, uint32_t f) { (void)t; (void)f; }
#define timer_set_mode(...) ((void)0)
#define timer_set_prescaler(...) ((void)0)
#define timer_set_period(...) ((void)0)
#define timer_disable_preload(...) ((void)0)
#define timer_enable_preload(...) ((void)0)
#define timer_continuous_mode(...) ((void)0)
#define timer_enable_counter(...) ((void)0)
#define timer_disable_counter(...) ((void)0)
#define timer_enable_irq(...) ((void)0)
#define timer_set_oc_mode(...) ((void)0)
#define timer_enable_oc_output(...) ((void)0)
#define timer_enable_oc_preload(...) ((void)0)
#define timer_enable_break_main_output(...) ((void)0)
#define timer_set_repetition_counter(...) ((void)0)
#define timer_ic_set_input(...) ((void)0)
#define timer_slave_set_mode(...) ((void)0)
#define timer_set_clock_division(...) ((void)0)
#define timer_set_oc_polarity_high(...) ((void)0)
#define timer_set_oc_polarity_low(...) ((void)0)

// ---- ADC ----
#define ADC1 1u
#define ADC2 2u
#define ADC_CHANNEL0 0
#define ADC_CHANNEL1 1
#define ADC_CHANNEL2 2
#define ADC_CHANNEL3 3
#define ADC_CHANNEL4 4
#define ADC_CHANNEL5 5
#define ADC_CHANNEL6 6
#define ADC_CHANNEL7 7
#define ADC_CHANNEL8 8
#define ADC_CHANNEL9 9
#define ADC_CHANNEL10 10
#define ADC_CHANNEL11 11
#define ADC_CHANNEL12 12
#define ADC_CHANNEL13 13
#define ADC_CHANNEL14 14
#define ADC_CHANNEL15 15
#define ADC_CCR_ADCPRE_BY2 0
#define ADC_CR1_RES_12BIT 0
#define ADC_CR2_JEXTEN_RISING_EDGE 0
#define ADC_CR2_JSWSTART 0
#define ADC_SMPR_SMP_15CYC 0
#define ADC_SMPR_SMP_3CYC 0
#define ADC_DR(adc) (*zsim_mmio32(0x40012000u + (adc) * 0x100u + 0x4Cu))
void adc_start_conversion_injected(uint32_t adc);
uint32_t adc_read_injected(uint32_t adc, uint8_t reg);
static inline bool adc_eoc_injected(uint32_t adc) { (void)adc; return true; }
#define adc_power_off(...) ((void)0)
#define adc_power_on(...) ((void)0)
#define adc_enable_scan_mode(...) ((void)0)
#define adc_set_single_conversion_mode(...) ((void)0)
#define adc_set_continuous_conversion_mode(...) ((void)0)
#define adc_enable_external_trigger_injected(...) ((void)0)
#define adc_disable_external_trigger_regular(...) ((void)0)
#define adc_set_right_aligned(...) ((void)0)
#define adc_set_sample_time_on_all_channels(...) ((void)0)
#define adc_set_injected_sequence(...) ((void)0)
#define adc_set_regular_sequence(...) ((void)0)
#define adc_set_resolution(...) ((void)0)
#define adc_set_clk_prescale(...) ((void)0)
#define adc_set_dma_continue(...) ((void)0)
#define adc_enable_dma(...) ((void)0)
#define adc_enable_eoc_interrupt(...) ((void)0)
#define adc_start_conversion_regular(...) ((void)0)

// ---- DMA, EXTI, NVIC, SysTick, RCC, DWT ----
#define DMA2 2u
#define DMA_STREAM0 0
#define DMA_STREAM4 4
#define dma_stream_reset(...) ((void)0)
#define dma_set_transfer_mode(...) ((void)0)
#define dma_set_priority(...) ((void)0)
#define dma_set_peripheral_size(...) ((void)0)
#define dma_set_memory_size(...) ((void)0)
#define dma_set_peripheral_address(...) ((void)0)
#define dma_set_memory_address(...) ((void)0)
#define dma_set_number_of_data(...) ((void)0)
#define dma_enable_transfer_complete_interrupt(...) ((void)0)
#define dma_enable_half_transfer_interrupt(...) ((void)0)
#define dma_enable_stream(...) ((void)0)
#define dma_enable_memory_increment_mode(...) ((void)0)
#define dma_enable_circular_mode(...) ((void)0)
#define dma_channel_select(...) ((void)0)
#define EXTI12 (1u << 12)
enum exti_trigger_type { EXTI_TRIGGER_RISING, EXTI_TRIGGER_FALLING, EXTI_TRIGGER_BOTH };
#define exti_select_source(...) ((void)0)
#define exti_set_trigger(...) ((void)0)
#define exti_enable_request(...) ((void)0)
#define exti_reset_request(...) ((void)0)
#define NVIC_SYSTICK_IRQ 0
#define NVIC_TIM2_IRQ 1
#define NVIC_TIM5_IRQ 2
#define NVIC_USART3_IRQ 3
#define NVIC_EXTI15_10_IRQ 4
#define NVIC_DMA2_STREAM0_IRQ 5
#define nvic_set_priority(...) ((void)0)
#define nvic_enable_irq(...) ((void)0)
#define nvic_disable_irq(...) ((void)0)
#define systick_set_frequency(...) ((void)0)
#define systick_counter_enable(...) ((void)0)
#define systick_interrupt_enable(...) ((void)0)
extern uint32_t rcc_apb1_frequency, rcc_apb2_frequency, rcc_ahb_frequency;
#define RCC_GPIOA 0
#define RCC_GPIOB 0
#define RCC_GPIOC 0
#define RCC_GPIOD 0
#define RCC_SYSCFG 0
#define RCC_USART3 0
#define RCC_SPI3 0
#define RCC_TIM1 0
#define RCC_TIM2 0
#define RCC_TIM3 0
#define RCC_TIM4 0
#define RCC_TIM5 0
#define RCC_TIM8 0
#define RCC_DMA1 0
#define RCC_DMA2 0
#define RCC_ADC1 0
#define RCC_ADC2 0
#define RST_TIM2 0
#define RST_TIM5 0
#define RCC_AHB1ENR 0
#define RCC_APB2ENR 0
#define RCC_AHB1ENR_DMA2EN 0
#define RCC_APB2ENR_ADC1EN 0
#define RCC_CLOCK_3V3_168MHZ 0
struct rcc_clock_scale { uint32_t dummy; };
extern const struct rcc_clock_scale rcc_hse_8mhz_3v3[];
#define rcc_clock_setup_pll(...) ((void)0)
#define rcc_periph_clock_enable(...) ((void)0)
#define rcc_peripheral_enable_clock(...) ((void)0)
#define rcc_periph_reset_pulse(...) ((void)0)
uint32_t dwt_read_cycle_counter(void);
static inline bool dwt_enable_cycle_counter(void) { return true; }

// ---- SPI, USART, flash ----
#define SPI3 3u
#define SPI_CR1_BAUDRATE_FPCLK_DIV_8 0
#define SPI_CR1_BAUDRATE_FPCLK_DIV_128 0
#define SPI_CR1_CPHA_CLK_TRANSITION_1 0
#define SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE 0
#define SPI_CR1_DFF_8BIT 0
#define SPI_CR1_MSBFIRST 0
void spi_send(uint32_t spi, uint16_t data);
uint16_t spi_read(uint32_t spi);
uint16_t spi_xfer(uint32_t spi, uint16_t data);
#define spi_reset(...) ((void)0)
#define spi_init_master(...) ((void)0)
#define spi_enable_software_slave_management(...) ((void)0)
#define spi_set_nss_high(...) ((void)0)
#define spi_enable(...) ((void)0)
#define USART1 1u
#define USART3 3u
#define USART_CR1(u) (*zsim_mmio32(0x40004800u + 0x0Cu))
#define USART_CR1_RXNEIE (1u << 5)
#define USART_FLOWCONTROL_NONE 0
#define USART_MODE_TX_RX 0
#define USART_PARITY_NONE 0
#define USART_STOPBITS_1 0
void usart_send_blocking(uint32_t usart, uint16_t data);
void usart_send(uint32_t usart, uint16_t data);
static inline uint16_t usart_recv(uint32_t usart) { (void)usart; return 0; }
#define usart_set_baudrate(...) ((void)0)
#define usart_set_databits(...) ((void)0)
#define usart_set_stopbits(...) ((void)0)
#define usart_set_mode(...) ((void)0)
#define usart_set_parity(...) ((void)0)
#define usart_set_flow_control(...) ((void)0)
#define usart_enable(...) ((void)0)
#define usart_enable_tx_interrupt(...) ((void)0)
#define usart_enable_rx_interrupt(...) ((void)0)
#define FLASH_CR_PROGRAM_X16 1
#define FLASH_CR_PROGRAM_X32 2
void flash_unlock(void);
void flash_lock(void);
void flash_erase_sector(uint8_t sector, uint32_t program_size);
void flash_program_word(uint32_t address, uint32_t data);

#endif
