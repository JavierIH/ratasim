// KOGUMA's STM32F446RE as its firmware sees it, in place of ST's HAL and CMSIS headers (its Drivers/ are not
// used): the types, registers, constants and functions its files use, its peripherals as host memory. kchip.c gives
// the functions and the time.
#ifndef KSIM_STM32F4XX_HAL_H
#define KSIM_STM32F4XX_HAL_H

#include <stddef.h>
#include <stdint.h>

#define __IO volatile
#define __I volatile const
#define __O volatile

typedef enum { RESET = 0U, SET = !RESET } FlagStatus, ITStatus;
typedef enum { DISABLE = 0U, ENABLE = !DISABLE } FunctionalState;
typedef enum { SUCCESS = 0U, ERROR = !SUCCESS } ErrorStatus;
typedef enum { HAL_OK = 0x00U, HAL_ERROR = 0x01U, HAL_BUSY = 0x02U, HAL_TIMEOUT = 0x03U } HAL_StatusTypeDef;
typedef enum { HAL_UNLOCKED = 0x00U, HAL_LOCKED = 0x01U } HAL_LockTypeDef;
#define HAL_MAX_DELAY 0xFFFFFFFFU

typedef enum {
  NonMaskableInt_IRQn = -14, MemoryManagement_IRQn = -12, BusFault_IRQn = -11, UsageFault_IRQn = -10,
  SVCall_IRQn = -5, DebugMonitor_IRQn = -4, PendSV_IRQn = -2, SysTick_IRQn = -1,
  TIM2_IRQn = 28, TIM3_IRQn = 29, TIM4_IRQn = 30, SPI1_IRQn = 35, USART2_IRQn = 38, SPI3_IRQn = 51,
  TIM6_DAC_IRQn = 54, DMA2_Stream0_IRQn = 56
} IRQn_Type;

// ---- Registers ---------------------------------------------------------------------------------------------------
typedef struct {
  __IO uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
} GPIO_TypeDef;

typedef struct {
  __IO uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3, CCR4, BDTR,
      DCR, DMAR, OR;
} TIM_TypeDef;

typedef struct {
  __IO uint32_t SR, CR1, CR2, SMPR1, SMPR2, JOFR1, JOFR2, JOFR3, JOFR4, HTR, LTR, SQR1, SQR2, SQR3, JSQR, JDR1, JDR2,
      JDR3, JDR4, DR;
} ADC_TypeDef;

typedef struct {
  __IO uint32_t CR1, CR2, SR, DR, CRCPR, RXCRCR, TXCRCR, I2SCFGR, I2SPR;
} SPI_TypeDef;

typedef struct {
  __IO uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} USART_TypeDef;

typedef struct {
  __IO uint32_t CR, NDTR, PAR, M0AR, M1AR, FCR;
} DMA_Stream_TypeDef;

typedef struct {
  __IO uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR, RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2],
      AHB1ENR, AHB2ENR, AHB3ENR, RESERVED2, APB1ENR, APB2ENR;
} RCC_TypeDef;

typedef struct { __IO uint32_t CTRL, CYCCNT; } DWT_Type;
typedef struct { __IO uint32_t DHCSR, DCRSR, DCRDR, DEMCR; } CoreDebug_Type;
#define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)

extern GPIO_TypeDef ksim_gpio[8];
extern TIM_TypeDef ksim_tim[7];        // TIM1..TIM7 (index 0 unused: TIM1)
extern ADC_TypeDef ksim_adc1;
extern SPI_TypeDef ksim_spi[4];        // SPI1..SPI3 (index 0 unused)
extern USART_TypeDef ksim_usart2;
extern DMA_Stream_TypeDef ksim_dma2_stream0;
extern RCC_TypeDef ksim_rcc;
extern DWT_Type ksim_dwt;
extern CoreDebug_Type ksim_coredebug;

#define GPIOA (&ksim_gpio[0])
#define GPIOB (&ksim_gpio[1])
#define GPIOC (&ksim_gpio[2])
#define GPIOD (&ksim_gpio[3])
#define GPIOE (&ksim_gpio[4])
#define GPIOF (&ksim_gpio[5])
#define GPIOG (&ksim_gpio[6])
#define GPIOH (&ksim_gpio[7])
#define TIM2 (&ksim_tim[2])
#define TIM3 (&ksim_tim[3])
#define TIM4 (&ksim_tim[4])
#define TIM5 (&ksim_tim[5])
#define TIM6 (&ksim_tim[6])
#define ADC1 (&ksim_adc1)
#define SPI1 (&ksim_spi[1])
#define SPI3 (&ksim_spi[3])
#define USART2 (&ksim_usart2)
#define DMA2_Stream0 (&ksim_dma2_stream0)
#define RCC (&ksim_rcc)
#define DWT (&ksim_dwt)
#define CoreDebug (&ksim_coredebug)

#define TIM_CR1_CEN (1UL << 0)
#define TIM_DIER_UIE (1UL << 0)
#define TIM_SR_UIF (1UL << 0)

extern uint32_t SystemCoreClock;
void __disable_irq(void);
void __enable_irq(void);
#define __NOP() ((void)0)
#define __DSB() ((void)0)
#define __ISB() ((void)0)

// ---- GPIO --------------------------------------------------------------------------------------------------------
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define GPIO_PIN_0 ((uint16_t)0x0001)
#define GPIO_PIN_1 ((uint16_t)0x0002)
#define GPIO_PIN_2 ((uint16_t)0x0004)
#define GPIO_PIN_3 ((uint16_t)0x0008)
#define GPIO_PIN_4 ((uint16_t)0x0010)
#define GPIO_PIN_5 ((uint16_t)0x0020)
#define GPIO_PIN_6 ((uint16_t)0x0040)
#define GPIO_PIN_7 ((uint16_t)0x0080)
#define GPIO_PIN_8 ((uint16_t)0x0100)
#define GPIO_PIN_9 ((uint16_t)0x0200)
#define GPIO_PIN_10 ((uint16_t)0x0400)
#define GPIO_PIN_11 ((uint16_t)0x0800)
#define GPIO_PIN_12 ((uint16_t)0x1000)
#define GPIO_PIN_13 ((uint16_t)0x2000)
#define GPIO_PIN_14 ((uint16_t)0x4000)
#define GPIO_PIN_15 ((uint16_t)0x8000)
#define GPIO_PIN_All ((uint16_t)0xFFFF)
#define GPIO_MODE_INPUT 0x0U
#define GPIO_MODE_OUTPUT_PP 0x1U
#define GPIO_MODE_OUTPUT_OD 0x11U
#define GPIO_MODE_AF_PP 0x2U
#define GPIO_MODE_AF_OD 0x12U
#define GPIO_MODE_ANALOG 0x3U
#define GPIO_NOPULL 0x0U
#define GPIO_PULLUP 0x1U
#define GPIO_PULLDOWN 0x2U
#define GPIO_SPEED_FREQ_LOW 0x0U
#define GPIO_SPEED_FREQ_MEDIUM 0x1U
#define GPIO_SPEED_FREQ_HIGH 0x2U
#define GPIO_SPEED_FREQ_VERY_HIGH 0x3U
#define GPIO_AF1_TIM2 0x01U
#define GPIO_AF2_TIM3 0x02U
#define GPIO_AF2_TIM4 0x02U
#define GPIO_AF5_SPI1 0x05U
#define GPIO_AF6_SPI3 0x06U
#define GPIO_AF7_SPI3 0x07U
#define GPIO_AF7_USART2 0x07U
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void HAL_GPIO_DeInit(GPIO_TypeDef *port, uint32_t pins);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state);
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pins);

// ---- Clocks, power, flash, NVIC, the tick ------------------------------------------------------------------------
typedef struct { uint32_t PLLState, PLLSource, PLLM, PLLN, PLLP, PLLQ, PLLR; } RCC_PLLInitTypeDef;
typedef struct {
  uint32_t OscillatorType, HSEState, LSEState, HSIState, HSICalibrationValue, LSIState;
  RCC_PLLInitTypeDef PLL;
} RCC_OscInitTypeDef;
typedef struct { uint32_t ClockType, SYSCLKSource, AHBCLKDivider, APB1CLKDivider, APB2CLKDivider; } RCC_ClkInitTypeDef;
#define RCC_OSCILLATORTYPE_HSE 0x1U
#define RCC_HSE_ON 0x1U
#define RCC_PLL_ON 0x2U
#define RCC_PLLSOURCE_HSE 0x400000U
#define RCC_PLLP_DIV2 0x2U
#define RCC_CLOCKTYPE_SYSCLK 0x1U
#define RCC_CLOCKTYPE_HCLK 0x2U
#define RCC_CLOCKTYPE_PCLK1 0x4U
#define RCC_CLOCKTYPE_PCLK2 0x8U
#define RCC_SYSCLKSOURCE_PLLCLK 0x2U
#define RCC_SYSCLK_DIV1 0x0U
#define RCC_HCLK_DIV2 0x1000U
#define RCC_HCLK_DIV4 0x1400U
#define FLASH_LATENCY_5 0x5U
#define PWR_REGULATOR_VOLTAGE_SCALE1 0xC000U
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *init);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *init, uint32_t latency);
uint32_t HAL_RCC_GetHCLKFreq(void);
HAL_StatusTypeDef HAL_PWREx_EnableOverDrive(void);
#define __KSIM_CLK(x) ((void)0)
#define __HAL_RCC_PWR_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_SYSCFG_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_GPIOH_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_DMA2_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_ADC1_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_ADC1_CLK_DISABLE() __KSIM_CLK(0)
#define __HAL_RCC_SPI1_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_SPI1_CLK_DISABLE() __KSIM_CLK(0)
#define __HAL_RCC_SPI3_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_SPI3_CLK_DISABLE() __KSIM_CLK(0)
#define __HAL_RCC_TIM2_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_TIM2_CLK_DISABLE() __KSIM_CLK(0)
#define __HAL_RCC_TIM3_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_TIM3_CLK_DISABLE() __KSIM_CLK(0)
#define __HAL_RCC_TIM4_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_TIM4_CLK_DISABLE() __KSIM_CLK(0)
#define __HAL_RCC_TIM6_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_TIM6_CLK_DISABLE() __KSIM_CLK(0)
#define __HAL_RCC_USART2_CLK_ENABLE() __KSIM_CLK(0)
#define __HAL_RCC_USART2_CLK_DISABLE() __KSIM_CLK(0)
#define __HAL_PWR_VOLTAGESCALING_CONFIG(x) ((void)(x))
#define NVIC_PRIORITYGROUP_4 0x3U
HAL_StatusTypeDef HAL_Init(void);
void HAL_MspInit(void);
void HAL_IncTick(void);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
void HAL_NVIC_DisableIRQ(IRQn_Type irq);
void HAL_NVIC_SetPriorityGrouping(uint32_t group);

// ---- DMA ---------------------------------------------------------------------------------------------------------
typedef struct {
  uint32_t Channel, Direction, PeriphInc, MemInc, PeriphDataAlignment, MemDataAlignment, Mode, Priority, FIFOMode,
      FIFOThreshold, MemBurst, PeriphBurst;
} DMA_InitTypeDef;
typedef struct __DMA_HandleTypeDef {
  DMA_Stream_TypeDef *Instance;
  DMA_InitTypeDef Init;
  HAL_LockTypeDef Lock;
  volatile uint32_t State;
  void *Parent;
  void (*XferCpltCallback)(struct __DMA_HandleTypeDef *hdma);
  volatile uint32_t ErrorCode;
} DMA_HandleTypeDef;
#define DMA_CHANNEL_0 0x0U
#define DMA_PERIPH_TO_MEMORY 0x0U
#define DMA_PINC_DISABLE 0x0U
#define DMA_MINC_ENABLE 0x400U
#define DMA_PDATAALIGN_HALFWORD 0x800U
#define DMA_MDATAALIGN_HALFWORD 0x2000U
#define DMA_NORMAL 0x0U
#define DMA_PRIORITY_LOW 0x0U
#define DMA_FIFOMODE_DISABLE 0x0U
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
#define __HAL_LINKDMA(handle, field, dma) \
  do {                                     \
    (handle)->field = &(dma);              \
    (dma).Parent = (handle);               \
  } while (0)

// ---- ADC ---------------------------------------------------------------------------------------------------------
typedef struct {
  uint32_t ClockPrescaler, Resolution, DataAlign, ScanConvMode, EOCSelection;
  FunctionalState ContinuousConvMode;
  uint32_t NbrOfConversion;
  FunctionalState DiscontinuousConvMode;
  uint32_t NbrOfDiscConversion, ExternalTrigConv, ExternalTrigConvEdge;
  FunctionalState DMAContinuousRequests;
} ADC_InitTypeDef;
typedef struct { uint32_t Channel, Rank, SamplingTime, Offset; } ADC_ChannelConfTypeDef;
typedef struct {
  ADC_TypeDef *Instance;
  ADC_InitTypeDef Init;
  volatile uint32_t NbrOfCurrentConversionRank;
  DMA_HandleTypeDef *DMA_Handle;
  HAL_LockTypeDef Lock;
  volatile uint32_t State, ErrorCode;
} ADC_HandleTypeDef;
#define ADC_CLOCK_SYNC_PCLK_DIV8 0x30000U
#define ADC_RESOLUTION_12B 0x0U
#define ADC_DATAALIGN_RIGHT 0x0U
#define ADC_EOC_SINGLE_CONV 0x1U
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0x0U
#define ADC_SOFTWARE_START 0x0F000001U
#define ADC_CHANNEL_9 9U
#define ADC_CHANNEL_10 10U
#define ADC_CHANNEL_11 11U
#define ADC_CHANNEL_12 12U
#define ADC_CHANNEL_13 13U
#define ADC_SAMPLETIME_3CYCLES 0x0U
#define ADC_SAMPLETIME_15CYCLES 0x1U
#define ADC_SAMPLETIME_28CYCLES 0x2U
#define ADC_SAMPLETIME_56CYCLES 0x3U
#define ADC_SAMPLETIME_84CYCLES 0x4U
#define ADC_SAMPLETIME_112CYCLES 0x5U
#define ADC_SAMPLETIME_144CYCLES 0x6U
#define ADC_SAMPLETIME_480CYCLES 0x7U
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *config);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t timeout);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *data, uint32_t length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc);
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc);
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

// ---- SPI ---------------------------------------------------------------------------------------------------------
typedef struct {
  uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS, BaudRatePrescaler, FirstBit, TIMode,
      CRCCalculation, CRCPolynomial;
} SPI_InitTypeDef;
typedef struct {
  SPI_TypeDef *Instance;
  SPI_InitTypeDef Init;
  HAL_LockTypeDef Lock;
  volatile uint32_t State, ErrorCode;
} SPI_HandleTypeDef;
#define SPI_MODE_MASTER 0x104U
#define SPI_DIRECTION_2LINES 0x0U
#define SPI_DATASIZE_8BIT 0x0U
#define SPI_POLARITY_HIGH 0x2U
#define SPI_PHASE_2EDGE 0x1U
#define SPI_NSS_SOFT 0x200U
#define SPI_BAUDRATEPRESCALER_2 0x0U
#define SPI_BAUDRATEPRESCALER_8 0x10U
#define SPI_FIRSTBIT_MSB 0x0U
#define SPI_TIMODE_DISABLE 0x0U
#define SPI_CRCCALCULATION_DISABLE 0x0U
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *tx, uint8_t *rx, uint16_t size,
                                          uint32_t timeout);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);

// ---- TIM ---------------------------------------------------------------------------------------------------------
typedef struct { uint32_t Prescaler, CounterMode, Period, ClockDivision, RepetitionCounter, AutoReloadPreload; } TIM_Base_InitTypeDef;
typedef struct { uint32_t OCMode, Pulse, OCPolarity, OCNPolarity, OCFastMode, OCIdleState, OCNIdleState; } TIM_OC_InitTypeDef;
typedef struct { uint32_t MasterOutputTrigger, MasterSlaveMode; } TIM_MasterConfigTypeDef;
typedef enum { HAL_TIM_ACTIVE_CHANNEL_CLEARED = 0 } HAL_TIM_ActiveChannel;
typedef struct {
  TIM_TypeDef *Instance;
  TIM_Base_InitTypeDef Init;
  HAL_TIM_ActiveChannel Channel;
  DMA_HandleTypeDef *hdma[7];
  HAL_LockTypeDef Lock;
  volatile uint32_t State;
} TIM_HandleTypeDef;
#define TIM_COUNTERMODE_UP 0x0U
#define TIM_CLOCKDIVISION_DIV1 0x0U
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0x0U
#define TIM_AUTORELOAD_PRELOAD_ENABLE 0x80U
#define TIM_TRGO_RESET 0x0U
#define TIM_TRGO_UPDATE 0x20U
#define TIM_MASTERSLAVEMODE_DISABLE 0x0U
#define TIM_OCMODE_PWM1 0x60U
#define TIM_OCPOLARITY_HIGH 0x0U
#define TIM_OCFAST_DISABLE 0x0U
#define TIM_CHANNEL_1 0x0U
#define TIM_CHANNEL_2 0x4U
#define TIM_CHANNEL_3 0x8U
#define TIM_CHANNEL_4 0xCU
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *config, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim, TIM_MasterConfigTypeDef *config);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
#define __HAL_TIM_SET_COMPARE(h, ch, v) (*(&(h)->Instance->CCR1 + ((ch) >> 2)) = (v))
#define __HAL_TIM_GET_COMPARE(h, ch) (*(&(h)->Instance->CCR1 + ((ch) >> 2)))
// The counter and the period through the chip (kchip.c): a timer's next update moves with them.
void ksim_tim_set_autoreload(TIM_HandleTypeDef *htim, uint32_t arr);
void ksim_tim_set_counter(TIM_TypeDef *tim, uint32_t cnt);
#define __HAL_TIM_SET_AUTORELOAD(h, v) ksim_tim_set_autoreload((h), (v))
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Instance->ARR)
#define __HAL_TIM_SET_COUNTER(h, v) ksim_tim_set_counter((h)->Instance, (v))
#define __HAL_TIM_GET_COUNTER(h) ((h)->Instance->CNT)

// ---- UART --------------------------------------------------------------------------------------------------------
typedef struct { uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling; } UART_InitTypeDef;
typedef struct {
  USART_TypeDef *Instance;
  UART_InitTypeDef Init;
  HAL_LockTypeDef Lock;
  volatile uint32_t gState, RxState, ErrorCode;
} UART_HandleTypeDef;
#define UART_WORDLENGTH_8B 0x0U
#define UART_STOPBITS_1 0x0U
#define UART_PARITY_NONE 0x0U
#define UART_MODE_TX 0x8U
#define UART_HWCONTROL_NONE 0x0U
#define UART_OVERSAMPLING_16 0x0U
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size, uint32_t timeout);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);

#endif
