/* Futura's chip for the host: the part of ST's STM32F4xx Standard Peripheral Library (and of CMSIS) its firmware
 * template uses, written here from its API (names, structures, constants), on the simulated chip of fchip.c: each
 * peripheral it touches through a pointer (GPIOx, TIMx, ADC1, SPI2, USART1) is host memory, SysTick is seen through
 * fsim_systick() (its counter at the time it is read), and the library's functions are fchip.c's. Nothing of ST's
 * own files is used. */
#ifndef __STM32F4xx_H
#define __STM32F4xx_H

#include <stdint.h>

#define __IO volatile
#define __I volatile const
#define __O volatile
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef enum { RESET = 0, SET = !RESET } FlagStatus, ITStatus;
typedef enum { DISABLE = 0, ENABLE = !DISABLE } FunctionalState;
typedef enum { ERROR = 0, SUCCESS = !ERROR } ErrorStatus;

/* ---- the simulated chip (fchip.c) ---- */
volatile u32 *fsim_millis(void);        /* the firmware's Millis (delay.c), read through it: time passes */
extern uint32_t SystemCoreClock;

/* ---- interrupts ---- */
typedef enum { SysTick_IRQn = -1, EXTI9_5_IRQn = 23, EXTI15_10_IRQn = 40 } IRQn_Type;
#define NVIC_PriorityGroup_4 ((uint32_t)0x300)
typedef struct {
  uint8_t NVIC_IRQChannel, NVIC_IRQChannelPreemptionPriority, NVIC_IRQChannelSubPriority;
  FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;
void NVIC_PriorityGroupConfig(uint32_t group);
void NVIC_Init(NVIC_InitTypeDef *init);
void NVIC_SetPriority(IRQn_Type irq, uint32_t priority);

typedef struct { __IO uint32_t CTRL, LOAD, VAL; __I uint32_t CALIB; } SysTick_Type;
SysTick_Type *fsim_systick(void);
#define SysTick (fsim_systick())
uint32_t SysTick_Config(uint32_t ticks);
void SystemInit(void);
void SystemCoreClockUpdate(void);

/* ---- RCC ---- */
#define RCC_AHB1Periph_GPIOA ((uint32_t)0x01)
#define RCC_AHB1Periph_GPIOB ((uint32_t)0x02)
#define RCC_AHB1Periph_GPIOC ((uint32_t)0x04)
#define RCC_AHB1Periph_GPIOD ((uint32_t)0x08)
#define RCC_APB1Periph_TIM2 ((uint32_t)0x01)
#define RCC_APB1Periph_TIM3 ((uint32_t)0x02)
#define RCC_APB1Periph_TIM4 ((uint32_t)0x04)
#define RCC_APB1Periph_TIM5 ((uint32_t)0x08)
#define RCC_APB1Periph_SPI2 ((uint32_t)0x4000)
#define RCC_APB1Periph_USART2 ((uint32_t)0x20000)
#define RCC_APB2Periph_USART1 ((uint32_t)0x10)
#define RCC_APB2Periph_ADC1 ((uint32_t)0x100)
#define RCC_APB2Periph_SYSCFG ((uint32_t)0x4000)
void RCC_AHB1PeriphClockCmd(uint32_t periph, FunctionalState state);
void RCC_APB1PeriphClockCmd(uint32_t periph, FunctionalState state);
void RCC_APB2PeriphClockCmd(uint32_t periph, FunctionalState state);

/* ---- GPIO ---- */
typedef struct {
  __IO uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR;
  __IO uint16_t BSRRL, BSRRH;
  __IO uint32_t LCKR, AFR[2];
} GPIO_TypeDef;
extern GPIO_TypeDef fsim_gpio[4];
#define GPIOA (&fsim_gpio[0])
#define GPIOB (&fsim_gpio[1])
#define GPIOC (&fsim_gpio[2])
#define GPIOD (&fsim_gpio[3])
#define GPIO_Pin_0 ((uint16_t)0x0001)
#define GPIO_Pin_1 ((uint16_t)0x0002)
#define GPIO_Pin_2 ((uint16_t)0x0004)
#define GPIO_Pin_3 ((uint16_t)0x0008)
#define GPIO_Pin_4 ((uint16_t)0x0010)
#define GPIO_Pin_5 ((uint16_t)0x0020)
#define GPIO_Pin_6 ((uint16_t)0x0040)
#define GPIO_Pin_7 ((uint16_t)0x0080)
#define GPIO_Pin_8 ((uint16_t)0x0100)
#define GPIO_Pin_9 ((uint16_t)0x0200)
#define GPIO_Pin_10 ((uint16_t)0x0400)
#define GPIO_Pin_11 ((uint16_t)0x0800)
#define GPIO_Pin_12 ((uint16_t)0x1000)
#define GPIO_Pin_13 ((uint16_t)0x2000)
#define GPIO_Pin_14 ((uint16_t)0x4000)
#define GPIO_Pin_15 ((uint16_t)0x8000)
#define GPIO_Pin_All ((uint16_t)0xFFFF)
#define GPIO_PinSource0 ((uint8_t)0)
#define GPIO_PinSource1 ((uint8_t)1)
#define GPIO_PinSource2 ((uint8_t)2)
#define GPIO_PinSource3 ((uint8_t)3)
#define GPIO_PinSource4 ((uint8_t)4)
#define GPIO_PinSource5 ((uint8_t)5)
#define GPIO_PinSource6 ((uint8_t)6)
#define GPIO_PinSource7 ((uint8_t)7)
#define GPIO_PinSource8 ((uint8_t)8)
#define GPIO_PinSource9 ((uint8_t)9)
#define GPIO_PinSource10 ((uint8_t)10)
#define GPIO_PinSource11 ((uint8_t)11)
#define GPIO_PinSource12 ((uint8_t)12)
#define GPIO_PinSource13 ((uint8_t)13)
#define GPIO_PinSource14 ((uint8_t)14)
#define GPIO_PinSource15 ((uint8_t)15)
typedef enum { GPIO_Mode_IN = 0, GPIO_Mode_OUT = 1, GPIO_Mode_AF = 2, GPIO_Mode_AN = 3 } GPIOMode_TypeDef;
typedef enum { GPIO_OType_PP = 0, GPIO_OType_OD = 1 } GPIOOType_TypeDef;
typedef enum { GPIO_Speed_2MHz = 0, GPIO_Speed_25MHz = 1, GPIO_Speed_50MHz = 2, GPIO_Speed_100MHz = 3 } GPIOSpeed_TypeDef;
typedef enum { GPIO_PuPd_NOPULL = 0, GPIO_PuPd_UP = 1, GPIO_PuPd_DOWN = 2 } GPIOPuPd_TypeDef;
typedef struct {
  uint32_t GPIO_Pin;
  GPIOMode_TypeDef GPIO_Mode;
  GPIOSpeed_TypeDef GPIO_Speed;
  GPIOOType_TypeDef GPIO_OType;
  GPIOPuPd_TypeDef GPIO_PuPd;
} GPIO_InitTypeDef;
#define GPIO_AF_TIM2 ((uint8_t)0x01)
#define GPIO_AF_TIM3 ((uint8_t)0x02)
#define GPIO_AF_TIM4 ((uint8_t)0x02)
#define GPIO_AF_TIM5 ((uint8_t)0x02)
#define GPIO_AF_SPI2 ((uint8_t)0x05)
#define GPIO_AF_USART1 ((uint8_t)0x07)
#define GPIO_AF_USART2 ((uint8_t)0x07)
void GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void GPIO_PinAFConfig(GPIO_TypeDef *port, uint16_t source, uint8_t af);
void GPIO_SetBits(GPIO_TypeDef *port, uint16_t pins);
void GPIO_ResetBits(GPIO_TypeDef *port, uint16_t pins);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *port, uint16_t pin);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef *port, uint16_t pin);
void GPIO_ToggleBits(GPIO_TypeDef *port, uint16_t pins);

/* ---- EXTI, SYSCFG ---- */
#define EXTI_Line5 ((uint32_t)0x00020)
#define EXTI_Line13 ((uint32_t)0x02000)
typedef enum { EXTI_Mode_Interrupt = 0x00, EXTI_Mode_Event = 0x04 } EXTIMode_TypeDef;
typedef enum { EXTI_Trigger_Rising = 0x08, EXTI_Trigger_Falling = 0x0C, EXTI_Trigger_Rising_Falling = 0x10 } EXTITrigger_TypeDef;
typedef struct {
  uint32_t EXTI_Line;
  EXTIMode_TypeDef EXTI_Mode;
  EXTITrigger_TypeDef EXTI_Trigger;
  FunctionalState EXTI_LineCmd;
} EXTI_InitTypeDef;
void EXTI_Init(EXTI_InitTypeDef *init);
ITStatus EXTI_GetITStatus(uint32_t line);
void EXTI_ClearITPendingBit(uint32_t line);
#define EXTI_PortSourceGPIOA ((uint8_t)0x00)
#define EXTI_PortSourceGPIOB ((uint8_t)0x01)
#define EXTI_PortSourceGPIOC ((uint8_t)0x02)
#define EXTI_PinSource5 ((uint8_t)0x05)
#define EXTI_PinSource13 ((uint8_t)0x0D)
void SYSCFG_EXTILineConfig(uint8_t port, uint8_t pin);

/* ---- TIM ---- */
typedef struct {
  __IO uint16_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER;
  __IO uint32_t CNT;
  __IO uint16_t PSC;
  __IO uint32_t ARR;
  __IO uint16_t RCR;
  __IO uint32_t CCR1, CCR2, CCR3, CCR4;
  __IO uint16_t BDTR, DCR, DMAR, OR;
} TIM_TypeDef;
extern TIM_TypeDef fsim_tim[6];
#define TIM2 (&fsim_tim[2])
#define TIM3 (&fsim_tim[3])
#define TIM4 (&fsim_tim[4])
#define TIM5 (&fsim_tim[5])
#define TIM_CR1_CEN ((uint16_t)0x0001)
#define TIM_CounterMode_Up ((uint16_t)0x0000)
#define TIM_OCMode_PWM1 ((uint16_t)0x0060)
#define TIM_OutputState_Enable ((uint16_t)0x0001)
#define TIM_OutputState_Disable ((uint16_t)0x0000)
#define TIM_OCPolarity_High ((uint16_t)0x0000)
#define TIM_OCPolarity_Low ((uint16_t)0x0002)
#define TIM_OCPreload_Enable ((uint16_t)0x0008)
#define TIM_OCPreload_Disable ((uint16_t)0x0000)
#define TIM_EncoderMode_TI1 ((uint16_t)0x0001)
#define TIM_EncoderMode_TI2 ((uint16_t)0x0002)
#define TIM_EncoderMode_TI12 ((uint16_t)0x0003)
#define TIM_ICPolarity_Rising ((uint16_t)0x0000)
#define TIM_ICPolarity_Falling ((uint16_t)0x0002)
typedef struct {
  uint16_t TIM_Prescaler, TIM_CounterMode;
  uint32_t TIM_Period;
  uint16_t TIM_ClockDivision;
  uint8_t TIM_RepetitionCounter;
} TIM_TimeBaseInitTypeDef;
typedef struct {
  uint16_t TIM_OCMode, TIM_OutputState, TIM_OutputNState;
  uint32_t TIM_Pulse;
  uint16_t TIM_OCPolarity, TIM_OCNPolarity, TIM_OCIdleState, TIM_OCNIdleState;
} TIM_OCInitTypeDef;
void TIM_TimeBaseInit(TIM_TypeDef *tim, TIM_TimeBaseInitTypeDef *init);
void TIM_OC1Init(TIM_TypeDef *tim, TIM_OCInitTypeDef *init);
void TIM_OC2Init(TIM_TypeDef *tim, TIM_OCInitTypeDef *init);
void TIM_OC3Init(TIM_TypeDef *tim, TIM_OCInitTypeDef *init);
void TIM_OC4Init(TIM_TypeDef *tim, TIM_OCInitTypeDef *init);
void TIM_OC1PreloadConfig(TIM_TypeDef *tim, uint16_t preload);
void TIM_OC2PreloadConfig(TIM_TypeDef *tim, uint16_t preload);
void TIM_OC3PreloadConfig(TIM_TypeDef *tim, uint16_t preload);
void TIM_OC4PreloadConfig(TIM_TypeDef *tim, uint16_t preload);
void TIM_ARRPreloadConfig(TIM_TypeDef *tim, FunctionalState state);
void TIM_Cmd(TIM_TypeDef *tim, FunctionalState state);
void TIM_SetAutoreload(TIM_TypeDef *tim, uint32_t autoreload);
void TIM_SetCounter(TIM_TypeDef *tim, uint32_t counter);
uint32_t TIM_GetCounter(TIM_TypeDef *tim);
void TIM_EncoderInterfaceConfig(TIM_TypeDef *tim, uint16_t mode, uint16_t ic1, uint16_t ic2);

/* ---- ADC ---- */
typedef struct {
  __IO uint32_t SR, CR1, CR2, SMPR1, SMPR2, JOFR1, JOFR2, JOFR3, JOFR4, HTR, LTR, SQR1, SQR2, SQR3, JSQR, JDR1, JDR2,
                JDR3, JDR4, DR;
} ADC_TypeDef;
extern ADC_TypeDef fsim_adc1;
#define ADC1 (&fsim_adc1)
#define ADC_Mode_Independent ((uint32_t)0x00000000)
#define ADC_Prescaler_Div2 ((uint32_t)0x00000000)
#define ADC_Prescaler_Div4 ((uint32_t)0x00010000)
#define ADC_Prescaler_Div6 ((uint32_t)0x00020000)
#define ADC_Prescaler_Div8 ((uint32_t)0x00030000)
#define ADC_DMAAccessMode_Disabled ((uint32_t)0x00000000)
#define ADC_TwoSamplingDelay_20Cycles ((uint32_t)0x00000F00)
#define ADC_Resolution_12b ((uint32_t)0x00000000)
#define ADC_DataAlign_Right ((uint32_t)0x00000000)
#define ADC_IT_EOC ((uint16_t)0x0205)
#define ADC_FLAG_EOC ((uint8_t)0x02)
#define ADC_SampleTime_3Cycles ((uint8_t)0x00)
#define ADC_SampleTime_15Cycles ((uint8_t)0x01)
#define ADC_SampleTime_28Cycles ((uint8_t)0x02)
#define ADC_SampleTime_56Cycles ((uint8_t)0x03)
#define ADC_SampleTime_84Cycles ((uint8_t)0x04)
#define ADC_SampleTime_112Cycles ((uint8_t)0x05)
#define ADC_SampleTime_144Cycles ((uint8_t)0x06)
#define ADC_SampleTime_480Cycles ((uint8_t)0x07)
typedef struct {
  uint32_t ADC_Mode, ADC_Prescaler, ADC_DMAAccessMode, ADC_TwoSamplingDelay;
} ADC_CommonInitTypeDef;
typedef struct {
  uint32_t ADC_Resolution;
  FunctionalState ADC_ScanConvMode, ADC_ContinuousConvMode;
  uint32_t ADC_ExternalTrigConvEdge, ADC_ExternalTrigConv, ADC_DataAlign;
  uint8_t ADC_NbrOfConversion;
} ADC_InitTypeDef;
void ADC_CommonInit(ADC_CommonInitTypeDef *init);
void ADC_Init(ADC_TypeDef *adc, ADC_InitTypeDef *init);
void ADC_ITConfig(ADC_TypeDef *adc, uint16_t it, FunctionalState state);
void ADC_Cmd(ADC_TypeDef *adc, FunctionalState state);
void ADC_RegularChannelConfig(ADC_TypeDef *adc, uint8_t channel, uint8_t rank, uint8_t sample_time);
void ADC_SoftwareStartConv(ADC_TypeDef *adc);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef *adc, uint8_t flag);
uint16_t ADC_GetConversionValue(ADC_TypeDef *adc);

/* ---- SPI ---- */
typedef struct {
  __IO uint16_t CR1, CR2, SR, DR, CRCPR, RXCRCR, TXCRCR, I2SCFGR, I2SPR;
} SPI_TypeDef;
extern SPI_TypeDef fsim_spi2;
#define SPI2 (&fsim_spi2)
#define SPI_Mode_Master ((uint16_t)0x0104)
#define SPI_Direction_1Line_Tx ((uint16_t)0xC000)
#define SPI_NSS_Soft ((uint16_t)0x0200)
#define SPI_BaudRatePrescaler_2 ((uint16_t)0x0000)
#define SPI_I2S_FLAG_TXE ((uint16_t)0x0002)
#define SPI_I2S_FLAG_BSY ((uint16_t)0x0080)
typedef struct {
  uint16_t SPI_Direction, SPI_Mode, SPI_DataSize, SPI_CPOL, SPI_CPHA, SPI_NSS, SPI_BaudRatePrescaler, SPI_FirstBit,
           SPI_CRCPolynomial;
} SPI_InitTypeDef;
void SPI_I2S_DeInit(SPI_TypeDef *spi);
void SPI_StructInit(SPI_InitTypeDef *init);
void SPI_Init(SPI_TypeDef *spi, SPI_InitTypeDef *init);
void SPI_SSOutputCmd(SPI_TypeDef *spi, FunctionalState state);
void SPI_Cmd(SPI_TypeDef *spi, FunctionalState state);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef *spi, uint16_t nss);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef *spi, uint16_t flag);
void SPI_I2S_SendData(SPI_TypeDef *spi, uint16_t data);

/* ---- USART ---- */
typedef struct {
  __IO uint16_t SR, _r0, DR, _r1, BRR, _r2, CR1, _r3, CR2, _r4, CR3, _r5, GTPR, _r6;
} USART_TypeDef;
extern USART_TypeDef fsim_usart[2];
#define USART1 (&fsim_usart[0])
#define USART2 (&fsim_usart[1])
#define USART_SR_RXNE ((uint16_t)0x0020)
#define USART_FLAG_TXE ((uint16_t)0x0080)
#define USART_FLAG_TC ((uint16_t)0x0040)
#define USART_FLAG_RXNE ((uint16_t)0x0020)
#define USART_WordLength_8b ((uint16_t)0x0000)
#define USART_StopBits_1 ((uint16_t)0x0000)
#define USART_Parity_No ((uint16_t)0x0000)
#define USART_HardwareFlowControl_None ((uint16_t)0x0000)
#define USART_Mode_Rx ((uint16_t)0x0004)
#define USART_Mode_Tx ((uint16_t)0x0008)
#define USART_Clock_Disable ((uint16_t)0x0000)
#define USART_CPOL_High ((uint16_t)0x0400)
#define USART_CPHA_1Edge ((uint16_t)0x0000)
#define USART_LastBit_Disable ((uint16_t)0x0000)
typedef struct {
  uint32_t USART_BaudRate;
  uint16_t USART_WordLength, USART_StopBits, USART_Parity, USART_Mode, USART_HardwareFlowControl;
} USART_InitTypeDef;
typedef struct {
  uint16_t USART_Clock, USART_CPOL, USART_CPHA, USART_LastBit;
} USART_ClockInitTypeDef;
void USART_OverSampling8Cmd(USART_TypeDef *usart, FunctionalState state);
void USART_Init(USART_TypeDef *usart, USART_InitTypeDef *init);
void USART_ClockInit(USART_TypeDef *usart, USART_ClockInitTypeDef *init);
void USART_Cmd(USART_TypeDef *usart, FunctionalState state);
void USART_SendData(USART_TypeDef *usart, uint16_t data);
uint16_t USART_ReceiveData(USART_TypeDef *usart);
FlagStatus USART_GetFlagStatus(USART_TypeDef *usart, uint16_t flag);

#endif
