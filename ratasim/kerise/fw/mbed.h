// KERISE v1's STM32F405 under mbed OS 5.1, simulated: what its firmware uses of mbed (DigitalIn/Out, BusOut,
// AnalogIn, PwmOut, SPI, Ticker, Timer), of its RTOS (Thread, its signals and waits, Queue) and of ST's HAL (GPIO,
// TIM as PWM and encoder, ADC), declared as their headers do; kchip.cpp implements them on the simulator's clock and
// its peripherals as host memory. Not mbed's code: only its interface, as the firmware calls it.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <queue>
#include <string>
#include <vector>

// ---- Pins (PinNames.h: port << 4 | pin) and modes ----------------------------------------------------------------
enum PinName {
#define KPORT(P, n)                                                                                                   \
    P##_0 = (n << 4), P##_1, P##_2, P##_3, P##_4, P##_5, P##_6, P##_7, P##_8, P##_9, P##_10, P##_11, P##_12, P##_13, \
    P##_14, P##_15
    KPORT(PA, 0), KPORT(PB, 1), KPORT(PC, 2),
#undef KPORT
    NC = -1
};
enum PinMode { PullNone = 0, PullUp = 1, PullDown = 2, OpenDrain = 3, PullDefault = PullNone };

// ---- ST's HAL: the registers and handles its firmware touches ---------------------------------------------------
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#define ENABLE 1
#define DISABLE 0

struct GPIO_TypeDef { int port; };
struct TIM_TypeDef {
    int n;
    volatile uint32_t CNT, ARR, CCR1, CCR2, CCR3, CCR4;
};
struct ADC_TypeDef { int n; };
namespace ksim {
extern GPIO_TypeDef gpio[3];
extern TIM_TypeDef tim[15];
extern ADC_TypeDef adc[4];
}  // namespace ksim
#define GPIOA (&ksim::gpio[0])
#define GPIOB (&ksim::gpio[1])
#define GPIOC (&ksim::gpio[2])
#define TIM1 (&ksim::tim[1])
#define TIM2 (&ksim::tim[2])
#define TIM3 (&ksim::tim[3])
#define TIM4 (&ksim::tim[4])
#define ADC1 (&ksim::adc[1])
#define ADC2 (&ksim::adc[2])
#define ADC3 (&ksim::adc[3])

#define GPIO_PIN_0 0x0001u
#define GPIO_PIN_1 0x0002u
#define GPIO_PIN_2 0x0004u
#define GPIO_PIN_3 0x0008u
#define GPIO_PIN_4 0x0010u
#define GPIO_PIN_5 0x0020u
#define GPIO_PIN_6 0x0040u
#define GPIO_PIN_7 0x0080u
#define GPIO_PIN_8 0x0100u
#define GPIO_PIN_9 0x0200u
#define GPIO_PIN_10 0x0400u
#define GPIO_PIN_11 0x0800u
#define GPIO_MODE_ANALOG 3u
#define GPIO_MODE_AF_PP 2u
#define GPIO_NOPULL 0u
#define GPIO_PULLUP 1u
#define GPIO_PULLDOWN 2u
#define GPIO_SPEED_HIGH 2u
#define GPIO_AF1_TIM1 1u
#define GPIO_AF1_TIM2 1u
#define GPIO_AF2_TIM3 2u
#define GPIO_AF2_TIM4 2u
struct GPIO_InitTypeDef { uint32_t Pin, Mode, Pull, Speed, Alternate; };
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);

#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_ADC2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_ADC3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_ADC_FORCE_RESET() ((void)0)
#define __HAL_RCC_ADC_RELEASE_RESET() ((void)0)
#define __GPIOA_CLK_ENABLE() ((void)0)
#define __GPIOB_CLK_ENABLE() ((void)0)
#define __TIM1_CLK_ENABLE() ((void)0)
#define __TIM2_CLK_ENABLE() ((void)0)
#define __TIM3_CLK_ENABLE() ((void)0)
#define __TIM4_CLK_ENABLE() ((void)0)

// TIM
#define TIM_COUNTERMODE_UP 0u
#define TIM_CLOCKDIVISION_DIV1 0u
#define TIM_CHANNEL_1 0x0u
#define TIM_CHANNEL_2 0x4u
#define TIM_CHANNEL_3 0x8u
#define TIM_CHANNEL_4 0xCu
#define TIM_OCMODE_PWM1 0x60u
#define TIM_OCPOLARITY_HIGH 0u
#define TIM_OCNPOLARITY_HIGH 0u
#define TIM_OCFAST_DISABLE 0u
#define TIM_OCIDLESTATE_RESET 0u
#define TIM_OCNIDLESTATE_RESET 0u
#define TIM_ENCODERMODE_TI12 3u
#define TIM_INPUTCHANNELPOLARITY_RISING 0u
#define TIM_INPUTCHANNELPOLARITY_FALLING 2u
#define TIM_ICPSC_DIV4 8u
#define TIM_ICSELECTION_DIRECTTI 1u
struct TIM_Base_InitTypeDef { uint32_t Prescaler, CounterMode, Period, ClockDivision, RepetitionCounter; };
struct TIM_HandleTypeDef {
    TIM_TypeDef *Instance;
    TIM_Base_InitTypeDef Init;
};
struct TIM_OC_InitTypeDef { uint32_t OCMode, Pulse, OCPolarity, OCNPolarity, OCFastMode, OCIdleState, OCNIdleState; };
struct TIM_Encoder_InitTypeDef {
    uint32_t EncoderMode, IC1Polarity, IC1Selection, IC1Prescaler, IC1Filter, IC2Polarity, IC2Selection, IC2Prescaler,
        IC2Filter;
};
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *h);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *h, TIM_OC_InitTypeDef *c, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *h, TIM_Encoder_InitTypeDef *c);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h, uint32_t channel);
uint32_t ksim_tim_counter(TIM_TypeDef *t);
#define __HAL_TIM_GET_COUNTER(h) ksim_tim_counter((h)->Instance)

// ADC
#define ADC_CLOCKPRESCALER_PCLK_DIV4 1u
#define ADC_RESOLUTION12b 0u
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0u
#define ADC_EXTERNALTRIGCONV_T1_CC1 0u
#define ADC_DATAALIGN_RIGHT 0u
#define ADC_SAMPLETIME_3CYCLES 0u
#define ADC_CHANNEL_10 10u
#define ADC_CHANNEL_11 11u
#define ADC_CHANNEL_12 12u
#define ADC_CHANNEL_13 13u
struct ADC_InitTypeDef {
    uint32_t ClockPrescaler, Resolution, ScanConvMode, ContinuousConvMode, DiscontinuousConvMode, NbrOfDiscConversion,
        ExternalTrigConvEdge, ExternalTrigConv, DataAlign, NbrOfConversion, DMAContinuousRequests, EOCSelection;
};
struct ADC_HandleTypeDef {
    ADC_TypeDef *Instance;
    ADC_InitTypeDef Init;
    uint32_t channel, value;            // the simulator's: its channel, its last conversion
};
struct ADC_ChannelConfTypeDef { uint32_t Channel, Rank, SamplingTime, Offset; };
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c);
HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef *h);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h);

// ---- CMSIS-RTOS (RTX) --------------------------------------------------------------------------------------------
typedef enum {
    osPriorityIdle = -3, osPriorityLow = -2, osPriorityBelowNormal = -1, osPriorityNormal = 0,
    osPriorityAboveNormal = 1, osPriorityHigh = 2, osPriorityRealtime = 3, osPriorityError = 0x84
} osPriority;
typedef enum {
    osOK = 0, osEventSignal = 0x08, osEventMessage = 0x10, osEventMail = 0x20, osEventTimeout = 0x40,
    osErrorParameter = 0x80, osErrorResource = 0x81
} osStatus;
#define osWaitForever 0xFFFFFFFFu
struct osEvent {
    osStatus status;
    union { uint32_t v; void *p; int32_t signals; } value;
};

namespace ksim {
struct Task;
}

namespace rtos {

class Thread {
public:
    Thread(osPriority priority = osPriorityNormal, uint32_t stack_size = 2048, unsigned char *stack_pointer = NULL);
    ~Thread();
    osStatus start(std::function<void()> task);
    osStatus start(void (*task)()) { return start(std::function<void()>(task)); }
    template <typename T, typename M> osStatus start(T *obj, M method) {
        return start(std::function<void()>([obj, method]() { (obj->*method)(); }));
    }
    osStatus terminate();
    int32_t signal_set(int32_t signals);
    static osEvent signal_wait(int32_t signals, uint32_t millisec = osWaitForever);
    static osStatus wait(uint32_t millisec);
    static osStatus yield();
    uint32_t gettid() const { return (uint32_t)(uintptr_t)_task; }  // its id, 32 bits as on its chip (an older API)

private:
    ksim::Task *_task;
};

// Queue<T, N>: pointers (or values cast to them) in a FIFO; get() waits for one.
osEvent ksim_queue_get(void *q, uint32_t millisec);
void ksim_queue_put(void *q);
template <typename T, uint32_t N> class Queue {
public:
    osStatus put(T *data, uint32_t millisec = 0) {
        if (items.size() >= N) return osErrorResource;
        items.push((void *)data);
        ksim_queue_put(this);
        return osOK;
    }
    osEvent get(uint32_t millisec = osWaitForever) { return ksim_queue_get(this, millisec); }
    std::queue<void *> items;
};

}  // namespace rtos

// ---- mbed's drivers ---------------------------------------------------------------------------------------------
namespace mbed {

class DigitalOut {
public:
    DigitalOut(PinName pin, int value = 0);
    void write(int value);
    int read();
    DigitalOut &operator=(int value) { write(value); return *this; }
    operator int() { return read(); }
private:
    PinName _pin;
};

class DigitalIn {
public:
    DigitalIn(PinName pin, PinMode mode = PullDefault);
    int read();
    operator int() { return read(); }
private:
    PinName _pin;
};

class BusOut {
public:
    BusOut(PinName p0, PinName p1 = NC, PinName p2 = NC, PinName p3 = NC, PinName p4 = NC, PinName p5 = NC,
           PinName p6 = NC, PinName p7 = NC, PinName p8 = NC, PinName p9 = NC, PinName p10 = NC, PinName p11 = NC,
           PinName p12 = NC, PinName p13 = NC, PinName p14 = NC, PinName p15 = NC);
    ~BusOut();
    void write(int value);
    int read();
    BusOut &operator=(int value) { write(value); return *this; }
    DigitalOut &operator[](int index) { return *_pins[index]; }
    operator int() { return read(); }
private:
    DigitalOut *_pins[16];
};

class AnalogIn {
public:
    AnalogIn(PinName pin);
    float read();
    unsigned short read_u16() { return (unsigned short)(read() * 65535); }
    operator float() { return read(); }
private:
    PinName _pin;
};

class PwmOut {
public:
    PwmOut(PinName pin);
    void period(float seconds) { period_us((int)(seconds * 1e6f)); }
    void period_ms(int ms) { period_us(ms * 1000); }
    void period_us(int us);
    void pulsewidth(float seconds) { pulsewidth_us((int)(seconds * 1e6f)); }
    void pulsewidth_ms(int ms) { pulsewidth_us(ms * 1000); }
    void pulsewidth_us(int us);
    void write(float duty);
    float read();
    PwmOut &operator=(float duty) { write(duty); return *this; }
private:
    PinName _pin;
};

class SPI {
public:
    SPI(PinName mosi, PinName miso, PinName sclk, PinName ssel = NC);
    void format(int bits, int mode = 0) {}
    void frequency(int hz = 1000000) {}
    int write(int value);
};

class Timer {
public:
    Timer();
    void start();
    void stop();
    void reset();
    float read();
    int read_ms();
    int read_us();
    operator float() { return read(); }
private:
    uint64_t _start_ns, _total_ns;
    bool _running;
};

class Ticker {
public:
    Ticker();
    ~Ticker();
    void attach_us(std::function<void()> fn, uint32_t us);
    template <typename T, typename M> void attach_us(T *obj, M method, uint32_t us) {
        attach_us(std::function<void()>([obj, method]() { (obj->*method)(); }), us);
    }
    void attach(std::function<void()> fn, float seconds) { attach_us(fn, (uint32_t)(seconds * 1e6f)); }
    template <typename T, typename M> void attach(T *obj, M method, float seconds) {
        attach_us(obj, method, (uint32_t)(seconds * 1e6f));
    }
    void detach();
private:
    int _id;
};

void wait(float seconds);
void wait_ms(int ms);
void wait_us(int us);

}  // namespace mbed

using namespace mbed;
using namespace rtos;
using namespace std;
