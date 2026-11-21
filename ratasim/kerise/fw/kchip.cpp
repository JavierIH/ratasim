// KERISE v2's STM32F405 under mbed OS 5.1 (its RTX kernel), simulated: what mbed.h declares. Python
// (kerise/robot.py) drives it with the ksim_* functions at the end.
//
// Its RTOS: each of its threads a coroutine (ucontext) with its priority. The highest ready thread runs until it
// waits (Thread::wait, signal_wait, Queue::get) or wakes one of higher priority (it is preempted, as RTX does);
// its computation takes no time. When none is ready, time jumps to the next event: a Ticker's interrupt (its
// callback runs then, in no thread, as on the chip) or a thread's wait ending. Time is in nanoseconds.
//
// Its peripherals: GPIO pins as host memory; TIM2's four compare registers its motors' PWM (DRV8835, IN/IN);
// TIM3 and TIM4 its encoders (IEH2-1024 on each motor, x4); ADC2 and ADC3 its IR receivers (SFH213FA behind an
// amplifier biased at 2048: lit by its emitter, the reading falls by the light the wall sends back), their
// emitters two PwmOut (PB14: SL and FR, PB15: SR and FL); AnalogIn PB1 its battery (20k/10k divider); SPI1 its
// MPU6500 (its registers 0x3B-0x48); its console (printf) buffered for the simulator.
#include <ucontext.h>

#include <cstdarg>

#include "mbed.h"

int kerise_main();      // its main() (main.cpp, built with main renamed)

#define STACK_BYTES (512 * 1024)
#define MAX_SWITCHES_AT_ONCE 2000000    // a thread that never waits: the chip would hang (a fault for the simulator)
enum { KSIM_RUNNING = 0, KSIM_FAULT = 3 };

namespace ksim {

GPIO_TypeDef gpio[3] = {{0}, {1}, {2}};
TIM_TypeDef tim[15];
ADC_TypeDef adc[4] = {{0}, {1}, {2}, {3}};

uint64_t now_ns;
static uint64_t until_ns;
static uint8_t state;
static ucontext_t sched_ctx;
static uint64_t ready_seq;

// ---- Threads -------------------------------------------------------------------------------------------------------
struct Task {
    ucontext_t ctx;
    char *stack = nullptr;
    int prio = 0;
    std::function<void()> fn;
    enum { DEAD, READY, BLOCKED } st = DEAD;
    uint64_t wake_ns = UINT64_MAX;      // its wait's end (blocked)
    int32_t signals = 0, wait_mask = 0; // wait_mask: waiting for these signals
    void *queue = nullptr;              // waiting for a message in this queue
    bool timed_out = false;
    uint64_t seq = 0;                   // when it became ready (the oldest first among equals)
};
static std::vector<Task *> tasks;
static Task *current;

static void ready(Task *t, bool first = false) {
    t->st = Task::READY;
    t->wake_ns = UINT64_MAX;
    t->wait_mask = 0;
    t->queue = nullptr;
    t->seq = first ? 0 : ++ready_seq;
}

static void to_scheduler() { swapcontext(&current->ctx, &sched_ctx); }

// A thread made ready by the running one: if higher, it runs now (the running one ready, first among its equals).
static void preempt_for(Task *t) {
    if (current && t->prio > current->prio) {
        ready(current, true);
        to_scheduler();
    }
}

static void block(uint64_t wake_ns) {
    current->st = Task::BLOCKED;
    current->wake_ns = wake_ns;
    current->timed_out = false;
    to_scheduler();
}

static void entry() {
    Task *t = current;
    t->fn();
    t->st = Task::DEAD;     // a thread whose function returns ends
    to_scheduler();
}

static Task *pick() {
    Task *best = nullptr;
    for (Task *t : tasks)
        if (t->st == Task::READY && (!best || t->prio > best->prio || (t->prio == best->prio && t->seq < best->seq)))
            best = t;
    return best;
}

// ---- Tickers (mbed's us_ticker queue) ----------------------------------------------------------------------------
struct Tick {
    bool on = false;
    uint64_t period_ns = 0, next_ns = 0;
    std::function<void()> fn;
};
static std::vector<Tick> tickers;

static uint64_t next_event() {
    uint64_t t = UINT64_MAX;
    for (auto &k : tickers)
        if (k.on && k.next_ns < t) t = k.next_ns;
    for (Task *k : tasks)
        if (k->st == Task::BLOCKED && k->wake_ns < t) t = k->wake_ns;
    return t;
}

static void fire_due() {
    for (;;) {
        Tick *due = nullptr;
        for (auto &k : tickers)
            if (k.on && k.next_ns <= now_ns && (!due || k.next_ns < due->next_ns)) due = &k;
        if (!due) break;
        due->next_ns += due->period_ns;
        std::function<void()> fn = due->fn;     // its callback may detach or re-attach it
        fn();
    }
    for (Task *k : tasks)
        if (k->st == Task::BLOCKED && k->wake_ns <= now_ns) {
            bool waiting = k->wait_mask || k->queue;
            ready(k);
            k->timed_out = waiting;
        }
}

// ---- The world, from the simulator --------------------------------------------------------------------------------
struct Wheel { double counts, rate; uint64_t t0; };     // its motor's encoder counts at t0, counts a nanosecond
static Wheel wheel[2];
static double enc_zero[2];
static uint32_t enc_base[2];
static double ir[4];            // light back at each receiver, ADC counts: SL, FL, FR, SR
static double gyro_dps[3], accel_g[3] = {0, 0, 1};
static double battery_v = 3.9;
static bool button;
static uint8_t pins[48];
static bool tim_running[15][4];
static uint32_t noise = 12345;

static double counts(int side) { return wheel[side].counts + wheel[side].rate * (double)(now_ns - wheel[side].t0); }

static int noise_counts() {     // the ADC's own: -1..1
    noise = noise * 1103515245u + 12345u;
    return (int)((noise >> 16) % 3) - 1;
}

// PwmOut: period and high time per pin (us), its output high from the start of each period.
struct Pwm { uint32_t period_us = 20000, pulse_us = 0; };
static Pwm pwm[48];
static bool pwm_high(int pin) {
    const Pwm &p = pwm[pin];
    if (!p.pulse_us || !p.period_us) return false;
    return (now_ns / 1000) % p.period_us < p.pulse_us;
}

// MPU6500 on SPI1: a transaction from its chip select going low; its first byte the register (bit 7: read).
static int spi_byte = -1, spi_reg;
static bool spi_read;
static uint8_t mpu_regs[128];

static void mpu_snapshot() {
    auto put = [](int reg, double v) {
        long r = lround(v);
        r = r > 32767 ? 32767 : r < -32768 ? -32768 : r;
        mpu_regs[reg] = (uint8_t)((uint16_t)r >> 8);
        mpu_regs[reg + 1] = (uint8_t)r;
    };
    const double accel_lsb = 2048.0, gyro_lsb = 16.384;        // +-16 g, +-2000 dps (its setup)
    for (int i = 0; i < 3; i++) {
        put(0x3B + 2 * i, accel_g[i] * accel_lsb);
        put(0x43 + 2 * i, gyro_dps[i] * gyro_lsb);
    }
    put(0x41, 25.0 * 333.87 - 21 * 333.87);                    // 25 C
}

// ---- Its console --------------------------------------------------------------------------------------------------
static char tx[1 << 18];
static uint32_t tx_len;
static void console_out(const char *s, size_t n) {
    for (size_t i = 0; i < n && tx_len < sizeof(tx); i++)
        if (s[i] != '\r') tx[tx_len++] = s[i];
}

}  // namespace ksim

using namespace ksim;

extern "C" {
int printf(const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) console_out(buf, n < (int)sizeof(buf) ? n : sizeof(buf) - 1);
    return n;
}
int puts(const char *s) {
    console_out(s, strlen(s));
    console_out("\n", 1);
    return 1;
}
int putchar(int c) {
    char ch = (char)c;
    console_out(&ch, 1);
    return c;
}
int getchar(void) { return EOF; }       // nothing typed on its serial port
}

// ---- RTOS ---------------------------------------------------------------------------------------------------------
namespace rtos {

Thread::Thread(osPriority priority, uint32_t, unsigned char *) {
    _task = new Task;
    _task->prio = priority;
    tasks.push_back(_task);
}

Thread::~Thread() { terminate(); }

osStatus Thread::start(std::function<void()> fn) {
    if (_task->st != Task::DEAD) return osErrorParameter;
    if (!_task->stack) _task->stack = (char *)malloc(STACK_BYTES);
    _task->fn = fn;
    _task->signals = 0;
    getcontext(&_task->ctx);
    _task->ctx.uc_stack.ss_sp = _task->stack;
    _task->ctx.uc_stack.ss_size = STACK_BYTES;
    _task->ctx.uc_link = nullptr;
    makecontext(&_task->ctx, entry, 0);
    ready(_task);
    preempt_for(_task);
    return osOK;
}

osStatus Thread::terminate() {
    if (_task->st == Task::DEAD) return osErrorResource;
    _task->st = Task::DEAD;
    _task->wait_mask = 0;
    _task->queue = nullptr;
    if (_task == current) to_scheduler();       // never back
    return osOK;
}

int32_t Thread::signal_set(int32_t signals) {
    if (_task->st == Task::DEAD) return (int32_t)0x80000000;
    int32_t before = _task->signals;
    _task->signals |= signals;
    if (_task->st == Task::BLOCKED && _task->wait_mask && (_task->signals & _task->wait_mask) == _task->wait_mask) {
        ready(_task);
        preempt_for(_task);
    }
    return before;
}

osEvent Thread::signal_wait(int32_t signals, uint32_t millisec) {
    osEvent e;
    Task *t = current;
    if ((t->signals & signals) != signals) {
        if (millisec == 0) {
            e.status = osOK;
            return e;
        }
        t->wait_mask = signals;
        block(millisec == osWaitForever ? UINT64_MAX : now_ns + (uint64_t)millisec * 1000000);
        if (t->timed_out) {
            e.status = osEventTimeout;
            return e;
        }
    }
    e.status = osEventSignal;
    e.value.signals = t->signals;
    t->signals &= ~signals;
    return e;
}

osStatus Thread::wait(uint32_t millisec) {
    if (!millisec) return yield();
    block(now_ns + (uint64_t)millisec * 1000000);
    return osEventTimeout;
}

osStatus Thread::yield() {
    ready(current);
    to_scheduler();
    return osOK;
}

void ksim_queue_put(void *q) {
    Task *best = nullptr;
    for (Task *t : tasks)
        if (t->st == Task::BLOCKED && t->queue == q && (!best || t->prio > best->prio)) best = t;
    if (best) {
        ready(best);
        preempt_for(best);
    }
}

osEvent ksim_queue_get(void *qp, uint32_t millisec) {
    auto *q = (std::queue<void *> *)&((Queue<int, 1> *)qp)->items;     // every Queue<T, N> keeps the same member
    osEvent e;
    while (q->empty()) {
        if (millisec == 0) {
            e.status = osOK;
            return e;
        }
        current->queue = qp;
        block(millisec == osWaitForever ? UINT64_MAX : now_ns + (uint64_t)millisec * 1000000);
        if (current->timed_out) {
            e.status = osEventTimeout;
            return e;
        }
    }
    e.status = osEventMessage;
    e.value.p = q->front();
    q->pop();
    return e;
}

}  // namespace rtos

// ---- mbed's drivers -----------------------------------------------------------------------------------------------
namespace mbed {

DigitalOut::DigitalOut(PinName pin, int value) : _pin(pin) { write(value); }
void DigitalOut::write(int value) {
    if (_pin == NC) return;
    pins[_pin] = value ? 1 : 0;
    if (_pin == PA_4) spi_byte = -1;        // the MPU6500's chip select: a transaction starts or ends
}
int DigitalOut::read() { return _pin == NC ? 0 : pins[_pin]; }

DigitalIn::DigitalIn(PinName pin, PinMode) : _pin(pin) {}
int DigitalIn::read() {
    if (_pin == PB_0) return button ? 0 : 1;    // its button, to ground, pulled up
    return pins[_pin];
}

BusOut::BusOut(PinName p0, PinName p1, PinName p2, PinName p3, PinName p4, PinName p5, PinName p6, PinName p7,
               PinName p8, PinName p9, PinName p10, PinName p11, PinName p12, PinName p13, PinName p14, PinName p15) {
    PinName p[16] = {p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15};
    for (int i = 0; i < 16; i++) _pins[i] = p[i] == NC ? nullptr : new DigitalOut(p[i]);
}
BusOut::~BusOut() {
    for (auto *d : _pins) delete d;
}
void BusOut::write(int value) {
    for (int i = 0; i < 16; i++)
        if (_pins[i]) _pins[i]->write((value >> i) & 1);
}
int BusOut::read() {
    int v = 0;
    for (int i = 0; i < 16; i++)
        if (_pins[i] && _pins[i]->read()) v |= 1 << i;
    return v;
}

AnalogIn::AnalogIn(PinName pin) : _pin(pin) {}
float AnalogIn::read() {
    if (_pin != PB_1) return 0.0f;
    double v = battery_v * 10.0 / (10.0 + 20.0) / 3.3;
    return (float)(v > 1.0 ? 1.0 : v);
}

PwmOut::PwmOut(PinName pin) : _pin(pin) {}
void PwmOut::period_us(int us) { pwm[_pin].period_us = us > 0 ? us : 1; }
void PwmOut::pulsewidth_us(int us) { pwm[_pin].pulse_us = us > 0 ? us : 0; }
void PwmOut::write(float duty) { pwm[_pin].pulse_us = (uint32_t)(duty * pwm[_pin].period_us); }
float PwmOut::read() { return (float)pwm[_pin].pulse_us / pwm[_pin].period_us; }

SPI::SPI(PinName, PinName, PinName, PinName) {}
int SPI::write(int value) {
    if (pins[PA_4]) return 0xFF;            // not selected
    if (spi_byte < 0) {
        spi_reg = value & 0x7F;
        spi_read = value & 0x80;
        spi_byte = 0;
        if (spi_read) mpu_snapshot();
        return 0;
    }
    spi_byte++;
    int r = spi_reg & 0x7F;
    spi_reg++;
    if (!spi_read) {
        mpu_regs[r] = (uint8_t)value;
        return 0;
    }
    return mpu_regs[r];
}

Timer::Timer() : _start_ns(0), _total_ns(0), _running(false) {}
void Timer::start() {
    if (!_running) _start_ns = now_ns, _running = true;
}
void Timer::stop() {
    if (_running) _total_ns += now_ns - _start_ns, _running = false;
}
void Timer::reset() {
    _total_ns = 0;
    _start_ns = now_ns;
}
static uint64_t elapsed(uint64_t total, uint64_t start, bool running) {
    return total + (running ? now_ns - start : 0);
}
float Timer::read() { return elapsed(_total_ns, _start_ns, _running) * 1e-9f; }
int Timer::read_ms() { return (int)(elapsed(_total_ns, _start_ns, _running) / 1000000); }
int Timer::read_us() { return (int)(elapsed(_total_ns, _start_ns, _running) / 1000); }

Ticker::Ticker() {
    _id = (int)tickers.size();
    tickers.emplace_back();
}
Ticker::~Ticker() { detach(); }
void Ticker::attach_us(std::function<void()> fn, uint32_t us) {
    Tick &k = tickers[_id];
    k.fn = fn;
    k.period_ns = (uint64_t)(us ? us : 1) * 1000;
    k.next_ns = now_ns + k.period_ns;
    k.on = true;
}
void Ticker::detach() { tickers[_id].on = false; }

void wait(float seconds) { Thread::wait((uint32_t)(seconds * 1000)); }
void wait_ms(int ms) { Thread::wait(ms); }
void wait_us(int us) { Thread::wait((us + 999) / 1000); }

}  // namespace mbed

// ---- ST's HAL -----------------------------------------------------------------------------------------------------
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *) {}

HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *h) {
    h->Instance->ARR = h->Init.Period;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *h, TIM_OC_InitTypeDef *c, uint32_t channel) {
    volatile uint32_t *ccr[] = {&h->Instance->CCR1, &h->Instance->CCR2, &h->Instance->CCR3, &h->Instance->CCR4};
    *ccr[channel / 4] = c->Pulse;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel) {
    tim_running[h->Instance - tim][channel / 4] = true;
    return HAL_OK;
}

static int encoder_side(TIM_TypeDef *t) { return t == TIM3 ? 0 : t == TIM4 ? 1 : -1; }
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *h, TIM_Encoder_InitTypeDef *) {
    int s = encoder_side(h->Instance);
    if (s < 0) return HAL_ERROR;
    h->Instance->ARR = h->Init.Period;
    enc_zero[s] = counts(s);
    enc_base[s] = h->Instance->CNT;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t) { return HAL_OK; }

// Its encoders count up with their motor turning forward on the right (TIM4), backward on the left (TIM3): its
// firmware negates the left one.
uint32_t ksim_tim_counter(TIM_TypeDef *t) {
    int s = encoder_side(t);
    if (s < 0) return t->CNT;
    double c = counts(s) - enc_zero[s];
    long n = lround(s == 0 ? -c : c);
    t->CNT = (uint16_t)(enc_base[s] + (uint32_t)n);
    return t->CNT;
}

HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h) {
    h->channel = 0;
    h->value = 2048;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c) {
    h->channel = c->Channel;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef *h) {
    // channels 10-13: SL, FL, FR, SR; their emitters PB14 (SL, FR) and PB15 (SR, FL)
    static const int emitter[4] = {PB_14, PB_15, PB_14, PB_15};
    int v = 2048;
    if (h->channel >= 10 && h->channel <= 13) {
        int i = h->channel - 10;
        if (pwm_high(emitter[i])) v -= (int)lround(ir[i]);
    }
    v += noise_counts();
    h->value = (uint32_t)(v < 0 ? 0 : v > 4095 ? 4095 : v);
    return HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h) { return h->value; }

// ---- Simulator API ------------------------------------------------------------------------------------------------
static Thread *main_thread;

extern "C" {
int ksim_boot(void) {
    main_thread = new Thread(osPriorityNormal);
    main_thread->start(std::function<void()>([]() { kerise_main(); }));
    return 1;
}

// Runs the firmware until this time (ns since it was switched on).
uint8_t ksim_run_until(uint64_t t_ns) {
    if (state != KSIM_RUNNING) return state;
    until_ns = t_ns;
    uint64_t at = now_ns;
    long switches = 0;
    for (;;) {
        Task *t = pick();
        if (t) {
            if (now_ns != at) at = now_ns, switches = 0;
            if (++switches > MAX_SWITCHES_AT_ONCE) {
                fprintf(stderr, "kerise: its threads run without waiting at %.3f s\n", now_ns * 1e-9);
                return state = KSIM_FAULT;
            }
            current = t;
            swapcontext(&sched_ctx, &t->ctx);
            current = nullptr;
            continue;
        }
        uint64_t next = next_event();
        if (next > until_ns) {
            now_ns = until_ns;
            return state;
        }
        now_ns = next;
        fire_due();
    }
}
uint64_t ksim_time_ns(void) { return now_ns; }

void ksim_set_encoder(int side, double counts_now, double counts_per_ns) {
    if (side >= 0 && side < 2) wheel[side] = Wheel{counts_now, counts_per_ns, now_ns};
}
void ksim_set_ir(double sl, double fl, double fr, double sr) { ir[0] = sl, ir[1] = fl, ir[2] = fr, ir[3] = sr; }
void ksim_set_imu(double gx, double gy, double gz, double ax, double ay, double az) {
    gyro_dps[0] = gx, gyro_dps[1] = gy, gyro_dps[2] = gz;
    accel_g[0] = ax, accel_g[1] = ay, accel_g[2] = az;
}
void ksim_set_battery(double volts) { battery_v = volts; }
void ksim_button(int pressed) { button = pressed != 0; }

// Its motors' drive (-1..1, + forward) from TIM2's compare registers: IN1 high for CCR1 of its 1000 counts, IN2
// for CCR2 (the left; the right CH3, CH4). Both low: coasting (its firmware's free()), then nan.
double ksim_motor(int side) {
    TIM_TypeDef *t = TIM2;
    double period = t->ARR + 1.0;
    uint32_t a = side == 0 ? t->CCR2 : t->CCR3, b = side == 0 ? t->CCR1 : t->CCR4;
    if (!tim_running[2][side == 0 ? 0 : 2] || (a == 0 && b == 0)) return NAN;
    return ((double)a - (double)b) / period;
}
int ksim_leds(void) {
    int v = 0;
    for (int i = 0; i < 8; i++) v |= pins[PC_5 + i] << i;
    return v;
}
int ksim_buzzer_hz(void) {
    const Pwm &p = pwm[PB_9];
    return p.pulse_us ? (int)(1000000 / p.period_us) : 0;
}
int ksim_emitters(void) { return (pwm[PB_14].pulse_us ? 1 : 0) | (pwm[PB_15].pulse_us ? 2 : 0); }

uint32_t ksim_console(char *buffer, uint32_t size) {
    const uint32_t n = tx_len < size ? tx_len : size;
    memcpy(buffer, tx, n);
    memmove(tx, tx + n, tx_len - n);
    tx_len -= n;
    return n;
}
}
