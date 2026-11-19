// Fujin's STM32G474, simulated: its firmware's main loop in a coroutine, its clock and SysTick, its console (printf,
// over USB on the robot). Its peripherals are behind its BSP (fbsp.cpp). Python (fujin/robot.py) drives it with the
// fsim_* functions at the end.
//
// Time: simulated, in nanoseconds. It moves when the firmware waits (delay_ms, delay_us), when it reads the clock and
// on each pass of its main loop (below); the rest of its computation takes none. SysTick fires every 500 us, as the
// firmware sets it (HAL_InitTick: 2 kHz): HAL's millisecond tick every other one, and the firmware's soft timer (its
// control loop's Timeout events, utils/soft_timer). When the time passes what the simulator asked for, the firmware
// yields to it (in the middle of whatever it was doing, as an interrupt would find it).
#include <ucontext.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "bsp/core.hpp"
#include "fsim.h"
#include "fsm/fsm.hpp"
#include "utils/soft_timer.hpp"

void startup();     // the firmware's (src/main.cpp, built with its main() renamed: its loop is the one below)

// ---- What the chip's calls cost (ns): 170 MHz -------------------------------------------------------------------
#define CLOCK_READ_NS 100       // the microsecond timer (TIM5), HAL's tick
#define LOOP_PASS_NS 1000       // a pass of its main loop with no event to handle (fsm.spin)
#define SYSTICK_NS 500000ull
#define STACK_BYTES (2 * 1024 * 1024)
enum { FSIM_RUNNING = 0 };

namespace fsim {

uint64_t now_ns;
static uint64_t until_ns, next_systick_ns = SYSTICK_NS;
static uint32_t systicks;
static ucontext_t sim_ctx, fw_ctx;
static char *fw_stack;
static uint8_t in_fw, state;

void advance(uint64_t ns) {
    now_ns += ns;
    while (now_ns >= next_systick_ns) {     // SysTick_Handler
        next_systick_ns += SYSTICK_NS;
        systicks++;
        soft_timer::tick();
    }
    if (in_fw && now_ns >= until_ns) swapcontext(&fw_ctx, &sim_ctx);
}

void wait_until(uint64_t t) {
    while (now_ns < t) {
        uint64_t step = t - now_ns;
        if (until_ns > now_ns && until_ns - now_ns < step) step = until_ns - now_ns;
        advance(step);
    }
}

uint32_t tick_ms() {
    advance(CLOCK_READ_NS);
    return systicks / 2;
}

// Its main(), with the time a pass of its loop takes: its FSM's event queue is empty more often than not (the
// SysTick and the buttons fill it from their interrupts).
static void fw_entry() {
    in_fw = 1;
    bsp::init();
    startup();
    fsm::FSM fsm;
    fsm.start();
    for (;;) {
        fsm.spin();
        advance(LOOP_PASS_NS);
    }
}

// ---- Console: its printf (to USB) buffered for the simulator ---------------------------------------------------
static char tx[1 << 18];
static uint32_t tx_len;

void console_out(const char *s, size_t n) {
    for (size_t i = 0; i < n && tx_len < sizeof(tx); i++)
        if (s[i] != '\r') tx[tx_len++] = s[i];
}

}  // namespace fsim

// ---- Simulator API ----------------------------------------------------------------------------------------------
extern "C" {
int fsim_boot(void) {
    fsim::fw_stack = (char *)malloc(STACK_BYTES);
    if (!fsim::fw_stack) return 0;
    getcontext(&fsim::fw_ctx);
    fsim::fw_ctx.uc_stack.ss_sp = fsim::fw_stack;
    fsim::fw_ctx.uc_stack.ss_size = STACK_BYTES;
    fsim::fw_ctx.uc_link = NULL;
    makecontext(&fsim::fw_ctx, fsim::fw_entry, 0);
    return 1;
}
// Runs the firmware until this time (ns since it was switched on).
uint8_t fsim_run_until(uint64_t t_ns) {
    if (fsim::state != FSIM_RUNNING) return fsim::state;
    fsim::until_ns = t_ns;
    if (fsim::now_ns < fsim::until_ns) swapcontext(&fsim::sim_ctx, &fsim::fw_ctx);
    return fsim::state;
}
uint64_t fsim_time_ns(void) { return fsim::now_ns; }

void fsim_set_encoder(int side, double counts, double counts_per_ns) {
    if (side >= 0 && side < 2) fsim::wheel[side] = fsim::Wheel{counts, counts_per_ns, fsim::now_ns};
}
void fsim_set_ir(double r, double fl, double fr, double l) {
    fsim::ir_mm[0] = r, fsim::ir_mm[1] = fl, fsim::ir_mm[2] = fr, fsim::ir_mm[3] = l;
}
void fsim_ir_model(double a, double b, double c) {
    fsim::ir_model[0] = a, fsim::ir_model[1] = b, fsim::ir_model[2] = c;
}
void fsim_set_imu(double gyro_z_dps, double accel_z_g) {
    fsim::gyro_z_dps = gyro_z_dps, fsim::accel_z_g = accel_z_g;
}
void fsim_set_battery(double volts) { fsim::battery_v = volts; }
void fsim_button(int button, int pressed) { fsim::button_edge(button, pressed != 0); }

int fsim_motor(int side) { return side >= 0 && side < 2 ? fsim::motor[side] : 0; }
int fsim_fan(void) { return fsim::fan; }
uint32_t fsim_stripe(int i) { return i >= 0 && i < 2 ? fsim::stripe[i] : 0; }
int fsim_indication(void) { return fsim::indication; }
int fsim_emitters(void) { return fsim::emitters; }
int fsim_buzzer(void) { return fsim::buzzer; }

uint32_t fsim_console(char *buffer, uint32_t size) {
    const uint32_t n = fsim::tx_len < size ? fsim::tx_len : size;
    memcpy(buffer, fsim::tx, n);
    memmove(fsim::tx, fsim::tx + n, fsim::tx_len - n);
    fsim::tx_len -= n;
    return n;
}
}
