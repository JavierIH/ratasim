// Fujin's board as the simulator sees it: what fchip.cpp (its STM32G474's clock, interrupts and console) and fbsp.cpp
// (its BSP, the interface of its firmware's inc/bsp/*.hpp) share, set and read by ratasim/fujin/robot.py through the
// fsim_* functions of fchip.cpp.
#pragma once

#include <cstddef>
#include <cstdint>

namespace fsim {

// ---- Time (ns since switched on) and the chip's costs -------------------------------------------------------------
extern uint64_t now_ns;
void advance(uint64_t ns);              // time passes; the SysTick interrupts that fall in it run
void wait_until(uint64_t t_ns);
uint32_t tick_ms();                     // HAL's tick: a millisecond every two SysTicks (2 kHz)

// ---- The world, from the simulator --------------------------------------------------------------------------------
struct Wheel {                          // its encoder: counts (1024 a turn of the wheel) at t0, counts a nanosecond
    double counts, rate;
    uint64_t t0;
};
extern Wheel wheel[2];                  // 0 left, 1 right
extern double ir_mm[4];                 // each sensor's reading with its emitter on (the firmware's mm): R, FL, FR, L
extern double ir_model[3];              // raw ADC counts of a reading: mm = a / ln(raw + c) - b
extern double gyro_z_dps, accel_z_g;    // the IMU's (bias and noise in)
extern double battery_v;
double wheel_counts(int side);          // the counts now

// ---- The board's outputs, for the simulator ---------------------------------------------------------------------
extern int16_t motor[2];                // -1000..1000, + forward (as the firmware asks for them)
extern uint16_t fan;                    // 0..999
extern uint32_t stripe[2];              // the two WS2812, 0xRRGGBB as sent
extern uint8_t indication, emitters, buzzer;

// ---- Its console (fconsole.cpp: printf and the like): buffered for the simulator ----------------------------------
void console_out(const char *s, size_t n);

// ---- Buttons (fbsp.cpp): an edge on button 1 or 2 (pressed: its pin low), as its EXTI interrupt sees it -----------
void button_edge(int button, bool pressed);

}  // namespace fsim
