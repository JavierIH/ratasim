#include <cstdlib>
// Fujin's BSP for the simulator: the interface its firmware is written against (its inc/bsp/*.hpp), implemented on
// the simulated board of fchip.cpp instead of its STM32G474's peripherals. Our own code: nothing of its
// src/bsp/target-* is used. Where its interface holds more than hardware (the IR sensors' wall detection, the
// encoders' speeds, the gyro's angle), this does what its header and its firmware's use of it ask for, with its
// firmware's parameters (services::Config) and the thresholds its sensing is tuned to (below).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buttons.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/core.hpp"
#include "bsp/debug.hpp"
#include "bsp/eeprom.hpp"
#include "bsp/encoders.hpp"
#include "bsp/fan.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "bsp/usb.hpp"
#include "fsim.h"
#include "services/config.hpp"

// ---- The simulated world (set by the simulator through fchip.cpp) --------------------------------------------------
namespace fsim {
Wheel wheel[2];
double ir_mm[4] = {270, 270, 270, 270};
double ir_model[3] = {4000.0, 440.0, 0.0};
double gyro_z_dps, accel_z_g = 1.0;
double battery_v = 12.0;
int16_t motor[2];
uint16_t fan;
uint32_t stripe[2];
uint8_t indication, emitters, buzzer;

double wheel_counts(int side) {
    const Wheel &w = wheel[side];
    return w.counts + w.rate * (double)(now_ns - w.t0);
}
}  // namespace fsim

// ---- Timers: HAL's millisecond tick, its 32-bit microsecond timer (TIM5) -------------------------------------------
namespace bsp {
void timers::init() {}
uint32_t get_tick_ms() { return fsim::tick_ms(); }
uint32_t get_tick_us() {
    fsim::advance(100);
    return (uint32_t)(fsim::now_ns / 1000u);
}
void delay_ms(uint32_t ms) { fsim::wait_until(fsim::now_ns + (uint64_t)ms * 1000000u); }
void delay_us(uint32_t us) { fsim::wait_until(fsim::now_ns + (uint64_t)us * 1000u); }

void init() {
    analog_sensors::init();
    ble::init();
    buttons::init();
    buzzer::init();
    encoders::init();
    fan::init();
    leds::init();
    motors::init();
    timers::init();
    usb::init();
    imu::init();
    eeprom::init();
    delay_ms(20);
}

void debug::print(const char *s) { std::printf("%s\r\n", s); }
void usb::init() {}
}  // namespace bsp

// ---- Motors and fan: the duties the firmware sets ------------------------------------------------------------------
namespace bsp::motors {
void init() { fsim::motor[0] = fsim::motor[1] = 0; }
void set(int16_t left, int16_t right) {
    fsim::motor[0] = std::clamp<int16_t>(left, -MAX_SPEED, MAX_SPEED);
    fsim::motor[1] = std::clamp<int16_t>(right, -MAX_SPEED, MAX_SPEED);
}
}  // namespace bsp::motors

namespace bsp::fan {
void init() { fsim::fan = 0; }
void set(uint16_t speed) { fsim::fan = std::min<uint16_t>(speed, 999); }
float get_max_fan_voltage() { return 10.0f; }     // its fan's rated voltage (the firmware scales its duty to it)
}  // namespace bsp::fan

// ---- LEDs, buzzer ------------------------------------------------------------------------------------------------
namespace bsp::leds {
Color Color::White = Color(0x3F, 0x3F, 0x3F);
Color Color::Black = Color(0x00, 0x00, 0x00);
Color Color::Red = Color(0x7F, 0x00, 0x00);
Color Color::Green = Color(0x00, 0x3F, 0x00);
Color Color::Blue = Color(0x00, 0x00, 0x7F);
Color Color::Purple = Color(0x7F, 0x00, 0x7F);
Color Color::Yellow = Color(0x3F, 0x3F, 0x00);
Color Color::Orange = Color(0x7F, 0x30, 0x00);
Color Color::Pink = Color(0x7F, 0x20, 0x40);
Color Color::Cyan = Color(0x00, 0x3F, 0x3F);

uint32_t Color::encode() const { return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b; }   // 0xRRGGBB here

static uint32_t pending[2];
void init() { pending[0] = pending[1] = 0; }
void indication_on() { fsim::indication = 1; }
void indication_off() { fsim::indication = 0; }
void indication_toggle() { fsim::indication ^= 1; }
// The emitters' bits: LEFT_SIDE, LEFT_FRONT, RIGHT_FRONT, RIGHT_SIDE
void ir_emitter_on(Emitter led) { fsim::emitters |= 1u << led; }
void ir_emitter_off(Emitter led) { fsim::emitters &= ~(1u << led); }
void ir_emitter_all_on() { fsim::emitters = 0x0F; }
void ir_emitter_all_off() { fsim::emitters = 0; }
void stripe_set(Color const &c1, Color const &c2) {
    pending[0] = c1.encode(), pending[1] = c2.encode();
    stripe_send();
}
void stripe_set(Color const &c) { stripe_set(c, c); }
void stripe_send() {
    fsim::stripe[0] = pending[0], fsim::stripe[1] = pending[1];
    fsim::advance(2 * 24 * 1250 + 50000);     // 48 bits at 800 kHz and the reset
}
}  // namespace bsp::leds

namespace bsp::buzzer {
void init() { fsim::buzzer = 0; }
void start() { fsim::buzzer = 1; }
void stop() { fsim::buzzer = 0; }
void set_volume(uint8_t) {}
void set_frequency(uint16_t) {}
void beep(uint32_t ms) {
    start();
    delay_ms(ms);
    stop();
}
void beep_double(uint32_t first_ms, uint32_t gap_ms, uint32_t second_ms) {
    beep(first_ms);
    delay_ms(gap_ms);
    beep(second_ms);
}
}  // namespace bsp::buzzer

// ---- Buttons: a press's length when released: short over 50 ms, long over 700 ms -----------------------------------
namespace bsp::buttons {
static ButtonCallback callback[2];
static bool enabled = true;
static uint32_t pressed_at[2];
void init() {
    enabled = true;
    pressed_at[0] = pressed_at[1] = 0;
}
void register_callback_button1(ButtonCallback cb) { callback[0] = cb; }
void register_callback_button2(ButtonCallback cb) { callback[1] = cb; }
void enable(bool on) {
    enabled = on;
    if (!on) pressed_at[0] = pressed_at[1] = 0;
}
void disable() { enable(false); }
void set_enabled(bool on) { enable(on); }
bool is_enabled() { return enabled; }
}  // namespace bsp::buttons

void fsim::button_edge(int button, bool pressed) {
    using namespace bsp::buttons;
    if (button < 1 || button > 2 || !enabled || !callback[button - 1]) return;
    const uint32_t ms = (uint32_t)(now_ns / 1000000u);
    uint32_t &t = pressed_at[button - 1];
    if (pressed) {
        t = ms ? ms : 1;
        return;
    }
    if (!t) return;
    const uint32_t held = ms - t;
    t = 0;
    if (held > 700) callback[button - 1](PressType::LONG);
    else if (held > 50) callback[button - 1](PressType::SHORT);
}

// ---- BLE (its JDY-10): what it sends is dropped; nothing comes in ------------------------------------------------
namespace bsp::ble {
static BleCallback callback;
static bool locked = true;
void init() {}
void start() {}
void stop() {}
void transmit(uint8_t *, uint8_t) { fsim::advance(1000); }
void register_callback(BleCallback cb) { callback = cb; }
void lock_config_rcv() { locked = true; }
void unlock_config_rcv() { locked = false; }
bool is_config_locked() { return locked; }
}  // namespace bsp::ble

// ---- EEPROM (its 24LC512): 64 KB, erased (0xFF) at every boot; multi-byte values big-endian ----------------------
namespace bsp::eeprom {
static uint8_t mem[0x10000];
EepromResult init() {
    memset(mem, 0xFF, sizeof(mem));
    return OK;
}
static EepromResult rd(uint16_t a, uint8_t *d, uint16_t n) {
    if ((uint32_t)a + n > sizeof(mem)) return ERROR;
    memcpy(d, mem + a, n);
    fsim::advance(100000 + n * 25000ull);     // 400 kHz I2C
    return OK;
}
static EepromResult wr(uint16_t a, const uint8_t *d, uint16_t n) {
    if ((uint32_t)a + n > sizeof(mem)) return ERROR;
    memcpy(mem + a, d, n);
    fsim::advance(100000 + n * 25000ull);
    return OK;
}
EepromResult read_u8(uint16_t a, uint8_t *d) { return rd(a, d, 1); }
EepromResult write_u8(uint16_t a, uint8_t d) { return wr(a, &d, 1); }
EepromResult read_u16(uint16_t a, uint16_t *d) {
    uint8_t b[2];
    const EepromResult r = rd(a, b, 2);
    *d = (uint16_t)(b[0] << 8 | b[1]);
    return r;
}
EepromResult write_u16(uint16_t a, uint16_t d) {
    const uint8_t b[2] = {(uint8_t)(d >> 8), (uint8_t)d};
    return wr(a, b, 2);
}
EepromResult read_u32(uint16_t a, uint32_t *d) {
    uint8_t b[4];
    const EepromResult r = rd(a, b, 4);
    *d = (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
    return r;
}
EepromResult write_u32(uint16_t a, uint32_t d) {
    const uint8_t b[4] = {(uint8_t)(d >> 24), (uint8_t)(d >> 16), (uint8_t)(d >> 8), (uint8_t)d};
    return wr(a, b, 4);
}
EepromResult read_array(uint16_t a, uint8_t *d, uint16_t n) { return rd(a, d, n); }
EepromResult write_array(uint16_t a, uint8_t *d, uint16_t n) { return wr(a, d, n); }
void clear() { memset(mem, 0xFF, sizeof(mem)); }
void print_all() {}
const char *param_name(uint16_t address) {
    for (const auto &p : paramInfoArray)
        if (p.address == address) return p.name;
    return "UNKNOWN";
}
}  // namespace bsp::eeprom

// The simulator writes its memory as a person would with its makers' tools (a maze known before its run)
extern "C" void fsim_eeprom_write(uint32_t a, const uint8_t *d, uint32_t n) {
    if (a + n <= sizeof(bsp::eeprom::mem)) memcpy(bsp::eeprom::mem + a, d, n);
}
extern "C" void fsim_eeprom_read(uint32_t a, uint8_t *d, uint32_t n) {
    if (a + n <= sizeof(bsp::eeprom::mem)) memcpy(d, bsp::eeprom::mem + a, n);
}

// ---- Encoders (AS5047P on each wheel, its A output: 1024 counts a turn) -------------------------------------------
namespace bsp::encoders {
static constexpr float COUNTS_PER_TURN = 1024.0f;
static constexpr uint32_t STILL_US = 10000;        // no count for this long: the wheel is still
static constexpr float VELOCITY_SMOOTHING = 0.1f;  // of each new speed into the filtered one, every update
static float wheel_radius_mm = 13.25f;
static EncoderData data[2];
static int64_t counted[2];          // counts at the last clear_ticks
static uint32_t last_velocity_us;
static float linear_m_s, filtered_m_s, filtered_ang[2];

static int64_t counts_now(int s) { return (int64_t)std::floor(fsim::wheel_counts(s)); }

void init() { reset(); }
void reset() {
    for (int s = 0; s < 2; s++) {
        counted[s] = counts_now(s);
        data[s] = {0, CW, 0, STILL_US + 1, 0, 0};
    }
    reset_velocities();
}
void clear_ticks() {
    for (int s = 0; s < 2; s++) {
        counted[s] += data[s].ticks;
        data[s].ticks = 0;
    }
}
EncoderData get_data(EncoderSide side) { return data[side == LEFT ? 0 : 1]; }

// The counts since the last clear_ticks, and when the last count came (its time from the wheel's speed: the count's
// edge and the one before it).
void update_ticks() {
    const uint32_t now_us = (uint32_t)(fsim::now_ns / 1000u);
    for (int s = 0; s < 2; s++) {
        const double c = fsim::wheel_counts(s);
        data[s].ticks = (int32_t)((int64_t)std::floor(c) - counted[s]);
        const double rate = std::fabs(fsim::wheel[s].rate) * 1000.0;     // counts a microsecond
        if (rate > 1e-9) {
            const double since = (c - std::floor(c)) / rate;
            data[s].last_update_tick_time = now_us - (uint32_t)std::min(since, 1e9);
            data[s].last_delta_tick_time = (uint32_t)std::min(1.0 / rate, 1e9);
            data[s].direction = fsim::wheel[s].rate >= 0 ? CCW : CW;
        }
    }
}
void reset_velocities() {
    linear_m_s = filtered_m_s = 0;
    filtered_ang[0] = filtered_ang[1] = 0;
    last_velocity_us = (uint32_t)(fsim::now_ns / 1000u);
    data[0].linear_vel_m_s = data[1].linear_vel_m_s = 0;
}
// Each wheel's speed over the period since the last update, from its counts in it (none: still); the robot's, their
// mean, and a low-pass filtered one (what its control loop runs on).
void update_velocities(float) {
    const uint32_t now_us = get_tick_us();
    const uint32_t dt = now_us - last_velocity_us;
    last_velocity_us = now_us;
    const float mm = get_encoder_dist_mm_pulse();
    for (int s = 0; s < 2; s++) {
        EncoderData &d = data[s];
        const bool still = now_us - d.last_update_tick_time > STILL_US;
        d.linear_vel_m_s = (still || d.ticks == 0 || dt == 0) ? 0.0f : d.ticks * mm / (float)dt * MM_PER_US_TO_M_PER_S;
        d.ang_vel_rad_s = d.linear_vel_m_s / (wheel_radius_mm / 1000.0f);
        filtered_ang[s] += 0.1f * (d.ang_vel_rad_s - filtered_ang[s]);
    }
    linear_m_s = (data[0].linear_vel_m_s + data[1].linear_vel_m_s) / 2.0f;
    filtered_m_s += VELOCITY_SMOOTHING * (linear_m_s - filtered_m_s);
}
void set_right_ang_vel_rad_s(float w) {
    data[1].ang_vel_rad_s = w;
    filtered_ang[1] += 0.1f * (w - filtered_ang[1]);
}
void set_left_ang_vel_rad_s(float w) {
    data[0].ang_vel_rad_s = w;
    filtered_ang[0] += 0.1f * (w - filtered_ang[0]);
}
float get_linear_velocity_m_s() { return linear_m_s; }
float get_left_linear_velocity_m_s() { return data[0].linear_vel_m_s; }
float get_right_linear_velocity_m_s() { return data[1].linear_vel_m_s; }
float get_filtered_velocity_m_s() { return filtered_m_s; }
float get_right_filtered_ang_vel_rad_s() { return filtered_ang[1]; }
float get_left_filtered_ang_vel_rad_s() { return filtered_ang[0]; }
float get_encoder_dist_mm_pulse() { return 2.0f * (float)M_PI * wheel_radius_mm / COUNTS_PER_TURN; }
void set_wheel_radius_mm(float r) { wheel_radius_mm = r; }
float get_wheel_radius_mm() { return wheel_radius_mm; }
}  // namespace bsp::encoders

// ---- IMU (LSM6DSR): its z rate less the bias the firmware sets, lightly filtered, integrated into its angle -------
namespace bsp::imu {
static float bias_z_dps, w, raw_w, last_w, angle, incremental, a_z;
static uint32_t last_us;
static bool have_last;
ImuResult init() { return OK; }
ImuResult update() {
    const uint32_t now_us = get_tick_us();
    const float dt = have_last ? (now_us - last_us) * 1e-6f : 0.0f;
    last_us = now_us, have_last = true;
    raw_w = (float)((fsim::gyro_z_dps - bias_z_dps) * M_PI / 180.0);
    a_z = (float)fsim::accel_z_g;
    w = 0.3f * raw_w + 0.7f * last_w;
    last_w = w;
    angle += w * dt;
    incremental += w * dt;
    angle = std::remainder(angle, 2.0f * (float)M_PI);   // -pi..pi
    return OK;
}
float get_angle() { return angle; }
float get_incremental_angle() { return incremental; }
void reset_angle() {
    have_last = false;
    angle = incremental = 0;
}
void reset() {
    reset_angle();
    last_w = raw_w = 0;
}
float get_rad_per_s() { return w; }
float get_raw_rad_per_s() { return raw_w; }
float get_z_acceleration() { return a_z; }
void set_g_bias_z(float z) { bias_z_dps = z; }
float get_g_bias_z() { return bias_z_dps; }
void enable_motion_gc_filter(bool) {}      // ST's MotionGC (its gyro's bias, in its IMU calibration) is not simulated
bool is_imu_emergency() { return false; }
}  // namespace bsp::imu

// ---- Analog: the four IR sensors (emitter and phototransistor pairs) and the battery --------------------------------
namespace bsp::analog_sensors {
// Its firmware's sensing (what its search and its control loop are tuned to): mm, as its calibration gives them.
static constexpr float IR_MAX_MM = 270.0f;             // nothing in sight
static constexpr float FRONT_SEEING_MM = 160.0f;       // a front sensor sees a wall (its control)
static constexpr float START_MM = 150.0f;              // a hand in front of both front sensors: start
static constexpr float SIDE_WALL_MM = 230.0f;          // its maze's walls, as its search reads them
static constexpr float FRONT_WALL_MM = 205.0f;
static constexpr float WALL_CONFIRM_MM = 30.0f;        // a side wall seen this long is a wall
static constexpr float BATTERY_DIVIDER = 4.63f;        // its battery's divider (the ADC sees 1/4.63 of it)
static constexpr float BATTERY_SCALE = 4.19f;          // what its firmware's control uses for it (reads ~10 % low)
static constexpr float BATTERY_LOW_MV = 11300.0f;
// Each sensor's emitter: R, FL, FR, L
static constexpr leds::Emitter EMITTER[4] = {leds::RIGHT_SIDE, leds::LEFT_FRONT, leds::RIGHT_FRONT, leds::LEFT_SIDE};

static bool running, modulation;
static uint32_t raw[4];
static float dist[4], slope[4], history[4][3];
static bool have_history[4], confirmed[4];
static float seen_mm[4];
static float slope_max[4] = {90.0f, 90.0f, 90.0f, 90.0f};
static IrCalibParams calib[4];
static std::array<SensingPattern, 8> patterns;
static uint32_t battery_raw;
static uint32_t current_raw[2];
static bsp_analog_ready_callback_t ready_callback;

static IrCalibParams default_calib() { return {(float)fsim::ir_model[0], (float)fsim::ir_model[1], (float)fsim::ir_model[2]}; }
// Its walls as each sensor reads them (mm: L, FL, FR, R), for its wall pattern calibration: walls where the pattern
// has them (F-L-R, F-L, F-R, F, L-R, L, R, none), at the readings of the robot centred in a cell.
static SensingPattern default_pattern(uint8_t i) {
    static const bool walls[8][3] = {{1, 1, 1}, {1, 1, 0}, {1, 0, 1}, {1, 0, 0}, {0, 1, 1}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}};
    const float front = walls[i][0] ? 60.0f : IR_MAX_MM;
    return {walls[i][1] ? services::Config::ir_wall_dist_ref_left : IR_MAX_MM, front, front,
            walls[i][2] ? services::Config::ir_wall_dist_ref_right : IR_MAX_MM};
}

void init() {
    for (int i = 0; i < 4; i++) calib[i] = default_calib(), dist[i] = IR_MAX_MM;
    reset_all_wall_patterns();
}
void start() {
    leds::ir_emitter_all_off();
    for (int i = 0; i < 4; i++) slope[i] = 0, have_history[i] = false, confirmed[i] = false, seen_mm[i] = 0;
    running = true;
}
void stop() {
    running = false;
    leds::ir_emitter_all_off();
}
void register_callback(bsp_analog_ready_callback_t cb) { ready_callback = cb; }

float raw_to_distance_mm(SensingDirection d, uint32_t r) {
    if ((unsigned)d >= 4) return IR_MAX_MM;
    const float x = std::max((float)r + calib[d].c, 2.0f);
    return std::min(calib[d].a / std::log(x) - calib[d].b, IR_MAX_MM);
}
static uint32_t distance_to_raw(double mm) {   // the inverse of the sensor's model (fsim::ir_model)
    const double a = fsim::ir_model[0], b = fsim::ir_model[1], c = fsim::ir_model[2];
    const double r = std::exp(a / std::max(mm + b, 1.0)) - c;
    return (uint32_t)std::clamp(r, 0.0, 4095.0);
}
static void update_slope(int i, float d) {      // how fast its reading changes (mm over its last few readings)
    if (!have_history[i]) {
        history[i][0] = history[i][1] = history[i][2] = d;
        slope[i] = 0, have_history[i] = true;
        return;
    }
    slope[i] = 4.0f * (d - history[i][2]) + (history[i][0] - history[i][1]);
    history[i][2] = history[i][1], history[i][1] = history[i][0], history[i][0] = d;
}

// A new reading of every sensor (the simulator's, every 0.5 ms; on the robot its ADC's DMA, every ~0.5 ms): the
// sensor's emitter on (its modulation cycles them), or off (ambient light: nothing).
void sample() {
    battery_raw = (uint32_t)(fsim::battery_v / BATTERY_DIVIDER / 3.3 * 4095.0);
    if (!running) return;
    for (int i = 0; i < 4; i++) {
        const bool lit = modulation || (fsim::emitters >> EMITTER[i] & 1);
        raw[i] = lit ? distance_to_raw(fsim::ir_mm[i]) : 0;
        dist[i] = raw_to_distance_mm((SensingDirection)i, raw[i]);
        update_slope(i, dist[i]);
    }
    if (ready_callback) ready_callback();
}

uint32_t *ir_latest_reading() { return raw; }
float *ir_latest_distance() { return dist; }
uint32_t battery_latest_reading() { return battery_raw; }
uint32_t *current_latest_reading() { return current_raw; }
float battery_latest_reading_mv() { return battery_raw / 4095.0f * 3300.0f * BATTERY_SCALE; }
float battery_latest_reading_volts() { return battery_raw / 4095.0f * 3.3f * BATTERY_SCALE; }
float battery_latest_reading_mv_real() { return battery_raw / 4095.0f * 3300.0f * BATTERY_DIVIDER; }
float battery_latest_reading_volts_real() { return battery_raw / 4095.0f * 3.3f * BATTERY_DIVIDER; }
bool battery_low() { return battery_latest_reading_mv_real() <= BATTERY_LOW_MV; }

float ir_distance_mm(SensingDirection d) { return dist[d]; }
uint32_t ir_raw_reading(SensingDirection d) { return raw[d]; }

// A side sensor's wall threshold (mm): its own since 003dd98, set from its general parameters (Control::init)
static float wall_detect_th[2] = {240.0f, 240.0f};     // left, right: services::Config's defaults
static float detect_th(SensingDirection d) { return d == RIGHT ? wall_detect_th[1] : wall_detect_th[0]; }
void set_ir_wall_detect_th_left(float th) { wall_detect_th[0] = th; }
void set_ir_wall_detect_th_right(float th) { wall_detect_th[1] = th; }
float get_ir_wall_detect_th_left() { return wall_detect_th[0]; }
float get_ir_wall_detect_th_right() { return wall_detect_th[1]; }
bool ir_reading_wall(SensingDirection d) {
    if (d == FRONT_LEFT || d == FRONT_RIGHT) return dist[d] < FRONT_SEEING_MM;
    if (d != LEFT && d != RIGHT) return false;
    return dist[d] < detect_th(d) && std::fabs(slope[d]) < slope_max[d];
}
bool ir_is_wall_confirmed(SensingDirection d) { return (unsigned)d < 4 && confirmed[d]; }
// A side wall ends: its reading past the wall's, or growing fast (the beam slipping off the wall's end).
bool ir_wall_break_condition(SensingDirection d) {
    if (d != LEFT && d != RIGHT) return false;
    return dist[d] > detect_th(d) || slope[d] > slope_max[d];
}
void ir_update_wall_hysteresis(float travelled_mm) {
    if (travelled_mm <= 0.0f) return;
    for (SensingDirection d : {LEFT, RIGHT}) {
        if (ir_reading_wall(d) && !ir_wall_break_condition(d)) {
            seen_mm[d] += travelled_mm;
            if (seen_mm[d] >= WALL_CONFIRM_MM) confirmed[d] = true;
        } else {
            confirmed[d] = false, seen_mm[d] = 0;
        }
    }
}
void ir_reset_wall_hysteresis(SensingDirection d) {
    if ((unsigned)d < 4) confirmed[d] = false, seen_mm[d] = 0;
}
void ir_reset_all_wall_hysteresis() {
    for (int i = 0; i < 4; i++) confirmed[i] = false, seen_mm[i] = 0;
}
float ir_slope_value(SensingDirection d) { return slope[d]; }
void set_sensor_r_slope_max_th(float th) { slope_max[RIGHT] = th; }
void set_sensor_l_slope_max_th(float th) { slope_max[LEFT] = th; }
float get_sensor_r_slope_max_th() { return slope_max[RIGHT]; }
float get_sensor_l_slope_max_th() { return slope_max[LEFT]; }

bool ir_start_condition() { return dist[FRONT_LEFT] < START_MM && dist[FRONT_RIGHT] < START_MM; }

SensingStatus ir_get_sensing_status() {
    const SensingStatus st = {dist[FRONT_LEFT] < FRONT_WALL_MM && dist[FRONT_RIGHT] < FRONT_WALL_MM,
                              dist[RIGHT] < SIDE_WALL_MM, dist[LEFT] < SIDE_WALL_MM};
    static const bool log_walls = getenv("FUJIN_LOG_WALLS") != nullptr;    // the simulator's: what its search reads
    if (log_walls)
        printf("walls F%d L%d R%d (%.0f %.0f %.0f %.0f)\r\n", st.front_seeing, st.left_seeing, st.right_seeing,
               dist[LEFT], dist[FRONT_LEFT], dist[FRONT_RIGHT], dist[RIGHT]);
    return st;
}

// How far from the corridor's middle (its walls' references; with one wall, twice its error), + towards the right.
int32_t ir_side_wall_error() {
    const float l = dist[LEFT] - services::Config::ir_wall_dist_ref_left;
    const float r = dist[RIGHT] - services::Config::ir_wall_dist_ref_right;
    const bool wl = confirmed[LEFT], wr = confirmed[RIGHT];
    if (wl && wr) return (int32_t)(r - l);
    if (wl) return (int32_t)(-2.0f * l);
    if (wr) return (int32_t)(2.0f * r);
    return 0;
}
bool ir_diagonal_control_valid(SensingDirection d) {
    if (d == FRONT_LEFT) return dist[d] < services::Config::ir_diagonal_control_th_fl;
    if (d == FRONT_RIGHT) return dist[d] < services::Config::ir_diagonal_control_th_fr;
    return false;
}
// On a diagonal: the nearer front sensor's distance from its reference, + steering right.
int32_t ir_diagonal_error() {
    if (dist[FRONT_LEFT] < dist[FRONT_RIGHT])
        return ir_diagonal_control_valid(FRONT_LEFT) ? (int32_t)-(dist[FRONT_LEFT] - services::Config::ir_diagonal_ref_fl) : 0;
    return ir_diagonal_control_valid(FRONT_RIGHT) ? (int32_t)(dist[FRONT_RIGHT] - services::Config::ir_diagonal_ref_fr) : 0;
}
void enable_modulation(bool on) {
    modulation = on;
    leds::ir_emitter_all_off();
}

IrCalibParams get_calib_params(SensingDirection d) { return (unsigned)d < 4 ? calib[d] : default_calib(); }
void set_calib_params(SensingDirection d, const IrCalibParams &p) {
    if ((unsigned)d < 4) calib[d] = p;
}
void reset_calib_params(SensingDirection d) {
    if ((unsigned)d < 4) calib[d] = default_calib();
}
void reset_all_calib_params() {
    for (int i = 0; i < 4; i++) calib[i] = default_calib();
}
SensingPattern get_wall_pattern(uint8_t i) { return i < 8 ? patterns[i] : default_pattern(0); }
void set_wall_pattern(uint8_t i, const SensingPattern &p) {
    if (i < 8) patterns[i] = p;
}
void reset_wall_pattern(uint8_t i) {
    if (i < 8) patterns[i] = default_pattern(i);
}
void reset_all_wall_patterns() {
    for (uint8_t i = 0; i < 8; i++) patterns[i] = default_pattern(i);
}
const std::array<SensingPattern, 8> &get_all_wall_patterns() { return patterns; }
}  // namespace bsp::analog_sensors

extern "C" void fsim_sample_analog(void) { bsp::analog_sensors::sample(); }
