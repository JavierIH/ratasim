// The getters ZoroBot3's waiting loops poll (move.c, floodfill.c, menus), wrapped by the linker (--wrap): every
// few calls they give the time back to the simulator, as a loop on the chip would let the interrupts run.
#include <stdbool.h>
#include <stdint.h>

void zsim_poll(void);

#define WRAP(ret, name)            \
  ret __real_##name(void);         \
  ret __wrap_##name(void);         \
  ret __wrap_##name(void) {        \
    zsim_poll();                   \
    return __real_##name();        \
  }
WRAP(bool, is_race_started)
WRAP(bool, is_motor_saturated)
WRAP(int32_t, get_encoder_avg_micrometers)
WRAP(int32_t, get_ideal_linear_speed)
WRAP(uint16_t, get_front_wall_distance)
WRAP(float, lsm6dsr_get_gyro_z_degrees)
WRAP(bool, front_wall_detection)
WRAP(bool, get_menu_up_btn)
WRAP(bool, get_menu_down_btn)
WRAP(bool, get_menu_mode_btn)

uint16_t __real_get_sensor_distance(uint8_t pos);
uint16_t __wrap_get_sensor_distance(uint8_t pos);
uint16_t __wrap_get_sensor_distance(uint8_t pos) {
  zsim_poll();
  return __real_get_sensor_distance(pos);
}

// lsm6dsr_init() waits for the reset bit through a read the firmware never implements (platform_read leaves the
// buffer as it was): the simulated chip reports the reset done.
typedef struct stmdev_ctx_t stmdev_ctx_t;
int32_t __wrap_lsm6dsr_reset_get(stmdev_ctx_t *ctx, uint8_t *val);
int32_t __wrap_lsm6dsr_reset_get(stmdev_ctx_t *ctx, uint8_t *val) {
  (void)ctx;
  *val = 0;
  return 0;
}
