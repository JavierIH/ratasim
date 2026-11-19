// What Bulebule's waiting loops poll (mmlib's move.c, speed.c, hmi.c), wrapped (the linker's --wrap): each call
// counts towards main()'s next yield to the simulator (bchip.c). The functions themselves are its own.
#include <stdbool.h>
#include <stdint.h>

void bsim_poll(void);

#define WRAP(type, name)                \
  type __real_##name(void);             \
  type __wrap_##name(void) {            \
    bsim_poll();                        \
    return __real_##name();             \
  }

WRAP(int32_t, get_encoder_average_micrometers)
WRAP(float, get_ideal_linear_speed)
WRAP(float, get_front_left_distance)
WRAP(float, get_front_right_distance)
WRAP(bool, button_read_user)
