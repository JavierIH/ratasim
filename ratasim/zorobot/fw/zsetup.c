// Replaces ZoroBot3's setup.c: the chip needs no configuration here; what setup() does beyond the hardware stays.
#include "move.h"
#include "setup.h"

void setup(void) {
  setup_kinematics();
  setup_spi_high_speed();
  lsm6dsr_init();
}

void setup_spi_high_speed(void) {
  delay(100);
}

void setup_spi_low_speed(void) {
  delay(100);
}
