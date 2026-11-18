# ZoroBot3's firmware (github.com/OPRobots/ZoroBot3, cloned next to this repo or at ZOROBOT3_DIR) as a shared
# library for the simulator: its sources unchanged but setup.c and delay.c, on the fake libopencm3 of ratasim/opencm3/.
# -Bsymbolic: its globals are its own, whatever else is loaded (libc, other robots' firmware).
ZOROBOT3_DIR ?= $(abspath ../../../../../../ZoroBot3)
SRC      := $(ZOROBOT3_DIR)/source_code
APP      := $(filter-out $(SRC)/src/setup.c $(SRC)/src/delay.c,$(wildcard $(SRC)/src/*.c)) $(SRC)/lib/lsm6dsr_reg/lsm6dsr_reg.c
SIM      := zchip.c zsetup.c zdelay.c zwrap.c
WRAPS    := is_race_started is_motor_saturated get_encoder_avg_micrometers get_ideal_linear_speed \
            get_front_wall_distance lsm6dsr_get_gyro_z_degrees front_wall_detection get_menu_up_btn \
            get_menu_down_btn get_menu_mode_btn get_sensor_distance lsm6dsr_reset_get
BUILD    := ../build
LIB      := $(BUILD)/libzorofw.so
CFLAGS   := -std=gnu11 -O2 -g -fPIC -w -I../../opencm3 -I$(SRC)/include -I$(SRC)/lib/lsm6dsr_reg \
            -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0 -Dmain=firmware_main -Dprintf=zsim_printf

.PHONY: all clean
all: $(LIB)

$(BUILD)/obj/%.o: %.c ../../opencm3/fakecm3.h
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(LIB): $(APP) $(SIM) ../../opencm3/fakecm3.h $(wildcard $(SRC)/include/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -shared -Wl,-Bsymbolic -o $@ $(APP) $(SIM) -lm $(addprefix -Wl$(comma)--wrap=,$(WRAPS))

comma := ,
clean:
	rm -f $(LIB)
