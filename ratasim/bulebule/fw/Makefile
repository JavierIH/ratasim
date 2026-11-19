# Bulebule's firmware (the submodule external/bulebule, or BULEBULE_DIR) as a shared library for the simulator: its
# sources and mmlib's unchanged but mmlib's clock.c (bclock.c), with its printf (mpaland/printf), on the fake
# libopencm3 of include/ (fakef1.h; stdint.h and stdbool.h forced in: the chip's newlib brings them).
# -Bsymbolic: its globals are its own (the robots' libraries share names).
BULEBULE_DIR ?= $(abspath ../../../external/bulebule)
SRC      := $(BULEBULE_DIR)/src
APP      := $(wildcard $(SRC)/*.c) $(SRC)/printf/printf.c $(filter-out $(SRC)/mmlib/clock.c,$(wildcard $(SRC)/mmlib/*.c))
SIM      := bchip.c bclock.c bwrap.c
WRAPS    := get_encoder_average_micrometers get_ideal_linear_speed get_front_left_distance get_front_right_distance \
            button_read_user
BUILD    := ../build
LIB      := $(BUILD)/libbulebulefw.so
CFLAGS   := -std=gnu11 -O2 -g -fPIC -w -I. -Iinclude -I$(SRC) -DSTM32F1 -D_GNU_SOURCE \
            -include stdint.h -include stdbool.h -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0 -Dmain=firmware_main

.PHONY: all clean
all: $(LIB)

$(LIB): $(APP) $(SIM) fakef1.h $(wildcard $(SRC)/*.h $(SRC)/mmlib/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -shared -Wl,-Bsymbolic -o $@ $(APP) $(SIM) -lm $(addprefix -Wl$(comma)--wrap=,$(WRAPS))

comma := ,
clean:
	rm -f $(LIB)
