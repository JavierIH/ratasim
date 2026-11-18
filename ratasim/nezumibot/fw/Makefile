# NezumiBot's firmware (the submodule external/NezumiBot, or NEZUMIBOT_DIR) as a shared library for the simulator:
# its sources unchanged but setup.c and delay.c, on the fake libopencm3 of ratasim/opencm3/ (stdint.h and stdbool.h
# forced in: the chip's newlib brings them with stdio.h).
# -Bsymbolic: its globals are its own (controlHand.c's `error` would be libc's error(), and the robots' libraries
# share names).
NEZUMIBOT_DIR ?= $(abspath ../../../external/NezumiBot)
SRC      := $(NEZUMIBOT_DIR)/source_code
APP      := $(filter-out $(SRC)/src/setup.c $(SRC)/src/delay.c,$(wildcard $(SRC)/src/*.c))
SIM      := nchip.c ndelay.c
WRAPS    := get_sensor_bool
BUILD    := ../build
LIB      := $(BUILD)/libnezumifw.so
CFLAGS   := -std=gnu11 -O2 -g -fPIC -w -I../../opencm3 -I$(SRC)/include \
            -include stdint.h -include stdbool.h -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0 \
            -Dmain=firmware_main -Dprintf=nsim_printf

.PHONY: all clean
all: $(LIB)

$(LIB): $(APP) $(SIM) ../../opencm3/fakecm3.h $(wildcard $(SRC)/include/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -shared -Wl,-Bsymbolic -o $@ $(APP) $(SIM) -lm $(addprefix -Wl$(comma)--wrap=,$(WRAPS))

comma := ,
clean:
	rm -f $(LIB)
