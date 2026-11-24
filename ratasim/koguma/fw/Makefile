# KOGUMA's firmware (the submodule external/KOGUMA.MkIIl, or KOGUMA_DIR) as a shared library for the simulator: all
# of its Core/Src, unchanged, but for what only its ARM build needs (system_stm32f4xx.c, syscalls.c, sysmem.c: its
# clock's registers, newlib's calls), on the simulated chip (kchip.c), whose include/stm32f4xx_hal.h stands for ST's
# HAL (its Drivers/ are not used). include/ksim.h forced into each of its files: each pass of its loops takes time.
# Its main() built renamed (koguma_main). -Bsymbolic: its globals are its own (the robots' libraries share names).
# -fno-builtin-printf, -U_FORTIFY_SOURCE: its printf is the chip's (its USART), not the C library's.
# -Wno-error=...: what gcc 14 made errors and its ARM compiler took (its uint16_t buffer to HAL_ADC_Start_DMA).
KOGUMA_DIR ?= $(abspath ../../../external/KOGUMA.MkIIl)
CORE  := $(KOGUMA_DIR)/Core
BUILD := ../build
OBJ   := $(BUILD)/obj
LIB   := $(BUILD)/libkogumafw.so
SKIP  := system_stm32f4xx.c syscalls.c sysmem.c
SRC   := $(filter-out $(addprefix $(CORE)/Src/,$(SKIP)),$(wildcard $(CORE)/Src/*.c))
OBJS  := $(patsubst $(CORE)/Src/%.c,$(OBJ)/%.o,$(SRC)) $(OBJ)/kchip.o
HEADERS := $(wildcard include/*.h) $(wildcard $(CORE)/Inc/*.h) Makefile
CFLAGS := -std=gnu11 -O2 -g -fPIC -w -fcommon -fno-builtin-printf -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0 \
          -Wno-error=incompatible-pointer-types -Wno-error=int-conversion -Wno-error=implicit-function-declaration \
          -DSTM32F446xx -Iinclude -I$(CORE)/Inc

.PHONY: all clean
all: $(LIB)

$(OBJ)/main.o: $(CORE)/Src/main.c $(HEADERS)
	@mkdir -p $(OBJ)
	@$(CC) $(CFLAGS) -include ksim.h -Dmain=koguma_main -c -o $@ $<

$(OBJ)/%.o: $(CORE)/Src/%.c $(HEADERS)
	@mkdir -p $(OBJ)
	@$(CC) $(CFLAGS) -include ksim.h -c -o $@ $<

$(OBJ)/kchip.o: kchip.c $(HEADERS)
	@mkdir -p $(OBJ)
	@$(CC) $(CFLAGS) -c -o $@ $<

$(LIB): $(OBJS)
	@$(CC) -shared -Wl,-Bsymbolic -Wl,--no-undefined -o $@ $(OBJS) -lm

clean:
	rm -rf $(LIB) $(OBJ)
