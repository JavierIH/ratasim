# Futura's firmware (downloaded and unzipped by futura/config.py into FUTURA_DIR) as a shared library for the
# simulator: the files of its kit's Keil project (project/micromouse_test.uvproj) of its own, user/ and
# user_Libraries/, unchanged but for its main.c, in whose place main.c here runs its page's controller sample
# (controller-sample.c, mended by config.py into SAMPLE; include/controller.h forced into it); on the simulated chip
# (fchip.c), whose include/stm32f4xx.h stands for ST's library; include/fsim.h forced in. -Bsymbolic: its globals are
# its own (the robots' libraries share names).
FUTURA_DIR ?= $(abspath ../build/FuturaProject)
BUILD := ../build
SAMPLE ?= $(abspath $(BUILD)/controller/controller-sample.c)
LIB  := $(BUILD)/libfuturafw.so

SRC  := main.c $(FUTURA_DIR)/user/stm32f4xx_it.c \
        $(addprefix $(FUTURA_DIR)/user_Libraries/src/,adc.c button.c buzzer.c delay.c encoder.c led.c pwm.c \
          sensor_Function.c SPI.c usart.c)
# matrixDisplay.h declares its font static and matrixDisplay.c defines it (armcc took that, gcc does not): in that
# file alone, where it is the only `static`, the word is dropped.
DISPLAY := $(FUTURA_DIR)/user_Libraries/src/matrixDisplay.c
INC  := -Iinclude -I$(FUTURA_DIR)/user -I$(FUTURA_DIR)/user_Libraries/inc
# Keil's armcc took what gcc warns about (usart.c's _sys_exit with no type, its #pragma import): warnings stay quiet.
CFLAGS := -std=gnu11 -O2 -g -fPIC -w -fcommon $(INC) -include fsim.h \
          -Wno-error=implicit-int -Wno-error=implicit-function-declaration -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0
HEADERS := include/stm32f4xx.h include/fsim.h include/controller.h

.PHONY: all clean
all: $(LIB)

$(BUILD)/matrixDisplay.o: $(DISPLAY) $(HEADERS)
	@mkdir -p $(BUILD)
	@$(CC) $(CFLAGS) -Dstatic= -c -o $@ $<

$(BUILD)/controller-sample.o: $(SAMPLE) $(HEADERS) $(wildcard $(FUTURA_DIR)/user_Libraries/inc/*.h)
	@mkdir -p $(BUILD)
	@$(CC) $(CFLAGS) -include controller.h -DCONTROLLER_SAMPLE -c -o $@ $<

OBJ := $(BUILD)/matrixDisplay.o $(BUILD)/controller-sample.o
$(LIB): $(SRC) $(OBJ) fchip.c $(HEADERS) $(wildcard $(FUTURA_DIR)/user_Libraries/inc/*.h)
	@mkdir -p $(BUILD)
	@$(CC) $(CFLAGS) -shared -Wl,-Bsymbolic -o $@ $(SRC) $(OBJ) fchip.c

clean:
	rm -f $(LIB) $(OBJ)
