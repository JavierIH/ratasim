# Gamusino's sketch (the submodule external/gamusino, or GAMUSINO_DIR: src/gamusino/gamusino.ino) as a shared library
# for the simulator: turned into C++ as the Arduino IDE does (gen_ino.py), built for arduino-esp32 2.x (ESP32) on the
# simulated core of arduino/ and gchip.cpp, with the library its maker installs for its gyro, STM32duino LSM6DSR
# (LSM6DSR_DIR: its src/, downloaded at build time by gamusino/robot.py), unchanged.
# -Bsymbolic: its globals are its own (the robots' libraries share names).
GAMUSINO_DIR ?= $(abspath ../../../external/gamusino)
BUILD        := ../build
LSM6DSR_DIR  ?= $(BUILD)/LSM6DSR
SKETCH       ?= $(GAMUSINO_DIR)/src/gamusino/gamusino.ino
GEN          := $(BUILD)/gen
LIB          := $(BUILD)/libgamusinofw.so
DEFS         := -DESP32 -DARDUINO=10819 -DARDUINO_ARCH_ESP32 -DARDUINO_ESP32_DEV
INC          := -Iarduino -I$(LSM6DSR_DIR)
CXXFLAGS     := -std=gnu++17 -O2 -g -fPIC -w $(DEFS) $(INC)
CFLAGS       := -std=gnu11 -O2 -g -fPIC -w $(DEFS) $(INC)

.PHONY: all clean
all: $(LIB)

$(GEN)/sketch.cpp: $(SKETCH) gen_ino.py
	@mkdir -p $(GEN)
	python3 gen_ino.py $(SKETCH) $@

$(GEN)/lsm6dsr_reg.o: $(LSM6DSR_DIR)/lsm6dsr_reg.c
	@mkdir -p $(GEN)
	@$(CC) $(CFLAGS) -c -o $@ $<

$(LIB): $(GEN)/sketch.cpp $(GEN)/lsm6dsr_reg.o $(LSM6DSR_DIR)/LSM6DSRSensor.cpp gchip.cpp $(wildcard arduino/*.h)
	@$(CXX) $(CXXFLAGS) -shared -Wl,-Bsymbolic -o $@ $(GEN)/sketch.cpp $(LSM6DSR_DIR)/LSM6DSRSensor.cpp gchip.cpp \
	  $(GEN)/lsm6dsr_reg.o -lm

clean:
	rm -rf $(LIB) $(GEN)
