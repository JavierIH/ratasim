# UKMARSBOT's firmware, mazerunner-core (the submodule external/mazerunner-core, or MAZERUNNER_DIR: Peter Harrison's
# last commit of October 2025, as his ORION) as a shared library for the simulator: copied by gen_src.py (its event
# set to EVENT_UK, one float-to-int conversion written as the AVR does it), built for the Arduino Nano (ATmega328P:
# __AVR__, its register code) on the simulated core of arduino/ and uchip.cpp.
# -Bsymbolic: its globals are its own (the robots' libraries share names).
MAZERUNNER_DIR ?= $(abspath ../../../external/mazerunner-core)
BUILD        := ../build
GEN          := $(BUILD)/src
LIB          := $(BUILD)/libukmarsbotfw.so
SRC          := $(MAZERUNNER_DIR)/mazerunner-core
DEFS         := -D__AVR__ -D__AVR_ATmega328P__ -DARDUINO=10819 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR -DF_CPU=16000000L
CXXFLAGS     := -std=gnu++17 -O2 -g -fPIC -w $(DEFS) -Iarduino

.PHONY: all clean
all: $(LIB)

$(GEN)/mazerunner-core.ino: $(SRC)/mazerunner-core.ino $(wildcard $(SRC)/*.h) gen_src.py
	@python3 gen_src.py $(SRC) $(GEN)

$(LIB): $(GEN)/mazerunner-core.ino uchip.cpp $(wildcard arduino/*.h arduino/*/*.h)
	@$(CXX) $(CXXFLAGS) -shared -Wl,-Bsymbolic -o $@.tmp -x c++ $(GEN)/mazerunner-core.ino -x none uchip.cpp -lm
	@mv $@.tmp $@

clean:
	rm -rf $(LIB) $(GEN)
