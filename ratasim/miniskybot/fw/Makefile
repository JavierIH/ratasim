# Miniskybot's program (miniskybot/miniskybot.ino, ratasim's own) as a shared library for the simulator: built for
# its Skymega board (an ATmega328P at 16 MHz) on the simulated Arduino core of arduino/ and mchip.cpp.
# -Bsymbolic: its globals are its own (the robots' libraries share names).
BUILD    := ../build
LIB      := $(BUILD)/libminiskybotfw.so
DEFS     := -D__AVR_ATmega328P__ -DARDUINO=10819 -DARDUINO_ARCH_AVR -DF_CPU=16000000L
CXXFLAGS := -std=gnu++17 -O2 -g -fPIC -Wall -Wno-unused-parameter -U_FORTIFY_SOURCE $(DEFS) -Iarduino

.PHONY: all clean
all: $(LIB)

$(LIB): miniskybot/miniskybot.ino mchip.cpp $(wildcard arduino/*.h) Makefile
	@mkdir -p $(BUILD)
	@$(CXX) $(CXXFLAGS) -shared -Wl,-Bsymbolic -o $@.tmp -x c++ miniskybot/miniskybot.ino -x none mchip.cpp -lm
	@mv $@.tmp $@

clean:
	rm -f $(LIB)
