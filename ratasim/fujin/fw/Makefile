# Fujin's firmware (the submodule external/fujin, or FUJIN_DIR: firmware/) as a shared library for the simulator: its
# application (main loop, FSM, services, algorithms, utils), unchanged, on our BSP (fbsp.cpp: its inc/bsp interface)
# and simulated chip (fchip.cpp). Its main() is built renamed (fujin_main): fchip.cpp runs the same loop, timed.
# C++23, as its CMakeLists: g++ 13 or later, or clang++ 15 or later (fujin/robot.py picks one: CXX, CXXFLAGS_EXTRA,
# LDFLAGS_EXTRA).
# -Bsymbolic: its globals are its own (the robots' libraries share names).
# -U_FORTIFY_SOURCE: Ubuntu's g++ defines _FORTIFY_SOURCE at -O2, and glibc's headers then turn its printf calls into
# __printf_chk, past fconsole.cpp's printf: what it prints (its states) would go to the terminal, not the simulator.
FUJIN_DIR ?= $(abspath ../../../external/fujin)
FW        := $(FUJIN_DIR)/firmware
BUILD     := ../build
OBJ       := $(BUILD)/obj
LIB       := $(BUILD)/libfujinfw.so
CXX       ?= g++
CXXFLAGS_EXTRA ?=
LDFLAGS_EXTRA ?=
CXXFLAGS  := -std=gnu++2b -O2 -g -fPIC -w -fno-builtin-printf -U_FORTIFY_SOURCE -I. -I$(FW)/inc -include utils/types.hpp \
             $(CXXFLAGS_EXTRA)

APP := $(FW)/src/main.cpp $(wildcard $(FW)/src/fsm/*.cpp $(FW)/src/fsm/states/*.cpp $(FW)/src/algorithms/*.cpp \
         $(FW)/src/utils/*.cpp $(FW)/src/services/*.cpp)
OURS := fchip.cpp fbsp.cpp fconsole.cpp fprobe.cpp
OBJS := $(patsubst $(FW)/src/%.cpp,$(OBJ)/app/%.o,$(APP)) $(patsubst %.cpp,$(OBJ)/%.o,$(OURS))
HEADERS := $(wildcard $(FW)/inc/*/*.hpp) fsim.h Makefile

.PHONY: all clean
all: $(LIB)

$(OBJ)/app/main.o: $(FW)/src/main.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	@$(CXX) $(CXXFLAGS) -Dmain=fujin_main -c -o $@ $<

$(OBJ)/app/%.o: $(FW)/src/%.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJ)/%.o: %.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

$(LIB): $(OBJS)
	@$(CXX) -shared -Wl,-Bsymbolic -Wl,--no-undefined $(LDFLAGS_EXTRA) -o $@ $(OBJS) -lm

clean:
	rm -rf $(LIB) $(OBJ)
