# KERISE v1's firmware (the submodule external/kerise-v1, or KERISE_DIR: firmware/) as a shared library for the
# simulator: its files, unchanged but for one line, on our mbed and HAL (mbed.h, kchip.cpp). Its main() is built
# renamed (kerise_main): kchip.cpp runs it as its main thread.
# The one line: its goal (MazeSolver/MazeSolver_conf.h, MAZE_GOAL_LIST, one cell as published) is the maze's, GOAL
# ("x,y x,y ..."), as its maker would set it for a contest. Its files are copied to $(BUILD)/src with it changed (its
# headers include each other by their own directory).
# -funsigned-char: as on its Cortex-M4 (its Makefile's flags). -Bsymbolic: its globals are its own (the robots'
# libraries share names). -U_FORTIFY_SOURCE: Ubuntu's g++ defines _FORTIFY_SOURCE at -O2, and glibc's headers then
# turn its printf calls into __printf_chk, past kchip.cpp's printf (what it prints would go to the terminal).
KERISE_DIR ?= $(abspath ../../../external/kerise-v1)
FW        := $(KERISE_DIR)/firmware
BUILD     := ../build
SRC       := $(BUILD)/src
OBJ       := $(BUILD)/obj
LIB       := $(BUILD)/libkerisefw.so
GOAL      ?= 7,7 7,8 8,7 8,8
CXX       ?= g++
CXXFLAGS  := -std=gnu++14 -O2 -g -fPIC -funsigned-char -fno-builtin-printf -fno-builtin-puts -U_FORTIFY_SOURCE -I.

FW_FILES := $(wildcard $(FW)/*.h $(FW)/*.cpp $(FW)/MazeSolver/*.h $(FW)/MazeSolver/*.cpp)
GOAL_LIST := {$(shell echo '$(GOAL)' | sed -E 's/([0-9]+),([0-9]+)/IndexVec(\1,\2)/g; s/ +/,/g')}
APP  := main MazeSolver/Agent MazeSolver/Maze MazeSolver/mazeData MazeSolver/Operation MazeSolver/ShortestPath
OBJS := $(APP:%=$(OBJ)/app/%.o) $(OBJ)/kchip.o
OURS := mbed.h rtos.h Makefile

.PHONY: all clean FORCE
all: $(LIB)

# Its goal: the copy remade when it changes.
$(BUILD)/goal: FORCE
	@mkdir -p $(BUILD)
	@echo '$(GOAL_LIST)' | cmp -s - $@ || echo '$(GOAL_LIST)' > $@

$(SRC)/.copied: $(FW_FILES) $(BUILD)/goal
	@rm -rf $(SRC) && mkdir -p $(SRC)/MazeSolver
	@cp $(FW)/*.h $(FW)/*.cpp $(SRC)/ && cp $(FW)/MazeSolver/*.h $(FW)/MazeSolver/*.cpp $(SRC)/MazeSolver/
	@sed -i -E 's/^#define MAZE_GOAL_LIST .*/#define MAZE_GOAL_LIST $(GOAL_LIST)/' $(SRC)/MazeSolver/MazeSolver_conf.h
	@grep -q 'MAZE_GOAL_LIST $(GOAL_LIST)' $(SRC)/MazeSolver/MazeSolver_conf.h
	@touch $@

$(OBJ)/app/main.o: $(SRC)/.copied $(OURS)
	@mkdir -p $(dir $@)
	@$(CXX) $(CXXFLAGS) -Dmain=kerise_main -c -o $@ $(SRC)/main.cpp

$(OBJ)/app/%.o: $(SRC)/.copied $(OURS)
	@mkdir -p $(dir $@)
	@$(CXX) $(CXXFLAGS) -c -o $@ $(SRC)/$*.cpp

$(OBJ)/kchip.o: kchip.cpp $(OURS)
	@mkdir -p $(dir $@)
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

$(LIB): $(OBJS)
	@$(CXX) -shared -Wl,-Bsymbolic -Wl,--no-undefined -o $@ $(OBJS) -lm

clean:
	rm -rf $(LIB) $(OBJ) $(SRC) $(BUILD)/goal
