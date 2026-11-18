# Perenquen's firmware (the submodule external/perenquen, or PERENQUEN_DIR) with its library aversive4dspic
# (external/aversive4dspic, or AVERSIVE_DIR) as a shared library for the simulator: the files of its MPLAB X project
# (maindspic.X/nbproject/configurations.xml), unchanged, for its dsPIC33EP512MC806 (DSPIC, __dsPIC33E__) but not
# HOST_VERSION, on the simulated chip: its part header turned into host memory at build time (gen_sfr.py),
# pdspic.h forced in, aversive's dsPIC UART driver replaced by puart.c.
# -Bsymbolic: its globals are its own (the robots' libraries share names).
PERENQUEN_DIR ?= $(abspath ../../../external/perenquen)
AVERSIVE_DIR  ?= $(abspath ../../../external/aversive4dspic)
SRC  := $(PERENQUEN_DIR)/Software/perenquen-board
M    := $(AVERSIVE_DIR)/modules
BUILD := ../build
GEN  := $(BUILD)/gen
LIB  := $(BUILD)/libperenquenfw.so

APP  := main cmdline cs sensor strat_utils strat_base strat traps commands actuator telemetry
MODS := base/cirbuf base/scheduler base/math/fixed_point base/math/geometry base/math/vect2 base/time \
        debug/error ihm/parse ihm/rdline ihm/vt100 hardware/dspic/oscillator \
        devices/control_system/filters/pid devices/control_system/filters/quadramp \
        devices/control_system/control_system_manager devices/robot/robot_system \
        devices/robot/trajectory_manager devices/robot/position_manager \
        devices/robot/blocking_detection_manager devices/encoders/dspic/encoders_dspic
LIBSRC := $(foreach f,cirbuf_get_tail cirbuf cirbuf_add_buf_head cirbuf_add_buf_tail cirbuf_add_head cirbuf_add_tail \
            cirbuf_align cirbuf_del_buf_head cirbuf_del_buf_tail cirbuf_del_head cirbuf_del_tail cirbuf_get_buf_head \
            cirbuf_get_buf_tail cirbuf_get_head,$(M)/base/cirbuf/$(f).c) \
          $(foreach f,scheduler scheduler_add scheduler_del scheduler_dump scheduler_interrupt,$(M)/base/scheduler/$(f).c) \
          $(foreach f,f64_double f64_mul f64_msb_mul f64_print f64_int,$(M)/base/math/fixed_point/$(f).c) \
          $(foreach f,vect_base lines polygon,$(M)/base/math/geometry/$(f).c) \
          $(M)/base/math/vect2/vect2.c $(M)/base/time/time.c $(M)/debug/error/error.c \
          $(foreach f,parse parse_num parse_string,$(M)/ihm/parse/$(f).c) $(M)/ihm/rdline/rdline.c $(M)/ihm/vt100/vt100.c \
          $(M)/hardware/dspic/oscillator/oscillator.c \
          $(M)/devices/control_system/filters/pid/pid.c $(M)/devices/control_system/filters/quadramp/quadramp.c \
          $(M)/devices/control_system/control_system_manager/control_system_manager.c \
          $(M)/devices/robot/robot_system/angle_distance.c $(M)/devices/robot/robot_system/robot_system.c \
          $(M)/devices/robot/trajectory_manager/trajectory_manager.c \
          $(M)/devices/robot/position_manager/position_manager.c \
          $(M)/devices/robot/blocking_detection_manager/blocking_detection_manager.c \
          $(M)/devices/encoders/dspic/encoders_dspic/encoders_dspic.c
SIM  := pchip.c puart.c $(GEN)/sfr.c
INC  := -I$(GEN) -Iinclude -I$(SRC) -I$(AVERSIVE_DIR)/include -I$(AVERSIVE_DIR)/include/aversive \
        $(addprefix -I$(M)/,$(MODS)) -I$(M)/comm/dspic/uart -I$(M)/debug/diagnostic -I$(M)/debug -I$(M)/base -I$(M)
# gcc 14+ makes errors of what older ones warned about (its sensor.c: NULL for an unsigned char): warnings again.
CFLAGS := -std=gnu11 -O2 -g -fPIC -w -fno-toplevel-reorder -fcommon $(INC) \
          -Wno-error=int-conversion -Wno-error=incompatible-pointer-types -Wno-error=implicit-function-declaration \
          -Wno-error=implicit-int \
          -DDSPIC -D__dsPIC33E__ -D__dsPIC33EP512MC806__ -include pdspic.h \
          -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0 -Dmain=firmware_main -Dprintf=psim_printf -Dvfprintf=psim_vfprintf

.PHONY: all clean
all: $(LIB)

$(GEN)/sfr.c: gen_sfr.py $(SRC)/p33EP512MC806.h
	@mkdir -p $(GEN)
	python3 gen_sfr.py $(SRC)/p33EP512MC806.h $(GEN)

$(LIB): $(addprefix $(SRC)/,$(addsuffix .c,$(APP))) $(LIBSRC) $(SIM) include/pdspic.h include/libpic30.h $(wildcard $(SRC)/*.h)
	@$(CC) $(CFLAGS) -shared -Wl,-Bsymbolic -o $@ $(addprefix $(SRC)/,$(addsuffix .c,$(APP))) $(LIBSRC) $(SIM) -lm

clean:
	rm -rf $(BUILD)/libperenquenfw.so $(GEN)
