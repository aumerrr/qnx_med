# Makefile - one binary per main(), all into bin/.  Build: make   Clean: make clean
# QNX: qcc -Vgcc_ntoaarch64le.  Run "make PLATFORM=aarch64le" (default).
PLATFORM ?= aarch64le
BUILD_PROFILE ?= debug
CC = qcc -Vgcc_nto$(PLATFORM)

CCFLAGS_release = -O2
CCFLAGS_debug   = -g -O0 -fno-builtin
CFLAGS = -Wall -fmessage-length=0 -Icommon $(CCFLAGS_$(BUILD_PROFILE))
LIBS   = -lm

BIN = bin
COMMON_SERVO = common/servo.c

# program name : sources
PROGS = adc_reader actuator_rm ctl_loop monitor cpu_hog msg_flood overrun_inject pot_servo fake_pot

SRC_adc_reader     = $(wildcard src/adc_reader/*.c)
SRC_actuator_rm    = $(wildcard src/actuator_rm/*.c) $(COMMON_SERVO)
SRC_ctl_loop       = $(wildcard src/ctl_loop/*.c)
SRC_monitor        = $(wildcard src/monitor/*.c)
SRC_cpu_hog        = src/loadgen/cpu_hog.c
SRC_msg_flood      = src/loadgen/msg_flood.c
SRC_overrun_inject = src/loadgen/overrun_inject.c
SRC_pot_servo      = tools/pot_servo.c $(COMMON_SERVO)
SRC_fake_pot       = tools/fake_pot.c

.PHONY: all clean rebuild
all: $(addprefix $(BIN)/,$(PROGS))

define PROG_RULE
$(BIN)/$(1): $$(SRC_$(1)) $$(wildcard common/*.h)
	@mkdir -p $(BIN)
	$(CC) $(CFLAGS) -o $$@ $$(SRC_$(1)) $(LIBS)
endef
$(foreach p,$(PROGS),$(eval $(call PROG_RULE,$(p))))

clean:
	rm -rf $(BIN)
rebuild: clean all
