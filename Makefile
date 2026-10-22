# Host-side build and verification. Nothing here needs an STM32 attached.
#
#   make            run every check
#   make sim        the surface matrix, ABS against a locked wheel
#   make trace      per-tick CSV of one dry-asphalt stop, into build/
#
# The firmware itself is built by STM32CubeIDE from firmware/abs_node/, or
# headlessly by tools/build_firmware.sh.

# CC is a built-in make variable defaulting to "cc", so "?=" would never take
# effect. Override it only when the user has not chosen a compiler themselves.
ifeq ($(origin CC),default)
CC := gcc
endif

BUILD   := build/host
APP     := firmware/common/app
SIM     := sim

CFLAGS  := -std=c99 -Wall -Wextra -Werror
LDLIBS  := -lm

APP_SRC := $(APP)/wheel_speed.c $(APP)/imu_fusion.c $(APP)/vehicle_speed.c \
           $(APP)/abs_ctrl.c $(APP)/abs.c
APP_HDR := $(wildcard $(APP)/*.h)
SIM_SRC := $(SIM)/plant.c
SIM_HDR := $(wildcard $(SIM)/*.h)

INC     := -I$(APP) -I$(SIM) -Itests

.PHONY: all test test-estimator test-ctrl test-closed-loop sim trace clean

all: test sim
	@echo "all checks passed"

test: test-estimator test-ctrl test-closed-loop

## Encoder wraparound, IMU fusion and the velocity observer, in isolation.
test-estimator: $(BUILD)/test_estimator
	$(BUILD)/test_estimator

## The modulator state machine against synthetic slip and wheel acceleration.
test-ctrl: $(BUILD)/test_ctrl
	$(BUILD)/test_ctrl

## The real controller driving the real plant through complete stops. This is
## the one that catches a system whose parts are each individually correct.
test-closed-loop: $(BUILD)/test_closed_loop
	$(BUILD)/test_closed_loop

## Stopping distance with and without ABS, across four surfaces.
sim: $(BUILD)/abs_sim
	$(BUILD)/abs_sim

## Per-tick traces for plotting. Both runs, same plant, same seed.
trace: $(BUILD)/abs_sim
	$(BUILD)/abs_sim --csv        > $(BUILD)/trace_abs.csv
	$(BUILD)/abs_sim --csv-locked > $(BUILD)/trace_locked.csv
	@echo "wrote $(BUILD)/trace_abs.csv and $(BUILD)/trace_locked.csv"

$(BUILD)/test_estimator: tests/test_estimator.c tests/test_util.h $(APP_SRC) $(APP_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/test_estimator.c $(APP_SRC) $(LDLIBS) -o $@

$(BUILD)/test_ctrl: tests/test_ctrl.c tests/test_util.h $(APP_SRC) $(APP_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/test_ctrl.c $(APP)/abs_ctrl.c $(LDLIBS) -o $@

$(BUILD)/test_closed_loop: tests/test_closed_loop.c tests/test_util.h \
                           $(APP_SRC) $(APP_HDR) $(SIM_SRC) $(SIM_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/test_closed_loop.c $(SIM_SRC) $(APP_SRC) \
	      $(LDLIBS) -o $@

$(BUILD)/abs_sim: $(SIM)/abs_sim.c $(APP_SRC) $(APP_HDR) $(SIM_SRC) $(SIM_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) $(SIM)/abs_sim.c $(SIM_SRC) $(APP_SRC) \
	      $(LDLIBS) -o $@

clean:
	rm -rf $(BUILD)
