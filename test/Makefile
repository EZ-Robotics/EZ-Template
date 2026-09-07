# Host test target: compiles the pure-logic parts of EZ-Template's library on
# the developer's machine (or CI), with no PROS toolchain, no VEX brain, and
# no arm-none-eabi-* cross compiler -- just a regular desktop C++20 compiler.
#
# `make -C test` (or `make` from this directory) builds ./run_tests and runs
# it; a failing test fails the build, so this is safe to wire into CI as-is.

CXX ?= g++
CXXSTD := -std=c++20

# display.cpp, sdcard.cpp, health.cpp, pid_tuner.cpp, and user_input.cpp are
# excluded: they're screen/SD-card/controller-input code with no host-testable
# logic of their own, and pulling them in would drag in more of the PROS LCD
# API than this stub layer is worth building out. Where something we DO
# compile calls into a symbol only one of those files defines, it's stubbed
# in link_stubs.cpp instead.
EXCLUDED_SRC := display.cpp sdcard.cpp health.cpp pid_tuner.cpp user_input.cpp
ALL_LIB_SRC := $(wildcard ../src/EZ-Template/*.cpp ../src/EZ-Template/drive/*.cpp ../src/EZ-Template/drive/set_pid/*.cpp)
LIB_SRC := $(filter-out $(foreach f,$(EXCLUDED_SRC),%/$(f)), $(ALL_LIB_SRC))

TEST_SRC := $(wildcard test_*.cpp) link_stubs.cpp

# stub/ goes before ../include so e.g. "api.h" and "pros/motors.hpp" resolve
# to the stubs, not (nonexistent, on a host build) real PROS headers. The
# -iquote lets EZ-Template/piston.cpp's bare `#include "util.hpp"` resolve
# the way PROS's per-source-directory -iquote would on device. The plain
# `-I .` lets stub/pros/rtos.hpp's `#include "fake_hardware.hpp"` resolve
# from every translation unit, not just ones that happen to live next to it.
INCLUDES := -I stub -I ../include -iquote ../include/EZ-Template -I .

# _USE_MATH_DEFINES: M_PI is a POSIX/glibc extension, not a C++ standard
# guarantee. Harmless everywhere; only actually needed on an MSVC-headers
# host build (used to develop this suite locally, no g++ available there).
CXXFLAGS := $(CXXSTD) -g -Wall -Wextra -Wno-unused-parameter -D_USE_MATH_DEFINES $(INCLUDES)

BIN := run_tests

.PHONY: test clean
.DEFAULT_GOAL := test

$(BIN): $(LIB_SRC) $(TEST_SRC) doctest.h fake_hardware.hpp
	$(CXX) $(CXXFLAGS) -o $@ $(LIB_SRC) $(TEST_SRC)

test: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
