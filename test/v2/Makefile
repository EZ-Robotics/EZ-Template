# The rebuilt host tests. `make -C test/v2` builds ./run_tests and runs it.
# It reuses test/'s PROS stubs and doctest, and compiles the example project's src/autons.cpp so tests run its real
# default_constants() and autons.

CXX ?= g++
CXXFLAGS := -std=c++20 -g -O1 -Wall -Wextra -Wno-unused-parameter -D_USE_MATH_DEFINES -MMD -MP -I ../stub -I ../../include -iquote ../../include/EZ-Template -I ..

EXCLUDED := display.cpp sdcard.cpp health.cpp pid_tuner.cpp
LIB_SRC := $(filter-out $(foreach f,$(EXCLUDED),%/$(f)), $(wildcard ../../src/EZ-Template/*.cpp ../../src/EZ-Template/drive/*.cpp ../../src/EZ-Template/drive/set_pid/*.cpp))
LIB_OBJ := $(patsubst ../../src/%.cpp,build/lib/%.o,$(LIB_SRC))
TEST_OBJ := $(patsubst %.cpp,build/%.o,$(wildcard test_*.cpp)) build/autons.o $(patsubst %.cpp,build/%.o,$(wildcard corpus_autons.cpp)) build/example_chassis.o build/link_stubs.o build/doctest_main.o

.PHONY: test clean
.DEFAULT_GOAL := test

test: run_tests
	./run_tests

run_tests: $(LIB_OBJ) $(TEST_OBJ)
	$(CXX) -o $@ $^

build/lib/%.o: ../../src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# The example autons, through example_shim/main.h so `chassis` is the robot the test built (see robot.hpp)
build/autons.o: ../../src/autons.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -iquote example_shim -c $< -o $@

build/corpus_autons.o: corpus_autons.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -iquote example_shim -c $< -o $@

# Shared with the old suite in test/
build/link_stubs.o build/doctest_main.o: build/%.o: ../%.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -c $< -o $@

build/%.o: %.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include $(shell find build -name '*.d' 2>/dev/null)

clean:
	rm -rf build run_tests
