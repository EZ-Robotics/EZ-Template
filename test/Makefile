# Host tests: compiles the pure-logic parts of EZ-Template's library on the developer's machine (or CI), with no PROS toolchain, no VEX brain,
# and no arm-none-eabi-* cross compiler, just a regular desktop C++20 compiler.
#
# `make -C test` runs the tests in v2/ and then the compile matrix. A failing test fails the build, so this is safe to wire into CI as-is.
# The PROS stubs (stub/), doctest and the fake hardware in this folder are shared with v2/.

CXX ?= g++
CXXSTD := -std=c++20

.PHONY: test clean compile-matrix
.DEFAULT_GOAL := test

test: compile-matrix
	$(MAKE) -C v2 CXX="$(CXX)"

clean:
	$(MAKE) -C v2 clean

# What upgrading looks like from a user's project (see compile_matrix.sh). Deliberately
# does not get the library's include path: those put ../include, and any okapi in it, on the include path.
compile-matrix:
	sh compile_matrix.sh "$(CXX)" "$(CXXSTD) -Wall -Wextra -Wno-unused-parameter -D_USE_MATH_DEFINES"
