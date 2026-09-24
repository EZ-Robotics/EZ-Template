// Coverage gap found by mutation testing set_turn_pid.cpp's turn_set_internal(): starting a new
// turn resets turnPID's exit-condition timers before setting the new target. Without it, a turn
// that leaves turnPID's small-exit timer partway accumulated (e.g. aborted or superseded by
// another pid_turn_set() call before it ever actually exits) would hand that leftover count
// straight to the next turn, which could then fire an exit almost immediately instead of getting
// its own full small_exit_time.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

Drive* g_chassis = nullptr;

void hold_small_error() {
  g_chassis->turnPID.error = 0.5;  // inside small_error(2.0) throughout
}
}  // namespace

TEST_CASE("pid_turn_set() resets turnPID's timers so a new turn gets a fresh exit window") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  // Small exit only (200 ms/2 deg); big/velocity/mA off so nothing else can end either wait.
  chassis.pid_turn_exit_condition_set(200, 2.0, 0, 0.0, 0, 0);

  chassis.pid_turn_set(90.0, 100);
  g_chassis = &chassis;
  hold_small_error();
  test_stub::g_clock.on_delay = hold_small_error;

  // Accumulate turnPID's small-exit timer to 190ms (19 passes) without ever letting it fire --
  // safely under the 200ms threshold.
  test_stub::g_clock.delay_calls_until_stop = 20;
  bool first_returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    first_returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  REQUIRE_FALSE(first_returned);  // must not have actually exited yet

  // A second, different turn -- simulates an aborted/superseded turn immediately followed by a
  // fresh one. hold_small_error() keeps the new turn's error inside small_error from its very
  // first pass too, so the only question is whether it inherits the old accumulated timer.
  chassis.pid_turn_set(45.0, 100);

  test_stub::g_clock.delay_calls_until_stop = 8;  // budget far short of a fresh 200ms/20-pass window
  bool second_returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    second_returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("second_returned=", second_returned);

  // Correct code: turnPID's small-exit timer was reset to 0 by the second pid_turn_set(), so the
  // new turn needs its own fresh ~20 passes and must NOT have returned within 8.
  CHECK_FALSE(second_returned);
}
