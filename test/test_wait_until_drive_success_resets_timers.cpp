// Coverage gap found by mutation testing exit_conditions.cpp's wait_until_drive(): the success
// path (the robot has driven past the intermediate target) resets leftPID/rightPID's timers
// before returning, same as the failsafe path already does implicitly through exit_condition()'s
// own SMALL/BIG/VELOCITY_EXIT branches. wait_until_drive() is meant to be called more than once
// per motion (an intermediate look-ahead point, then another, then a final pid_wait()) without an
// intervening pid_drive_set() -- so if the success path left leftPID/rightPID's small-exit timer
// wherever it happened to be, a second wait_until_drive() call on the same ongoing motion could
// inherit an almost-expired timer and fire its own failsafe almost immediately, instead of giving
// the new leg its own full small_exit_time.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// Sets both drive sensors to `inches`.
void set_sensors(Drive& chassis, double inches) {
  std::int32_t ticks = (std::int32_t)(inches * chassis.drive_tick_per_inch());
  chassis.left_motors[0].fake().position = ticks;
  chassis.right_motors[0].fake().position = ticks;
}

Drive* g_chassis = nullptr;
double g_error_value = 0.0;  // held within small_error the whole time, both waits
int g_pass = 0;
int g_move_at = -1;  // pass at which to physically move the robot past the first target; -1: never
double g_move_to = 0.0;

void on_delay() {
  ++g_pass;
  Drive& c = *g_chassis;
  c.leftPID.error = g_error_value;
  c.rightPID.error = g_error_value;
  if (g_pass == g_move_at) set_sensors(c, g_move_to);
}
}  // namespace

TEST_CASE("wait_until_drive() resets leftPID/rightPID's timers on its own success path") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  // Small exit only (100 ms/1 in); big/velocity/mA off so nothing else can end either wait.
  chassis.pid_drive_exit_condition_set(100, 1.0, 0, 0.0, 0, 0);

  set_sensors(chassis, 0.0);
  chassis.pid_drive_set(64.0, 100);  // resets leftPID/rightPID timers itself -- not what's under test

  g_chassis = &chassis;
  g_pass = 0;
  g_error_value = 0.5;  // inside small_error(1.0) throughout both waits
  g_move_at = 9;        // move past the first target on pass 9: j accumulates to 80ms, under the
                         // 100ms threshold, then the move ends the wait via the SUCCESS branch,
                         // not the small-exit failsafe.
  g_move_to = 24.5;      // past the first wait_until_drive(24) target
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 50;

  bool first_returned = true;
  try {
    chassis.pid_wait_until(24.0);
  } catch (test_stub::StopLoop&) {
    first_returned = false;
  }
  REQUIRE(first_returned);  // must have ended via the physical crossing, not a timeout
  CHECK_FALSE(chassis.interfered);

  // Second leg of the same ongoing motion -- no pid_drive_set() in between. Never physically
  // reaches its target (g_move_at not reset), so the only way this second wait can end is via
  // leftPID/rightPID's own small-exit failsafe.
  g_move_at = -1;
  test_stub::g_clock.delay_calls_until_stop = 5;  // budget far short of a fresh 100ms/10-pass window

  bool second_returned = true;
  try {
    chassis.pid_wait_until(48.0);
  } catch (test_stub::StopLoop&) {
    second_returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("second_returned=", second_returned);

  // Correct code: leftPID/rightPID's small-exit timer was reset to 0 by the first wait's success
  // path, so the second leg needs its own fresh ~10 passes and must NOT have returned within 5.
  CHECK_FALSE(second_returned);
}
