// Coverage gap found by mutation testing set_drive_pid.cpp's pid_drive_set(): starting a new
// drive motion clears the `interfered` flag before the new motion begins. Without it, a motion
// that ended interfered (stuck, mA, or velocity exit) would leave that flag set on the Drive
// object, and a caller who checks `chassis.interfered` right after the NEXT motion's pid_wait()
// -- the ordinary way to tell whether that motion itself had trouble -- would see a stale true
// left over from the previous, unrelated motion, even though the new one finished perfectly
// cleanly.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

Drive* g_chassis = nullptr;

// Pinned error, above velocity_zero_main's default -- the first motion genuinely stalls and its
// velocity exit fires.
void pinned() {
  g_chassis->leftPID.error = 20.0;
  g_chassis->rightPID.error = 20.0;
  g_chassis->leftPID.derivative = 0.0;
  g_chassis->rightPID.derivative = 0.0;
}

// A healthy, converging second motion -- inside small_error from the very first pass.
void converged() {
  g_chassis->leftPID.error = 0.5;
  g_chassis->rightPID.error = 0.5;
  g_chassis->leftPID.derivative = 0.0;
  g_chassis->rightPID.derivative = 0.0;
}
}  // namespace

TEST_CASE("pid_drive_set() clears interfered so a clean motion isn't blamed for the previous one's trouble") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  // Velocity exit only (50 ms/5 passes); small/big/mA off -- isolates the first motion's failure
  // to a genuine VELOCITY_EXIT.
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 0.0, 50, 0);
  chassis.pid_drive_set(100, 100);

  g_chassis = &chassis;
  pinned();
  test_stub::g_clock.on_delay = pinned;
  test_stub::g_clock.delay_calls_until_stop = 200;
  bool first_returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    first_returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  REQUIRE(first_returned);
  REQUIRE(chassis.interfered);  // the first motion genuinely did stall

  // A second, unrelated motion, configured so it can only ever end cleanly: small exit only,
  // converging from its very first pass.
  chassis.pid_drive_exit_condition_set(50, 1.0, 0, 0.0, 0, 0);
  chassis.pid_drive_set(24, 100);
  converged();
  test_stub::g_clock.on_delay = converged;
  test_stub::g_clock.delay_calls_until_stop = 30;
  bool second_returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    second_returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("second_returned=", second_returned, " interfered=", chassis.interfered);

  REQUIRE(second_returned);  // the second motion must have finished (it converges immediately)
  CHECK_FALSE(chassis.interfered);
}
