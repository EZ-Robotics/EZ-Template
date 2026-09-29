// Coverage gap in exit_conditions.cpp's Channel::made(), the shared progress primitive behind
// StuckWatch/SingleStuckWatch: `if (size >= low - step) return false;` only credits progress on a
// reading strictly smaller than the last recorded low, minus a full step. At step == 0 (small_error
// == 0, and the velocity-exit-noise-floor fallback zeroed out too via velocity_sensor_main_exit_set,
// since stuck_step() otherwise falls back to that instead of 0), that means a reading exactly equal
// to the last low must still not count as progress -- only a strictly smaller one may. Loosening
// that strict inequality (`>` instead of `>=`) would let an unchanged reading credit progress at
// this boundary, defeating the whole backstop for a genuinely pinned robot in this configuration:
// "no change" would read identically to "new record low".
//
// No existing test exercises step == 0 specifically (small_error == 0 while a stuck backstop is
// still active, via mA_timeout as the fallback window), so this pins the documented boundary
// behavior directly.
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
int g_pass = 0;

// Perfectly static error every pass: never closes, never opens. At step == 0 this must never read
// as progress -- it is not a new low, just the same reading again.
void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_chassis->leftPID.error = 10.0;
  g_chassis->leftPID.derivative = 0.0;
  g_chassis->rightPID.error = 10.0;
  g_chassis->rightPID.derivative = 0.0;
}
}  // namespace

TEST_CASE("pid_wait() DRIVE: a perfectly static error at StuckWatch's step==0 configuration is still caught, not read as endless progress") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  // small_error=0, velocity_exit_time=0 -> Channel step==0 (stuck_step()) only once the velocity
  // exit's own noise-floor fallback is also zeroed (otherwise stuck_step() falls back to that
  // instead of a literal 0). mA_timeout=500 keeps SingleStuckWatch's window active (mA_timeout is
  // used as the fallback window when velocity_exit_time==0). big_error/big_exit_time and velocity
  // are all off so nothing but the stuck backstop itself can end this wait.
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 0.0, 0, 500);
  chassis.leftPID.velocity_sensor_main_exit_set(0.0);
  chassis.rightPID.velocity_sensor_main_exit_set(0.0);
  chassis.pid_drive_set(20.0, 100);

  g_chassis = &chassis;
  g_pass = 0;
  on_delay();  // initial state before the wait's first delay
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = 300;  // generous cap: real fire is ~150 passes (1500ms)
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=" << returned << " passes=" << g_pass << " interfered=" << chassis.interfered);

  // On the real, unmodified library code: a perfectly static error credits no progress at
  // step==0 (strictly-smaller only), so SingleStuckWatch's start-allowance (1000ms) plus its
  // window (500ms via the mA_timeout fallback) fires at ~1500ms (~150 passes), well inside the
  // 300-pass cap.
  CHECK(returned);
  CHECK(chassis.interfered);
  CHECK(g_pass < 250);
}
