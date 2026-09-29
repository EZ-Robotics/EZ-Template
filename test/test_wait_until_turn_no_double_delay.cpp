// Coverage gap found by mutation testing exit_conditions.cpp's wait_until_turn_swing_internal():
// its polling loop advances the fake clock exactly once per pass, at the loop's own bottom -- the
// "No delay here" comments next to the turn and swing branches exist specifically because an
// extra pros::delay() inside either branch would double the real time a fixed-pass exit window
// (small/big/velocity/mA, all counted in PID-internal pass units, not wall time) takes to fire in
// practice. This measures the actual fake-clock time elapsed when the turn branch's small exit
// fires and checks it against the configured window, not just against pass count.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
// A real compute_error() call every pass -- exit_condition()'s small exit timer only credits `error`
// when a real compute has landed since it last checked (see PID.cpp), so holding error steady now
// needs an on_delay hook rather than a single pre-wait write.
void hold_small_error() { g_chassis->turnPID.compute_error(0.5, 0.0); }
}  // namespace

TEST_CASE("pid_wait_until(angle) TURN does not double up its own per-pass delay while polling") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_turn_exit_condition_set(100, 2.0, 0, 0.0, 0, 0);  // small exit only: 100 ms/2 deg
  chassis.pid_turn_set(0.0, 100);
  // No IMU movement scripted, so drive_angle_get() stays at 0 the whole test -- the target-minus-
  // current sign used by the crossing check never changes, so this can only end via the small
  // exit below, never the "past target" success path.
  g_chassis = &chassis;
  hold_small_error();  // inside small_error(2.0) throughout
  test_stub::g_clock.on_delay = hold_small_error;

  test_stub::g_clock.delay_calls_until_stop = 50;  // safety net well above the expected ~10 passes
  std::uint32_t start_ms = test_stub::g_clock.now_ms;
  bool returned = true;
  try {
    chassis.pid_wait_until(0.0);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  std::uint32_t elapsed = test_stub::g_clock.now_ms - start_ms;
  MESSAGE("returned=", returned, " elapsed_ms=", elapsed);

  REQUIRE(returned);
  // Configured small_exit_time is 100 ms. One delay per pass while polling returns close to that;
  // a reintroduced second delay per pass would take about twice as long (~200+ ms).
  CHECK(elapsed >= 100);
  CHECK(elapsed < 160);
}
