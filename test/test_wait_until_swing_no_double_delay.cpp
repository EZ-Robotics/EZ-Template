// Same coverage gap as the matching turn test, for wait_until_turn_swing_internal()'s swing
// branch: the "No delay here" comment next to it exists for the same reason -- an extra
// pros::delay() there would double the real time a fixed-pass exit window takes to fire, since
// small/big/velocity/mA are all counted in PID-internal pass units, not wall time.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}
}  // namespace

TEST_CASE("pid_wait_until(angle) SWING does not double up its own per-pass delay while polling") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_swing_exit_condition_set(100, 2.0, 0, 0.0, 0, 0);  // small exit only: 100 ms/2 deg
  chassis.pid_swing_set(ez::LEFT_SWING, 0.0, 100);
  // No IMU movement scripted, so drive_angle_get() stays at 0 the whole test -- the target-minus-
  // current sign used by the crossing check never changes, so this can only end via the small
  // exit below, never the "past target" success path.
  chassis.swingPID.error = 0.5;  // inside small_error(2.0) throughout -- no on_delay needed

  test_stub::g_clock.delay_calls_until_stop = 50;  // safety net well above the expected ~10 passes
  std::uint32_t start_ms = test_stub::g_clock.now_ms;
  bool returned = true;
  try {
    chassis.pid_wait_until(0.0);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  std::uint32_t elapsed = test_stub::g_clock.now_ms - start_ms;
  MESSAGE("returned=", returned, " elapsed_ms=", elapsed);

  REQUIRE(returned);
  // Configured small_exit_time is 100 ms. One delay per pass while polling returns close to that;
  // a reintroduced second delay per pass would take about twice as long (~200+ ms).
  CHECK(elapsed >= 100);
  CHECK(elapsed < 160);
}
