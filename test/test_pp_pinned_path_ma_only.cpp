// A robot pinned partway through a pure pursuit path ends the wait as interfered, with the velocity exit off and only an mA exit set.  The pinned
// path check uses the velocity exit time as its window, or the mA timeout when there is no velocity exit time.
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
bool run_wait(const std::function<void()>& wait, const std::function<void(int)>& hook, int max_delays) {
  static int g_pass;
  static const std::function<void(int)>* g_hook;
  g_pass = 0;
  g_hook = &hook;
  test_stub::g_clock.on_delay = [] { (*g_hook)(++g_pass); };
  test_stub::g_clock.delay_calls_until_stop = max_delays;
  bool returned = true;
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  return returned;
}

bool pinned_path(int velocity_exit_time, int mA_timeout, bool& interfered) {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_pp_set({{{0.0, 12.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110}});
  chassis.xyPID.exit_condition_set(90, 1.0, 0, 0.0, velocity_exit_time, mA_timeout);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 0, 0.0, velocity_exit_time, mA_timeout);
  DriveTestAccess::pp_index(chassis) = 0;
  bool returned = run_wait([&] { chassis.pid_wait(); },
                           [&](int) {
                             chassis.xyPID.compute_error(5.0, 0.0);              // far from the path point and not moving
                             chassis.current_a_odomPID.compute_error(0.0, 0.0);  // heading right, so its small exit latches
                           },
                           400);
  interfered = chassis.interfered;
  return returned;
}
}  // namespace

TEST_CASE("a robot pinned partway through a pure pursuit path ends the wait with only an mA exit set") {
  bool interfered = false;
  REQUIRE(pinned_path(0, 500, interfered));
  CHECK(interfered);
}

TEST_CASE("the velocity exit time is still the pinned path window when both are set") {
  bool interfered = false;
  REQUIRE(pinned_path(200, 5000, interfered));  // 5000 ms would be 500 passes, past the 400 the case allows
  CHECK(interfered);
}
