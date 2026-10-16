// A pure pursuit path that ends driving into a goal or a wall settles its heading first, then xy goes over current.  The angle exit latches
// SMALL, xy reaches the mA exit, and the wait has to end as interfered.  It used to need the angle exit to reach the mA or velocity exit too, and
// waited forever on a heading that had already settled.
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
}  // namespace

TEST_CASE("xy over current while the heading has settled ends a pure pursuit wait as interfered") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_pp_set({{{0.0, 12.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110}});
  chassis.xyPID.exit_condition_set(90, 1.0, 0, 0.0, 0, 300);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 0, 0.0, 0, 0);
  DriveTestAccess::pp_index(chassis) = 0;
  bool returned = run_wait([&] { chassis.pid_wait(); },
                           [&](int pass) {
                             chassis.xyPID.compute_error(20.0, 0.3 * pass);      // still driving at 30 in/s, so nothing says it is stopped
                             chassis.current_a_odomPID.compute_error(0.0, 0.0);  // heading right, so its small exit latches
                             if (pass > 20) chassis.left_motors[0].fake().over_current = true;
                           },
                           400);
  REQUIRE(returned);
  CHECK(chassis.interfered);
}
