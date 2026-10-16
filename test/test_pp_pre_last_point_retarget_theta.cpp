// pid_wait() follows a new motion set mid-wait, and tells a new motion from the one it is waiting on by comparing odom_target_start's x, y
// AND theta against what they were when the wait began.  All three fields matter: a retarget that lands on the exact same final (x, y)
// but with a different explicit final heading is still a different motion (a boomerang-style approach angle change), so the wait has to
// start over on it.  This pins the theta comparison in PURE_PURSUIT's pre-last-point loop: a retarget that keeps the same final x/y and
// changes only the final heading.
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

constexpr int RETARGET_AT = 10;

// Holds xy outside small_error and big_error and never advances pp_index, so the path can only end on the mid-path stall check.
void on_delay(Drive& c, int n) {
  double e = 7.3 - 0.15 * (n % 3);
  c.xyPID.error = e;
  c.xyPID.derivative = 0.3;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
  if (n == RETARGET_AT) {
    // Same final (x, y) as the original path's last point (0, 45) -- only the explicit final
    // heading differs (ANGLE_NOT_SET -> 90), a real second pure pursuit path, still not on its
    // last point when this lands.
    std::vector<odom> new_path;
    for (int i = 1; i <= 44; i++) new_path.push_back({{0.0, (double)i, ANGLE_NOT_SET}, fwd, 100});
    new_path.push_back({{0.0, 45.0, 90}, fwd, 100});
    c.pid_odom_pp_set(new_path);
  }
}

void hook() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  on_delay(*g_chassis, g_pass);
}
}  // namespace

TEST_CASE("pid_wait() odom PURE_PURSUIT: a same-point, theta-only concurrent retarget before the last point is followed") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);

  std::vector<odom> path;
  for (int i = 1; i <= 44; i++) path.push_back({{0.0, (double)i, ANGLE_NOT_SET}, fwd, 100});
  path.push_back({{0.0, 45.0, ANGLE_NOT_SET}, fwd, 100});  // original: same x/y, no explicit theta
  chassis.pid_odom_pp_set(path);

  g_chassis = &chassis;
  g_pass = 0;
  on_delay(chassis, 0);
  test_stub::g_clock.on_delay = hook;
  test_stub::g_clock.delay_calls_until_stop = 300;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=" << returned << " passes=" << g_pass << " interfered=" << chassis.interfered);

  // The stall check ends a pinned path once speed has read 0 for over 1000 ms with no movement and then over the 500 ms velocity exit
  // time: 151 polls, counted from the start of the wait.  Without the retarget that is pass 151.  The wait starts over on the new path at
  // the retarget, so its stall count does too, and it ends RETARGET_AT passes later.  The new path is pinned too, so it ends interfered.
  CHECK(returned);
  CHECK(chassis.interfered);
  CHECK(g_pass == RETARGET_AT + 151);
}
