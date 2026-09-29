// pid_wait()'s PURE_PURSUIT pre-last-point loop guards against a concurrent motion retargeting the
// path mid-wait by comparing odom_target_start's x, y AND theta against a snapshot taken when the
// wait began (exit_conditions.cpp). All three fields matter: a retarget that lands on the exact same
// final (x, y) but with a different explicit final heading is still a genuinely different motion (a
// boomerang-style approach angle change) and must be caught immediately, the same as any other
// retarget shape in this family (the matching guard in the loop's own final leg, right after this
// one, already compares all three the same way).
//
// No existing test exercises a retarget that keeps the same final x/y but changes only the explicit
// final heading, so nothing currently pins the theta comparison specifically. Without it, this
// specific retarget shape would only be caught later -- by the final loop's own guard once the path
// reaches its last point, or by StuckWatch's own progress backstop -- instead of on the very next
// pass, the way every other retarget in this family already is.
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
constexpr int GUARD_SLACK = 5;

// Cruises without ever exiting on its own within the test's pass budget, so only the retarget guard
// (catching it immediately vs. not at all until much later) can explain an early vs. a run-to-cap
// outcome.
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

TEST_CASE("pid_wait() odom PURE_PURSUIT: a same-point, theta-only concurrent retarget before the last point is caught immediately") {
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

  CHECK(returned);
  CHECK(chassis.interfered);
  CHECK(g_pass <= RETARGET_AT + GUARD_SLACK);
}
