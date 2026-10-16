// Coverage gap found by mutation testing exit_conditions.cpp's pid_wait() PURE_PURSUIT/
// POINT_TO_POINT branch. When the robot is stuck on the final point (StuckWatch fires), it is only
// counted as "settled" (a clean finish, not a failure) when BOTH axes are inside their big-error
// windows:
//
//   bool settled = target_distance() < xyPID.exit.big_error
//               && std::fabs(current_a_odomPID.error) < current_a_odomPID.exit.big_error;
//
// If either axis alone were enough, a robot that is genuinely stuck a long way from the target on
// one axis, but happens to be facing the right direction (or vice versa), would be waved through as
// a clean success (interfered left false) instead of being flagged. No existing test pins the robot
// on the last point with only ONE axis converged -- the one hover-on-small-window test that reaches
// this branch has both axes already settled together, so it can't tell "&&" apart from "||".
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// Shipped default exit constants (odom): 90ms/1in small, 250ms/3in big, 500ms velocity, 750ms mA.
void configure(Drive& chassis) {
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
}

void start_path(Drive& chassis) {
  chassis.pid_odom_pp_set({{{0.0, 12.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 36.0, ANGLE_NOT_SET}, fwd, 110}});
  chassis.xyPID.exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.current_a_odomPID.exit_condition_set(90, 3.0, 250, 7.0, 500, 750);
}

int last_index(Drive& chassis) { return (int)DriveTestAccess::pp_movements(chassis).size() - 1; }

Drive* g_chassis = nullptr;
int g_last = 0;

// On the last point from the start: heading is dead on target (0 error, well inside both its
// small and big windows -- it genuinely small-exits for real early on) but xy is pinned 10 inches
// short the whole time (outside even its big window) -- a real stall on one axis only.
//
// target_distance() (what StuckWatch and the settled check actually read for xy) is computed from
// the robot's real odometry pose against pp_movements[pp_index].target, independent of xyPID.error
// -- both have to be set for this scenario to be internally consistent: xyPID.error so its own
// exit_condition() reads a real, unconverged error, and the pose so target_distance() agrees.
pose g_last_target{0, 0, 0};
// A real compute_error() call every pass, not a direct `.error =` write -- angle's own small exit
// timer, which this test needs to actually fire ("it genuinely small-exits for real early on"),
// only credits `error` when a real compute has landed since it last checked (see PID.cpp).
void script() {
  Drive& c = *g_chassis;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  DriveTestAccess::pp_index(c) = g_last;
  c.xyPID.compute_error(10.0, 10.0);
  c.current_a_odomPID.compute_error(0.0, 0.0);
  DriveTestAccess::odom_current(c) = {g_last_target.x, g_last_target.y - 10.0, DriveTestAccess::odom_current(c).theta};
}

struct Outcome {
  bool returned;
  bool interfered;
  double final_distance_from_target;
};

Outcome run_wait(Drive& chassis, int max_passes) {
  g_chassis = &chassis;
  g_last = last_index(chassis);
  g_last_target = DriveTestAccess::pp_movements(chassis)[g_last].target;
  script();
  test_stub::g_clock.on_delay = script;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, false, 0.0};
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  o.interfered = chassis.interfered;
  o.final_distance_from_target = util::distance_to_point(g_last_target, DriveTestAccess::odom_current(chassis));
  return o;
}
}  // namespace

TEST_CASE("pid_wait does not count a robot stuck on one axis as settled just because the other axis is on target") {
  Drive chassis = make_chassis();
  configure(chassis);
  start_path(chassis);

  Outcome o = run_wait(chassis, 400);

  REQUIRE(o.returned);
  // Real, current distance from the target is 10 inches -- nowhere close to arriving. A robot 10
  // inches short of its target must never be reported as a clean, uninterfered finish.
  CHECK(o.final_distance_from_target > 5.0);
  CHECK(o.interfered);
}
