// pid_wait_until_point() on a slow, healthy approach slower than the velocity exit's 5 in/s.  Its xy exit is xyPID's exit
// condition, the same as pid_wait()'s odom branch, and the velocity exit counts a robot under 5 in/s as stopped.  This is
// v3.2.2's tradeoff and the branch keeps it: the velocity exit arms once the robot has moved faster than 5 in/s or after 1000 ms,
// then ends the wait velocity_exit_time later, interfered, long before the robot arrives.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// A slow, steady 2.5 in/s crawl toward (0, 100) starting from (0, 0): above the 1.5 in/s stop speed, below the velocity exit's
// 5 in/s.  A real compute_error() lands every pass with xyPID's reading moving at the crawl's speed.  current_a_odomPID stays
// converged throughout so only the xy side is under test.
constexpr double TARGET_Y = 100.0;
constexpr double STEP_PER_PASS = 0.025;  // in/pass = 2.5 in/s at util::DELAY_TIME=10ms
Drive* g_chassis = nullptr;
double g_y = 0.0;

void crawl() {
  Drive& c = *g_chassis;
  g_y = std::fmin(TARGET_Y, g_y + STEP_PER_PASS);
  DriveTestAccess::odom_current(c) = {0.0, g_y, 0.0};
  c.xyPID.compute_error(TARGET_Y - g_y, g_y);
  c.current_a_odomPID.compute_error(0.0, c.current_a_odomPID.cur);
}
}  // namespace

TEST_CASE("pid_wait_until_point() ends an approach slower than 5 in/s on the velocity exit") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 0);  // shipped-default-shaped xy
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 0);   // angle stays converged
  chassis.pid_odom_ptp_set({{0.0, TARGET_Y, 0.0}, fwd, 100});

  g_chassis = &chassis;
  g_y = 0.0;
  crawl();
  test_stub::g_clock.on_delay = crawl;
  // The full 100 in at 2.5 in/s is ~4000 passes; this budget covers it, bounded.
  test_stub::g_clock.delay_calls_until_stop = 4500;

  bool returned = true;
  try {
    chassis.pid_wait_until_point({0.0, TARGET_Y, 0.0});
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " final_y=", g_y);

  // The crawl never goes faster than 5 in/s, so xy's velocity exit arms after 1000 ms and ends the wait 500 ms later (plus a pass or
  // two of rounding), ~96 in from the point and outside xy's big_error: interfered, far short.
  REQUIRE(returned);
  CHECK(chassis.interfered);
  CHECK(g_y >= STEP_PER_PASS * (100 + 50));
  CHECK(g_y <= STEP_PER_PASS * (100 + 50 + 5));
}
