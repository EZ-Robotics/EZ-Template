// wait_until_drive() on an odom move, cruising straight and healthy but slower than the velocity exit's 5 in/s.
//
// PID.cpp's velocity exit counts the robot as stopped under velocity_sensor_main_exit (default 0.05 in per 10 ms tick, 5 in/s),
// independent of wheel rpm, wheel diameter, or the speed passed to the motion setter.  A real drivetrain can cruise slower than
// that: a 200rpm/2.75in drivetrain at speed 20 cruises at ~2.9-3.4in/s.  This is v3.2.2's tradeoff and the branch keeps it: the
// velocity exit arms once the robot has moved faster than 5 in/s or after 1000 ms, then ends the wait velocity_exit_time later,
// interfered, short of the requested distance.  Two sites can end it:
//   1. POINT_TO_POINT, where on_last_point is unconditionally true: xyPID's velocity exit (through xy_exit_condition()).
//   2. PURE_PURSUIT on a multi-point path that never advances pp_index (ptp_task/pp_task are never run by this host build), so
//      on_last_point is always false and xyPID's exit is never called: leftPID/rightPID's velocity exits.
//
// This test scripts xyPID and leftPID/rightPID directly (same style test_wait_until_point_ignores_velocity_exit.cpp and
// test_wait_until_drive_odom_progress.cpp use): a real compute_error() every pass with each PID's reading moving 3 in/s, and the
// real left/right sensor readings moving with it.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// A real, low-gearing/low-speed straight cruise: genuinely progressing, but slower than the 5 in/s (0.05 in / 10ms) velocity
// exit -- see the file comment for the configuration this stands in for (2.9-3.4in/s; 3.0in/s here sits inside that band).
constexpr double CRUISE_IN_PER_PASS = 0.03;  // 3.0 in/s -- under the 5 in/s floor, above the 1.5 in/s stop speed

Drive* g_chassis = nullptr;
double g_l_start = 0.0, g_r_start = 0.0;
double g_driven = 0.0;
double g_xy_error_start = 0.0;

void set_sensor_inches(std::vector<pros::Motor>& motors, double tick_per_inch, double inches) {
  std::int32_t ticks = (std::int32_t)(inches * tick_per_inch);
  for (auto& m : motors) m.fake().position = ticks;
}

void crawl() {
  Drive& c = *g_chassis;
  ez::detail::stats.auto_task_passes.fetch_add(1, std::memory_order_relaxed);
  g_driven += CRUISE_IN_PER_PASS;
  double tpi = c.drive_tick_per_inch();
  set_sensor_inches(c.left_motors, tpi, g_l_start + g_driven);
  set_sensor_inches(c.right_motors, tpi, g_r_start + g_driven);

  // xyPID tracks the REAL odom target the whole time: its error shrinks every pass, in step with the real distance driven, and
  // never gets anywhere near small_error/big_error during this test.
  c.xyPID.compute_error(g_xy_error_start - g_driven, c.xyPID.cur + CRUISE_IN_PER_PASS);

  // Angle stays converged and inert -- current_a_odomPID must not be what ends this wait.
  c.current_a_odomPID.compute_error(0.0, c.current_a_odomPID.cur);

  // leftPID/rightPID see the same 3 in/s cruise on the same encoders.  Their target is the frozen look-ahead point set once by
  // raw_pid_odom_ptp_set() at motion start (7in default, dir=fwd).
  c.leftPID.compute_error(c.leftPID.target_get() - (g_l_start + g_driven), c.leftPID.cur + CRUISE_IN_PER_PASS);
  c.rightPID.compute_error(c.rightPID.target_get() - (g_r_start + g_driven), c.rightPID.cur + CRUISE_IN_PER_PASS);

  // Not a turn-bias pivot -- xy_translation_bias_gated stays false the whole time, so xy's velocity exit is not vetoed.
  DriveTestAccess::xy_translation_bias_gated(c) = false;
}
}  // namespace

TEST_CASE("wait_until_drive() on_last_point: a straight cruise slower than 5 in/s ends on xy's velocity exit") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(true);
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  // Shipped default constants.
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);

  // A single point-to-point move far beyond anything this test drives to, so on_last_point is
  // true for the whole run (mode == POINT_TO_POINT) and xyPID never itself nears convergence.
  // speed=20 matches the configuration this test's cruise rate is drawn from; the max_xy_speed cap is
  // not a promise of that exact cruise, so the actual, slower rate is scripted directly (see CRUISE_IN_PER_PASS).
  odom movement{{0.0, 500.0}, fwd, 20};
  chassis.pid_odom_ptp_set(movement);

  g_chassis = &chassis;
  g_l_start = chassis.drive_sensor_left();
  g_r_start = chassis.drive_sensor_right();
  g_driven = 0.0;
  g_xy_error_start = 500.0;  // real distance to the actual odom target at the start of the crawl

  test_stub::g_clock.on_delay = crawl;
  // A 3 in/s cruise reaches a 30in wait_until() target in 10s == 1000 passes; the budget is past that.
  test_stub::g_clock.delay_calls_until_stop = 1500;

  bool returned = true;
  try {
    chassis.pid_wait_until(30.0);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " inches_driven=", g_driven);

  // The cruise never goes faster than 5 in/s, so the velocity exit arms after 1000 ms and ends the wait 500 ms later (plus a pass or
  // two of rounding), far from the target: interfered, short of 30in.
  REQUIRE(returned);
  CHECK(chassis.interfered);
  CHECK(g_driven >= CRUISE_IN_PER_PASS * (100 + 50));
  CHECK(g_driven <= CRUISE_IN_PER_PASS * (100 + 50 + 5));
}

// The SECOND site: leftPID/rightPID's own velocity exits.  A multi-point pure pursuit path with pp_index never advancing
// (ptp_task()/pp_task() are never run by this host build) keeps on_last_point permanently false, so xyPID.exit_condition() is
// never even reached here -- any exit in this case can only be the leftPID/rightPID site.
TEST_CASE("wait_until_drive() pre-last-point (PURE_PURSUIT): the same slow cruise ends on left/right's velocity exits") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(true);
  DriveTestAccess::imu_calibration_complete(chassis) = true;

  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 750);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 750);

  // A multi-point path far beyond anything this test drives to. pp_index stays 0 the whole test
  // (nothing here runs pp_task), which is not the last index, so on_last_point is false throughout.
  std::vector<odom> path;
  for (int i = 1; i <= 5; i++) path.push_back({{0.0, 100.0 + i, ANGLE_NOT_SET}, fwd, 20});
  chassis.pid_odom_pp_set(path);

  g_chassis = &chassis;
  g_l_start = chassis.drive_sensor_left();
  g_r_start = chassis.drive_sensor_right();
  g_driven = 0.0;
  g_xy_error_start = 100.0;  // xyPID's exit is never reached; its error is what judges an odom wait interfered

  test_stub::g_clock.on_delay = crawl;
  test_stub::g_clock.delay_calls_until_stop = 1500;

  bool returned = true;
  try {
    chassis.pid_wait_until(30.0);
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;

  MESSAGE("returned=", returned, " interfered=", chassis.interfered, " inches_driven=", g_driven);

  // Same timing as above, through left/right's velocity exits; xy is ~95 in from its point, outside big_error: interfered.
  REQUIRE(returned);
  CHECK(chassis.interfered);
  CHECK(g_driven >= CRUISE_IN_PER_PASS * (100 + 50));
  CHECK(g_driven <= CRUISE_IN_PER_PASS * (100 + 50 + 5));
}
