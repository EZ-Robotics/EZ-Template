// Coverage gap found by mutation testing exit_conditions.cpp's pid_wait_until_point(): its xy
// exit is run through without_velocity(), the same as pid_wait()'s odom branch, because a
// velocity exit doesn't mean an odom wait is done -- a robot pivoting at a corner, or just moving
// slowly, can read as "stopped" to the velocity check while it's still genuinely making progress
// toward the point. StuckWatch (checked separately, right below) is what actually decides stuck
// for an odom wait; a raw, unfiltered VELOCITY_EXIT reaching xy_exit here would end the wait on a
// slow-but-healthy approach long before it actually arrives.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// A slow, steady 2.5 in/s crawl toward (0, 100) starting from (0, 0) -- above the library's own
// documented "StuckWatch never fires on a healthy crawl" floor, but well below velocity_zero_main
// (0.05) once expressed as a derivative, so xyPID's own velocity exit genuinely arms and would
// fire around velocity_exit_time if it weren't filtered here. current_a_odomPID stays converged
// throughout so only the xy side is under test.
constexpr double TARGET_Y = 100.0;
constexpr double STEP_PER_PASS = 0.025;  // in/pass = 2.5 in/s at util::DELAY_TIME=10ms
Drive* g_chassis = nullptr;
double g_y = 0.0;

void crawl() {
  Drive& c = *g_chassis;
  g_y = std::fmin(TARGET_Y, g_y + STEP_PER_PASS);
  DriveTestAccess::odom_current(c) = {0.0, g_y, 0.0};
  c.xyPID.error = TARGET_Y - g_y;
  c.xyPID.derivative = 0.02;  // moving, but below velocity_zero_main(0.05) -- reads as "stopped"
  // Also drive the raw reading itself (real motion, every tick genuinely fresh), not just
  // derivative -- so this keeps exercising a real velocity exit even once a raw value that
  // never changes stops counting as stalled evidence on its own (see test_pid.cpp).
  c.xyPID.cur += 0.02;
  c.current_a_odomPID.error = 0.0;
  c.current_a_odomPID.derivative = 0.0;
}
}  // namespace

TEST_CASE("pid_wait_until_point() does not end a slow, healthy approach on a raw velocity exit") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 0);   // shipped-default-shaped xy
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 500, 0);    // angle stays converged
  chassis.pid_odom_ptp_set({{0.0, TARGET_Y, 0.0}, fwd, 100});

  g_chassis = &chassis;
  g_y = 0.0;
  crawl();
  test_stub::g_clock.on_delay = crawl;
  // 12 in/s would cover 100 in in well under this many passes; the crawl here is far slower
  // (2.5 in/s, ~4000 ms/400 passes for the full 100 in), so this budget is generous but bounded.
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

  // Correct code: the raw VELOCITY_EXIT keeps getting converted back to RUNNING every pass, so
  // this keeps going until the robot actually arrives (xy small-exits once within 1 in for 90 ms,
  // which can settle a couple of passes short of exactly TARGET_Y) -- a clean, uninterfered finish
  // far past where the unfiltered mutant would have cut it off (~pass 151, well under y=4).
  REQUIRE(returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(g_y >= TARGET_Y - 5.0);
}
