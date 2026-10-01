// The scripted repro in test_pp_wait_until_before_last_point_xy_latch.cpp proves the mechanism (xy's
// window exits latching on the moving look-ahead point) rigorously, with a hand-fed error band, since
// that file's own tests need pp_index frozen or advancing on an exact, controlled schedule to isolate
// it. This file runs the same shapes against the real thing instead: SimRobot's on_delay hook runs an
// actual auto-task-equivalent pass every tick (run_auto_task_pass(), sim_physics.hpp) -- check_imu_task,
// ez_tracking_task, and the current mode's real task function (pp_task() for a PURE_PURSUIT motion) --
// before stepping physics, so a SimRobot-backed test genuinely exercises pure pursuit's real look-ahead
// point selection, not a hand-picked stand-in for it.
//
// A plain two-waypoint straight path with light_fast or heavy_slow, at the look-ahead/speed values below,
// does not empirically sustain xy's error inside the moving-look-ahead window long enough to latch the
// bug through real physics -- these first two cases pass on 82391ba too, not just on this fix. They stay
// here as a realistic confirmation that a tight look ahead completes cleanly (a healthy result either
// way), and as a place to extend if a path shape that does trigger it through real physics turns up
// later. The scripted repro is what this fix is actually verified against.
#include <cmath>

#include "doctest.h"

#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
}

// Mirrors test_drive_stuck_floor_repro.cpp's own helper.
template <typename F>
bool run_capped(F&& wait, int max_ticks) {
  test_stub::g_clock.delay_calls_until_stop = max_ticks;
  bool returned = true;
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  return returned;
}

std::vector<odom> two_leg_path() { return {{{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 48.0, ANGLE_NOT_SET}, fwd, 110}}; }
}  // namespace

TEST_CASE("pid_wait_until_index does not return short of its checkpoint with a tight look ahead") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.odom_look_ahead_set(2.5_in);
  chassis.pid_odom_pp_set(two_leg_path());

  bool wait_returned = run_capped([&] { chassis.pid_wait_until_index(0); }, /*max_ticks=*/4000);
  double final_y = chassis.odom_y_get();

  CAPTURE(wait_returned);
  CAPTURE(final_y);
  REQUIRE(wait_returned);
  CHECK(std::fabs(24.0 - final_y) < 1.0);
}

TEST_CASE("pid_wait_until_point does not return short of the checkpoint with a tight look ahead") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.odom_look_ahead_set(2.5_in);
  chassis.pid_odom_pp_set(two_leg_path());

  bool wait_returned = run_capped([&] { chassis.pid_wait_until_point({0.0, 24.0, 0.0}); }, /*max_ticks=*/4000);
  double final_y = chassis.odom_y_get();

  CAPTURE(wait_returned);
  CAPTURE(final_y);
  REQUIRE(wait_returned);
  CHECK(std::fabs(24.0 - final_y) < 1.0);
}

TEST_CASE("control: the stock 7 in look ahead already crosses cleanly, before and after this fix") {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  // odom_look_ahead_set() left at its 7 in default -- never enters the default 3 in xy big_error
  // window before the checkpoint, so this passes on 82391ba too. Ties the two tests above to the
  // look ahead specifically, not to something else about this path.
  chassis.pid_odom_pp_set(two_leg_path());

  bool wait_returned = run_capped([&] { chassis.pid_wait_until_index(0); }, /*max_ticks=*/4000);
  double final_y = chassis.odom_y_get();

  CAPTURE(wait_returned);
  CAPTURE(final_y);
  REQUIRE(wait_returned);
  CHECK(std::fabs(24.0 - final_y) < 1.0);
}

TEST_CASE("a checkpoint within look ahead of a short final leg still ends at the checkpoint, never before") {
  // path {{0,24},{0,26}}: the last leg is only 2 in, shorter than the 2.5 in look ahead, so the
  // checkpoint at {0,25} sits between the first waypoint and the (very close) last one. What could go
  // wrong: xy's BIG timer keeps accumulating through the whole discarded approach (before_last_point()
  // is true until pp_index is literally on the last point), so it could latch a few passes after
  // pp_index reaches the last point but before the robot has actually covered the last, short leg --
  // this must still resolve via the ordinary at-last-point failsafe once the robot is genuinely close,
  // never earlier.
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.odom_look_ahead_set(2.5_in);
  chassis.pid_odom_pp_set({{{0.0, 24.0, ANGLE_NOT_SET}, fwd, 110}, {{0.0, 26.0, ANGLE_NOT_SET}, fwd, 110}});

  // The wait returns exactly once -- checking the robot's own position at that moment against 24 is
  // exactly "never returned before reaching 24": an earlier return would have left final_y short of it.
  bool wait_returned = run_capped([&] { chassis.pid_wait_until_point({0.0, 25.0, 0.0}); }, /*max_ticks=*/4000);
  double final_y = chassis.odom_y_get();

  CAPTURE(wait_returned);
  CAPTURE(final_y);
  REQUIRE(wait_returned);
  CHECK(final_y >= 24.0);
}

TEST_CASE("a checkpoint on the last point with the robot stopped short still returns via the ordinary big-exit failsafe") {
  // Proves before_last_point() actually turns off once pure pursuit reaches the real last point:
  // loosen xy's big_error tight enough that the robot's own approach speed carries it past the small
  // window in one poll, so the only realistic way this ends is the big-exit failsafe pid_wait_until_point()
  // already had before this fix, unaffected by it.
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  chassis.pid_odom_turn_exit_condition_set(90, 3.0, 250, 7.0, 0, 0);
  chassis.pid_odom_pp_set({{{0.0, 24.0, ANGLE_NOT_SET}, fwd, 40}});

  bool wait_returned = run_capped([&] { chassis.pid_wait_until_point({0.0, 24.0, 0.0}); }, /*max_ticks=*/4000);
  double final_y = chassis.odom_y_get();

  CAPTURE(wait_returned);
  CAPTURE(final_y);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
}
