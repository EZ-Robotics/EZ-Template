// A team that loosens its drive exit constants to make an auton faster, such as (300 ms, 3 in, 500 ms, 7 in, 500 ms, 500 ms), must not
// lose the wait's ability to see a healthy cruise through, nor its ability to catch a pinned robot.  A healthy 200 rpm drive commanded at a
// normal speed (30) cruises above the 5 in/s the velocity exit counts as stopped and is not reported interfered; a robot that is not moving
// at all, or is held 5 in short of its target, still ends the wait with interfered=true.  Turns are the same at their own speeds against the
// 4 deg/s stop speed.
//
// Uses the suite's own physics sim (sim_physics.hpp) and its "sticky_high_friction" archetype, the same convention
// test_drive_stuck_floor_repro.cpp established for testing the velocity exit's speed floor.
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
}  // namespace

TEST_CASE("DRIVE pid_wait() does not report interfered on a healthy cruise at speed 60 under loosened exits") {
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  chassis.pid_exit_speed_window_set(40);  // 40 ms: even number of ticks, see test_speed_window.cpp
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  // (300 ms, 3 in, 500 ms, 7 in, 500 ms, 500 ms)
  chassis.pid_drive_exit_condition_set(300, 3.0, 500, 7.0, 500, 500);
  double leg_length_in = 24.0;
  int speed = 60;
  chassis.pid_drive_set(leg_length_in, speed);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double final_pos = chassis.drive_sensor_left();

  CAPTURE(wait_returned);
  CAPTURE(final_pos);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(leg_length_in - final_pos) < 7.0);
}

namespace {
Drive* g_pinned_chassis = nullptr;
void refresh_pinned_drive_pids() {
  DriveTestAccess::refresh(g_pinned_chassis->leftPID);
  DriveTestAccess::refresh(g_pinned_chassis->rightPID);
}
}  // namespace

TEST_CASE("DRIVE pid_wait() pinned still reports interfered promptly with the same loosened exits") {
  // Control for the test above: loosening exits must not also blind the wait to a real stall.  A robot that is not moving at all reads as
  // stopped for any exit constants -- so this doesn't need its own sim archetype; a plain, hand-scripted pin (same technique
  // test_pp_wait_stuck.cpp's pinned_from_start uses) is enough.
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(300, 3.0, 500, 7.0, 500, 500);
  chassis.pid_drive_set(24.0, 30);
  chassis.leftPID.error = 12.0;
  chassis.rightPID.error = 12.0;
  g_pinned_chassis = &chassis;
  refresh_pinned_drive_pids();
  test_stub::g_clock.on_delay = refresh_pinned_drive_pids;
  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/400);
  test_stub::g_clock.on_delay = nullptr;

  CAPTURE(wait_returned);
  REQUIRE(wait_returned);
  CHECK(chassis.interfered);
}

namespace {
Drive* g_noisy_pinned_chassis = nullptr;
int g_noisy_pinned_pass = 0;
// Pinned 5 in short of a 24 in target, with a bounded +/-0.05 in encoder jitter around that fixed point.  The jitter alternates sign every
// pass, so over the 30 ms speed window the net movement is at most 0.1 in (3 ticks of 0.05 in alternating) and the robot reads as stopped.  Deterministic
// (alternating, not a real RNG) so the pass bound below is exact, not probabilistic.
void jittered_pin() {
  ++g_noisy_pinned_pass;
  double jitter = (g_noisy_pinned_pass % 2 == 0) ? 0.05 : -0.05;
  g_noisy_pinned_chassis->leftPID.error = 19.0 + jitter;
  g_noisy_pinned_chassis->rightPID.error = 19.0 + jitter;
  DriveTestAccess::refresh(g_noisy_pinned_chassis->leftPID);
  DriveTestAccess::refresh(g_noisy_pinned_chassis->rightPID);
}
}  // namespace

TEST_CASE(
    "DRIVE pid_wait() pinned 5 in short under loosened exits still reports interfered within the arming fallback plus velocity_exit_time, despite noise") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(300, 3.0, 500, 7.0, 500, 500);
  chassis.pid_drive_set(24.0, 30);
  g_noisy_pinned_chassis = &chassis;
  g_noisy_pinned_pass = 0;
  jittered_pin();
  test_stub::g_clock.on_delay = jittered_pin;
  // An explicit, generous bound: the 1000 ms arming fallback plus the 500 ms velocity_exit_time is 150 passes.  400 matches the noise-free
  // pinned test above.
  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/400);
  test_stub::g_clock.on_delay = nullptr;

  CAPTURE(wait_returned);
  CAPTURE(g_noisy_pinned_pass);
  REQUIRE(wait_returned);
  CHECK(chassis.interfered);
}

TEST_CASE("TURN pid_wait() does not report interfered on a healthy turn with loosened exits") {
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  chassis.pid_exit_speed_window_set(40);  // 40 ms: even number of ticks, see test_speed_window.cpp
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  // (300 ms, 8 deg, 500 ms, 15 deg, 500 ms, 500 ms)
  chassis.pid_turn_exit_condition_set(300, 8.0, 500, 15.0, 500, 500);
  chassis.pid_turn_set(90, 45);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double final_angle = chassis.drive_imu_get();

  CAPTURE(wait_returned);
  CAPTURE(final_angle);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(90.0 - final_angle) < 15.0);
}

TEST_CASE("TURN pid_wait() does not report interfered on a healthy turn at speed 60 with loosened exits") {
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  chassis.pid_exit_speed_window_set(40);  // 40 ms: even number of ticks, see test_speed_window.cpp
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  chassis.pid_turn_exit_condition_set(300, 8.0, 500, 15.0, 500, 500);
  chassis.pid_turn_set(90, 60);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double final_angle = chassis.drive_imu_get();

  CAPTURE(wait_returned);
  CAPTURE(final_angle);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(90.0 - final_angle) < 15.0);
}
