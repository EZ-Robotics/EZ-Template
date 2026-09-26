// DRIVE's pid_wait() progress backstop (SingleStuckWatch, added so a sustained disturbance that
// never reads as fully stopped and never draws over current -- a continuous spin, a defender holding
// the robot, sensor jitter under contact -- doesn't hang the wait forever) uses each side's own
// small_error as its progress "step", checked over a window taken from velocity_exit_time. At the
// shipped defaults that's 1 in over 500 ms -- a floor of roughly 2 in/s. A real, healthy drivetrain
// that cruises slower than that for a sustained stretch -- not decaying into a nearby target, a
// genuinely uniform crawl, the shape a low commanded speed on a high-resistance drivetrain produces
// for most of a long leg -- gets read as making no progress and is falsely reported stuck
// (interfered=true), stopping the wait well short of where the robot was actually, healthily still
// headed. The robot itself is never actually stopped: its own drive task keeps running and keeps
// driving toward the target after the wait gives up on it, which this test also confirms directly by
// continuing to tick the simulation past the point pid_wait() returned.
//
// This reproduces from the default constructor alone, with no exit-condition setter of any kind: the
// small_error branch of the progress-step calculation (exit_conditions.cpp's stuck_step(): "if
// small_error > 0, use it as the step") is what small_error is at the shipped default (1 in), not an
// edge case. A related, narrower case -- small_error deliberately set to 0, which switches that same
// helper onto its OTHER, velocity-derived fallback step -- was already found and fixed; this is the
// same mechanism, but at the ordinary default constant, which was never checked afterward.
//
// Uses the suite's own dedicated physics sim (sim_physics.hpp) and its already-validated
// "sticky_high_friction" archetype (higher rolling resistance, matching a real high-traction
// drivetrain) rather than a hand-picked cruise number -- an existing odom straight-leg test in this
// suite already measured this exact archetype cruising at about 1.34 in/s at speed 15, driving under
// odom instead of plain DRIVE. Here, on a plain DRIVE motion, that speed's own uniform middle
// stretch -- not a decaying final approach -- is exactly what SingleStuckWatch's floor can catch.
#include <cmath>

#include "doctest.h"

#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
Drive make_chassis(const sim::SimArchetype& a) {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm, 1.0);
}

// Runs `wait`, letting the fake clock's on_delay hook (installed by the caller's own SimRobot)
// keep advancing physics every pass, capped at `max_ticks` so a genuine hang fails this test
// explicitly instead of freezing the whole suite.
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

TEST_CASE("DRIVE pid_wait() falsely reports interfered on a slow, healthy, shipped-default cruise, and the robot keeps driving anyway") {
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  // Shipped defaults, untouched: 90 ms/1 in small, 250 ms/3 in big, 500 ms velocity, 500 ms current.
  double leg_length_in = 30.0;
  int speed = 15;  // low end of a realistic commanded-speed range; see the file header for why
  chassis.pid_drive_set(leg_length_in, speed);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000 /* 60 s cap */);
  double pos_when_wait_returned = chassis.drive_sensor_left();
  bool interfered_when_wait_returned = chassis.interfered;

  // The wait is done, but Drive's own background drive task is not -- it keeps running and keeps
  // driving toward the same target regardless of what pid_wait() decided. Ticking the simulation
  // further, without calling pid_wait() again, shows whether the robot genuinely stopped making
  // progress (a real stall, where this loop's own cap would fail it) or was simply still closing in
  // when the wait gave up on it.
  run_capped([&] { for (int i = 0; i < 3000; i++) pros::delay(10); }, /*max_ticks=*/6000);
  double pos_after_more_time = chassis.drive_sensor_left();

  CAPTURE(wait_returned);
  CAPTURE(pos_when_wait_returned);
  CAPTURE(interfered_when_wait_returned);
  CAPTURE(pos_after_more_time);
  REQUIRE(wait_returned);

  // The actual bug: a healthy cruise (nothing in this scenario ever stops the robot) reported
  // interfered=true.
  CHECK_FALSE(interfered_when_wait_returned);
  // Proof the robot was never really stuck: left alone, it keeps closing in and reaches the target
  // it was falsely reported to have given up on.
  CHECK(std::fabs(leg_length_in - pos_after_more_time) < 3.0);
}

TEST_CASE("control: the same drivetrain at a higher commanded speed is not falsely flagged stuck") {
  // Same archetype and leg, only the commanded speed changes -- ties the false report above to how
  // slow the resulting cruise actually is, not to some other property of this scenario. At this
  // higher speed the same drivetrain cruises well above the floor and finishes cleanly.
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  double leg_length_in = 30.0;
  int speed = 20;
  chassis.pid_drive_set(leg_length_in, speed);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double final_pos = chassis.drive_sensor_left();

  CAPTURE(wait_returned);
  CAPTURE(final_pos);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(leg_length_in - final_pos) < 3.0);
}
