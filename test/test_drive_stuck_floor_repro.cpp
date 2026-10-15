// DRIVE's pid_wait() ends on the velocity exit (3.2.2's) when the robot's speed over the last 100 ms is at or under 5 in/s for
// velocity_exit_time.  The exit arms once the robot has moved faster than that, or after 1000 ms (VELOCITY_ARM_FALLBACK) if it never does.
// So a real, healthy drivetrain that cruises slower than 5 in/s for a sustained stretch -- not decaying into a nearby target, a genuinely
// uniform crawl, the shape a low commanded speed on a high-resistance drivetrain produces for most of a long leg -- reads as stopped and
// the wait ends with interfered=true, well short of where the robot was healthily still headed.  The robot itself is never actually
// stopped: its own drive task keeps running and keeps driving toward the target after the wait gives up on it, which this test also
// confirms directly by continuing to tick the simulation past the point pid_wait() returned.
//
// A team that needs a slower drivetrain to survive its own pid_wait() can drop the velocity exit (velocity_exit_time 0) or command a
// higher speed.  This test locks the 5 in/s floor in as a regression guard, not a bug repro: if the velocity exit's speed or arming
// changes, this test needs a deliberate update.
//
// Uses the suite's own dedicated physics sim (sim_physics.hpp) and its already-validated "sticky_high_friction" archetype (higher
// rolling resistance, matching a real high-traction drivetrain) rather than a hand-picked cruise number -- an existing odom
// straight-leg test in this suite already measured this exact archetype cruising at about 1.34 in/s at speed 15, driving under odom
// instead of plain DRIVE.  Here, on a plain DRIVE motion, that speed's own uniform middle stretch is exactly what the velocity exit
// reads as stopped.
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

TEST_CASE("DRIVE pid_wait() reports interfered on a sustained cruise below the 5 in/s velocity exit speed, by design") {
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
  run_capped(
      [&] {
        for (int i = 0; i < 3000; i++) pros::delay(10);
      },
      /*max_ticks=*/6000);
  double pos_after_more_time = chassis.drive_sensor_left();

  CAPTURE(wait_returned);
  CAPTURE(pos_when_wait_returned);
  CAPTURE(interfered_when_wait_returned);
  CAPTURE(pos_after_more_time);
  REQUIRE(wait_returned);

  // The velocity exit doing exactly what it's meant to: a cruise below 5 in/s reads as stopped
  // once the exit has armed and reports interfered=true.
  CHECK(interfered_when_wait_returned);
  // The robot itself was never actually stopped, though -- pid_wait() giving up on watching it is
  // not the same as the drive task giving up on driving it. Left alone, it keeps closing in and
  // reaches the target on its own.
  CHECK(std::fabs(leg_length_in - pos_after_more_time) < 3.0);
}

TEST_CASE("control: the same drivetrain at a higher commanded speed is not falsely flagged stuck") {
  // Same archetype and leg, only the commanded speed changes -- ties the early exit above to how
  // slow the resulting cruise actually is, not to some other property of this scenario.  At this
  // higher speed the same drivetrain cruises above the 5 in/s velocity exit speed and finishes cleanly.
  sim::SimArchetype a = sim::archetype_sticky_high_friction();
  Drive chassis = make_chassis(a);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::NoiseConfig no_noise{/*enabled=*/false, /*seed=*/1};
  sim::SimRobot sim(chassis, a, no_noise);

  double leg_length_in = 30.0;
  int speed = 30;  // speed 20 still cruises under 5 in/s on this archetype and ends early on the velocity exit
  chassis.pid_drive_set(leg_length_in, speed);

  bool wait_returned = run_capped([&] { chassis.pid_wait(); }, /*max_ticks=*/6000);
  double final_pos = chassis.drive_sensor_left();

  CAPTURE(wait_returned);
  CAPTURE(final_pos);
  REQUIRE(wait_returned);
  CHECK_FALSE(chassis.interfered);
  CHECK(std::fabs(leg_length_in - final_pos) < 3.0);
}
