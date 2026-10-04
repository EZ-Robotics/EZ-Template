// An rpm change at runtime (a shifting transmission) must not move the sensors.
//
// drive_rpm_set() changes how many encoder ticks an inch is worth. The raw count since the last reset
// has not moved, so what the robot has traveled in inches must not move either: odom, the drive PID's start and target,
// the active brake and every wait read inches from drive_sensor_left()/right(), and all of them used to see the robot
// jump by (old ticks per inch / new ticks per inch) - 1 of its whole distance since the last reset.
// (3.x and the betas also had drive_ratio_set(); it was removed in 4.0, and a shift in a ratio r is drive_rpm_set(rpm / r).)
//
// The sim's shift_gearing() is the other half of a real shift: the wheels really change speed and the encoders keep
// counting from where they were, at a new number of ticks per wheel inch. A test that shifts mid-motion calls both, at
// the same instant, the way a shifter and its code would.
#include <cmath>
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
struct Rig {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis;
  sim::SimRobot sim;
  Rig() : chassis(make()), sim(chassis, a, sim::NoiseConfig{false, 1}) {
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
  }
  static Drive make() {
    test_stub::reset_all();
    auto a = sim::archetype_light_fast();
    return Drive({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  }
  double travel() const { return (sim.left().position_in + sim.right().position_in) / 2.0; }
  void idle(int ticks) {
    for (int i = 0; i < ticks; i++) pros::delay(ez::util::DELAY_TIME);
  }
};

// Runs `wait`, capped so a genuine hang fails the test instead of freezing the suite.
bool run_capped(const std::function<void()>& wait, int max_ticks = 4000) {
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

// --- The audit's repro: nothing moved, so nothing may change ---------------------------------------------------------

TEST_CASE("drive_rpm_set with the robot still does not move odom or the sensors") {
  // 300, 480 and 1200 are what the ratios 2.0, 1.25 and 0.5 gave on a 600 rpm drive in 3.x and the betas
  for (double rpm : {300.0, 450.0, 480.0, 900.0, 1200.0}) {
    Rig r;
    r.chassis.pid_drive_set(24_in, 110);
    r.chassis.pid_wait();
    r.idle(5);
    r.chassis.ez_tracking_task();
    double y0 = r.chassis.odom_y_get();
    double l0 = r.chassis.drive_sensor_left();
    double r0 = r.chassis.drive_sensor_right();
    REQUIRE(l0 > 20.0);

    r.chassis.drive_rpm_set(rpm);
    for (int i = 0; i < 3; i++) r.chassis.ez_tracking_task();

    CAPTURE(rpm);
    CHECK(std::fabs(r.chassis.odom_y_get() - y0) < 0.01);
    CHECK(std::fabs(r.chassis.drive_sensor_left() - l0) < 0.01);
    CHECK(std::fabs(r.chassis.drive_sensor_right() - r0) < 0.01);
  }
}

// --- A real shift in the middle of a motion ---------------------------------------------------------------------------

TEST_CASE("a shift in the middle of pid_drive_set(48) still ends the drive 48 inches from where it started") {
  for (double new_rpm : {300.0, 450.0, 900.0}) {
    Rig r;
    bool shifted = false;
    r.sim.before_pass = [&](int) {
      if (shifted || r.travel() < 20.0) return;
      shifted = true;
      r.sim.shift_gearing(new_rpm);
      r.chassis.drive_rpm_set(new_rpm);
    };
    r.chassis.pid_drive_set(48_in, 110);
    bool returned = run_capped([&] { r.chassis.pid_wait(); });

    CAPTURE(new_rpm);
    CAPTURE(r.travel());
    CHECK(shifted);
    CHECK(returned);
    CHECK(std::fabs(r.travel() - 48.0) <= 1.0);  // the drive's small_error
    CHECK_FALSE(r.chassis.interfered);
  }
}

TEST_CASE("a shift in the middle of pid_odom_set to (0, 48) leaves odom within an inch of where the robot really is") {
  for (double new_rpm : {300.0, 450.0, 900.0}) {
    Rig r;
    r.chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    r.idle(5);
    bool shifted = false;
    r.sim.before_pass = [&](int) {
      if (shifted || r.travel() < 20.0) return;
      shifted = true;
      r.sim.shift_gearing(new_rpm);
      r.chassis.drive_rpm_set(new_rpm);
    };
    r.chassis.pid_odom_set({{0_in, 48_in}, fwd, 110});
    bool returned = run_capped([&] { r.chassis.pid_wait(); });
    r.idle(3);

    CAPTURE(new_rpm);
    pose p = r.chassis.odom_pose_get();
    CHECK(shifted);
    CHECK(returned);
    CHECK(std::fabs(p.y - r.travel()) <= 1.0);
    CHECK(std::fabs(p.x) <= 1.0);
  }
}

// --- What could go wrong with the fix ---------------------------------------------------------------------------------

TEST_CASE("a failed encoder read at the moment of the change cannot poison the offset") {
  Rig r;
  r.chassis.pid_drive_set(24_in, 110);
  r.chassis.pid_wait();
  r.idle(5);
  double l0 = r.chassis.drive_sensor_left();  // a good read, remembered as the last good one
  double tpi0 = r.chassis.drive_tick_per_inch();
  double raw0 = r.chassis.drive_sensor_left_raw();

  r.chassis.left_motors.front().fake().disconnected = true;  // the read fails (PROS_ERR_F) exactly when the shift happens
  r.chassis.drive_rpm_set(300.0);
  CHECK(std::isfinite(r.chassis.drive_sensor_left()));
  CHECK(std::fabs(r.chassis.drive_sensor_left() - l0) < 0.01);

  // the fault clears and the robot has rolled 100 ticks: the reading continues from l0 at the NEW scale
  r.chassis.left_motors.front().fake().disconnected = false;
  r.chassis.left_motors.front().fake().position = raw0 + 100.0;
  CHECK(r.chassis.drive_sensor_left() == doctest::Approx(l0 + 100.0 / r.chassis.drive_tick_per_inch()).epsilon(1e-9));
  CHECK(r.chassis.drive_tick_per_inch() == doctest::Approx(tpi0 * 2.0));
}

TEST_CASE("a run of rpm changes each leaves the reading where it was, and later reads use the final scale") {
  Rig r;
  r.chassis.pid_drive_set(24_in, 110);
  r.chassis.pid_wait();
  r.idle(5);
  double l0 = r.chassis.drive_sensor_left();
  double raw0 = r.chassis.drive_sensor_left_raw();

  r.chassis.drive_rpm_set(300.0);
  CHECK(r.chassis.drive_sensor_left() == doctest::Approx(l0).epsilon(1e-9));
  r.chassis.drive_rpm_set(450.0);
  CHECK(r.chassis.drive_sensor_left() == doctest::Approx(l0).epsilon(1e-9));
  r.chassis.drive_rpm_set(450.0);  // no change: no effect
  CHECK(r.chassis.drive_sensor_left() == doctest::Approx(l0).epsilon(1e-9));
  r.chassis.drive_rpm_set(800.0);
  CHECK(r.chassis.drive_sensor_left() == doctest::Approx(l0).epsilon(1e-9));

  r.chassis.left_motors.front().fake().position = raw0 + 500.0;
  CHECK(r.chassis.drive_sensor_left() == doctest::Approx(l0 + 500.0 / r.chassis.drive_tick_per_inch()).epsilon(1e-9));
}

// --- Controls: nothing else changes -----------------------------------------------------------------------------------

TEST_CASE("with no shift every sensor read over a whole motion is exactly raw / ticks per inch, as before") {
  Rig r;
  int checked = 0, mismatched = 0;
  r.sim.after_pass = [&](int) {
    double tpi = r.chassis.drive_tick_per_inch();
    if (r.chassis.drive_sensor_left() != r.chassis.drive_sensor_left_raw() / tpi) mismatched++;
    if (r.chassis.drive_sensor_right() != r.chassis.drive_sensor_right_raw() / tpi) mismatched++;
    checked++;
  };
  r.chassis.pid_drive_set(36_in, 110);
  r.chassis.pid_wait();
  r.chassis.pid_turn_set(90_deg, 90);
  r.chassis.pid_wait();
  r.chassis.pid_drive_set(-12_in, 110);
  r.chassis.pid_wait();
  CHECK(checked > 50);
  CHECK(mismatched == 0);
}

TEST_CASE("setting the rpm to what it already is changes nothing, exactly") {
  Rig r;
  r.chassis.pid_drive_set(24_in, 110);
  r.chassis.pid_wait();
  double l0 = r.chassis.drive_sensor_left();
  r.chassis.drive_rpm_set(r.chassis.drive_rpm_get());
  CHECK(r.chassis.drive_sensor_left() == l0);
}

TEST_CASE("after a shift, drive_sensor_reset reads 0 and later reads scale with the new rpm only") {
  Rig r;
  r.chassis.pid_drive_set(24_in, 110);
  r.chassis.pid_wait();
  r.idle(5);

  r.sim.shift_gearing(300.0);
  r.chassis.drive_rpm_set(300.0);
  r.chassis.drive_sensor_reset();
  r.sim.tare_encoders();
  CHECK(r.chassis.drive_sensor_left() == doctest::Approx(0.0).epsilon(1e-9));
  CHECK(r.chassis.drive_sensor_right() == doctest::Approx(0.0).epsilon(1e-9));

  r.sim.displace(10.0);
  // the raw count is a whole number of ticks, about 0.0085 in each at the new scale
  CHECK(std::fabs(r.chassis.drive_sensor_left() - 10.0) < 0.02);
  CHECK(std::fabs(r.chassis.drive_sensor_right() - 10.0) < 0.02);
}

TEST_CASE("drive_sensor_reset after a shift also zeroes the offsets, so a second shift starts from zero") {
  Rig r;
  r.chassis.pid_drive_set(24_in, 110);
  r.chassis.pid_wait();
  r.chassis.drive_rpm_set(300.0);
  r.chassis.drive_sensor_reset();
  CHECK(r.chassis.drive_sensor_left() == doctest::Approx(0.0).epsilon(1e-9));
  r.chassis.drive_rpm_set(600.0);
  CHECK(r.chassis.drive_sensor_left() == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("with tracking wheels set, drive_rpm_set changes nothing about the readings") {
  Rig r;
  tracking_wheel left_tracker(1, 3.25, 3.5);
  tracking_wheel right_tracker(2, 3.25, 3.5);
  r.chassis.odom_tracker_left_set(&left_tracker);
  r.chassis.odom_tracker_right_set(&right_tracker);
  left_tracker.smart_encoder.fake_position = 3000;
  right_tracker.smart_encoder.fake_position = 3100;

  double l0 = r.chassis.drive_sensor_left();
  double r0 = r.chassis.drive_sensor_right();
  double tpi0 = r.chassis.drive_tick_per_inch();
  REQUIRE(l0 != 0.0);

  r.chassis.drive_rpm_set(1200.0);
  r.chassis.drive_rpm_set(300.0);
  CHECK(r.chassis.drive_sensor_left() == l0);
  CHECK(r.chassis.drive_sensor_right() == r0);
  CHECK(r.chassis.drive_tick_per_inch() == tpi0);
}

// --- Found by the verification round: a degenerate value must not poison the carried inches --------------------------------------

TEST_CASE("a bad rpm (0, negative, NaN or infinite) followed by a good one leaves the sensors finite and where the raw count says") {
  // 0 and infinity give infinite and zero ticks per inch, a negative rpm gives backwards ones, NaN gives NaN. None of them is
  // a real scale to carry the inches across, and none may leave an offset that the next good rpm cannot recover from.
  for (double bad : {0.0, -600.0, std::nan(""), HUGE_VAL}) {
    Rig r;
    r.chassis.pid_drive_set(24_in, 110);
    r.chassis.pid_wait();
    r.idle(5);
    double l0 = r.chassis.drive_sensor_left();
    double r0 = r.chassis.drive_sensor_right();

    r.chassis.drive_rpm_set(bad);
    r.chassis.drive_rpm_set(600.0);

    CAPTURE(bad);
    CHECK(std::isfinite(r.chassis.drive_sensor_left()));
    CHECK(std::isfinite(r.chassis.drive_sensor_right()));
    CHECK(std::fabs(r.chassis.drive_sensor_left() - l0) < 0.05);
    CHECK(std::fabs(r.chassis.drive_sensor_right() - r0) < 0.05);
  }
}

TEST_CASE("the active brake holds a still robot across a shift, with the sticks released") {
  for (double kp : {2.0, 4.0}) {
    Rig r;
    r.sim.use_real_auto_task(true);
    r.chassis.opcontrol_drive_activebrake_set(kp);
    r.chassis.pid_drive_set(24_in, 110);
    r.chassis.pid_wait();
    for (int i = 0; i < 50; i++) {
      r.chassis.opcontrol_arcade_standard(ez::SPLIT);
      pros::delay(ez::util::DELAY_TIME);
    }
    double p0 = r.sim.left().position_in;
    r.sim.shift_gearing(300.0);
    r.chassis.drive_rpm_set(300.0);
    double worst = 0;
    for (int i = 0; i < 100; i++) {
      r.chassis.opcontrol_arcade_standard(ez::SPLIT);
      pros::delay(ez::util::DELAY_TIME);
      worst = std::fmax(worst, std::fabs(r.sim.left().position_in - p0));
    }
    CAPTURE(kp);
    CHECK(worst < 0.5);
  }
}
