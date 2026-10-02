// Active brake when driver control starts after the robot was disabled and moved by hand.
//
// opcontrol_drive_sensors_reset() re-aims the active brake target only while util::AUTON_RAN is set, and nothing set it
// on a disable. A robot that is driven in the pits, plugged into field control (disabled) and slid into the starting
// corner has its wheels rolled while the motors are limp. When driver control starts with the sticks released, the brake
// target is still wherever the sticks were last released before the disable, so the brake drives the robot back.
//
// light_fast, no sensor noise, the real ez_auto_task() on every tick, and test_stub::g_competition for the field status.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
enum class Style {
  Arcade,
  Tank
};

void field(bool disabled, bool autonomous) {
  test_stub::g_competition.disabled = disabled;
  test_stub::g_competition.autonomous = autonomous;
}

void ticks(int n) {
  for (int i = 0; i < n; i++) pros::delay(util::DELAY_TIME);
}

void drive_one(Drive& chassis, Style style) {
  if (style == Style::Arcade)
    chassis.opcontrol_arcade_standard(ez::SPLIT);
  else
    chassis.opcontrol_tank();
}

void sticks(Style style, int fwd) {
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = fwd;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = style == Style::Tank ? fwd : 0;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_X] = 0;
}

struct Outcome {
  double moved_in = 0;      // largest wheel travel from where driver control began (after any displacement), first second
  double target_error = 0;  // |left brake target - left sensor| after driver control has run for a second
  double target_shift = 0;  // how far the left brake target moved between the release and the end of the run
};

// Drive forward in driver control, release, then (if `disable`) field control disables the robot with the motors zeroed.
// If shift != 0 the robot is slid by `shift` inches in between. Then driver control runs with the sticks released.
Outcome run(double kp, Style style, double shift, bool disable) {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  sim.use_real_auto_task(true);
  chassis.opcontrol_drive_activebrake_set(kp);

  field(false, false);
  sticks(style, 60);
  for (int i = 0; i < 100; i++) {
    drive_one(chassis, style);
    pros::delay(util::DELAY_TIME);
  }
  sticks(style, 0);
  for (int i = 0; i < 50; i++) {
    drive_one(chassis, style);
    pros::delay(util::DELAY_TIME);
  }
  if (disable) {
    field(true, false);
    chassis.drive_set(0, 0);
    ticks(50);
  }
  double target_before = chassis.left_activebrakePID.target;
  if (shift != 0) sim.displace(shift);
  if (disable) field(false, false);

  double l0 = sim.left().position_in, r0 = sim.right().position_in;
  Outcome o;
  for (int i = 0; i < 100; i++) {
    drive_one(chassis, style);
    pros::delay(util::DELAY_TIME);
    o.moved_in = std::fmax(o.moved_in, std::fmax(std::fabs(sim.left().position_in - l0), std::fabs(sim.right().position_in - r0)));
  }
  o.target_shift = std::fabs(chassis.left_activebrakePID.target - target_before);
  o.target_error = std::fabs(chassis.left_activebrakePID.target - chassis.drive_sensor_left());
  return o;
}
}  // namespace

TEST_CASE("active brake after a disable: a robot slid by hand while disabled does not drive itself back") {
  for (double kp : {2.0, 4.0}) {
    for (Style s : {Style::Arcade, Style::Tank}) {
      Outcome o = run(kp, s, -10.0, true);
      CAPTURE(kp);
      CAPTURE(s == Style::Arcade);
      CAPTURE(o.moved_in);
      CHECK(o.moved_in < 0.5);
    }
  }
}

TEST_CASE("control: disabled and enabled again without being moved does not move and keeps the brake on the robot") {
  for (double kp : {2.0, 4.0}) {
    for (Style s : {Style::Arcade, Style::Tank}) {
      Outcome o = run(kp, s, 0.0, true);
      CAPTURE(kp);
      CHECK(o.moved_in < 0.5);
      CHECK(o.target_error < 0.5);
    }
  }
}

TEST_CASE("control: a shove while enabled, with no disable, is still pulled back") {
  for (double kp : {2.0, 4.0}) {
    Outcome o = run(kp, Style::Arcade, -3.0, false);
    CAPTURE(kp);
    CAPTURE(o.moved_in);
    CHECK(o.target_shift < 1e-9);  // never re-aimed, so the brake keeps pulling toward where it was released
  }
}

// Re-aim fires once per enable, never on every pass. After the enable edge a robot that is pushed is pulled back to the
// target the edge set, whether the status says driver control or autonomous (a hybrid or skills routine run from driver).
namespace {
// Runs `n` driver passes with the sticks released.
void driver_passes(Drive& chassis, int n) {
  for (int i = 0; i < n; i++) {
    drive_one(chassis, Style::Arcade);
    pros::delay(util::DELAY_TIME);
  }
}

double target_shift_after_push(bool autonomous_status) {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  sim.use_real_auto_task(true);
  chassis.opcontrol_drive_activebrake_set(4.0);

  sticks(Style::Arcade, 0);
  field(true, false);
  ticks(50);
  field(false, autonomous_status);
  driver_passes(chassis, 30);
  double target_before = chassis.left_activebrakePID.target;
  sim.displace(-3.0);
  driver_passes(chassis, 30);
  return std::fabs(chassis.left_activebrakePID.target - target_before);
}
}  // namespace

TEST_CASE("control: after the enable re-aim, a push is not re-aimed away (driver status and autonomous status)") {
  CHECK(target_shift_after_push(false) < 1e-9);
  CHECK(target_shift_after_push(true) < 1e-9);
}

TEST_CASE("a one-pass disable blip mid-driver re-aims once to where the robot is, and a later push is still pulled back") {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  sim.use_real_auto_task(true);
  chassis.opcontrol_drive_activebrake_set(4.0);

  field(false, false);
  sticks(Style::Arcade, 60);
  driver_passes(chassis, 100);
  sticks(Style::Arcade, 0);
  driver_passes(chassis, 50);
  field(true, false);
  ticks(1);
  field(false, false);
  driver_passes(chassis, 20);
  CHECK(std::fabs(chassis.left_activebrakePID.target - chassis.drive_sensor_left()) < 0.05);
  double target_before = chassis.left_activebrakePID.target;
  sim.displace(-3.0);
  driver_passes(chassis, 20);
  CHECK(std::fabs(chassis.left_activebrakePID.target - target_before) < 1e-9);
}
