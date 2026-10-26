// Active brake at the start of driver control after a real field-control autonomous.
//
// ez_auto_task() switches the drive to DISABLE when the robot is disabled or autonomous ends, and it recomputed
// util::AUTON_RAN = (mode != DISABLE) on every pass. During the disabled gap every match has between autonomous
// and driver that set the flag back to false, so opcontrol_drive_sensors_reset() -- the only thing that re-aims
// the active brake after autonomous, and only while the flag is true -- never fired. The brake target stayed at 0
// (left there by drive_sensor_reset() at the top of autonomous()), and with the sticks released the first driver
// pass commanded kp * (0 - distance driven in autonomous).
//
// Sequence, as run on a field: disabled 0.5 s, the template autonomous() (sensor reset, HOLD, drive 36 in, turn 90,
// drive -12 in), disabled 3 s, driver with the sticks released. light_fast, no sensor noise, the real
// ez_auto_task() on every tick (the sim's own task loop never runs the disable/AUTON_RAN code).
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

struct Outcome {
  double moved_in;       // largest wheel travel from where driver control began, over the first second
  double turned_deg;     // largest heading change over the same second
  double first_pass_mv;  // largest |voltage| commanded on the first driver pass
};

void field(bool disabled, bool autonomous) {
  test_stub::g_competition.disabled = disabled;
  test_stub::g_competition.autonomous = autonomous;
}

void ticks(int n) {
  for (int i = 0; i < n; i++) pros::delay(util::DELAY_TIME);
}

// A stick script: the value each analog channel reads on every driver tick. Default: everything released.
struct Sticks {
  std::int32_t ly = 0, ry = 0, rx = 0;
};

void drive_one(Drive& chassis, Style style) {
  if (style == Style::Arcade)
    chassis.opcontrol_arcade_standard(ez::SPLIT);
  else
    chassis.opcontrol_tank();
}

// full_match: false runs autonomous from opcontrol (B+DOWN practice run): no field status change at all.
Outcome run(double brake_kp, Style style, bool full_match, int autons = 1) {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  sim.use_real_auto_task(true);
  chassis.opcontrol_drive_activebrake_set(brake_kp);

  if (full_match) {
    field(true, false);
    ticks(50);  // disabled 0.5 s
  }
  for (int n = 0; n < autons; n++) {
    if (full_match) field(false, true);
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.pid_drive_set(36_in, 110);
    chassis.pid_wait();
    chassis.pid_turn_set(90_deg, 90);
    chassis.pid_wait();
    chassis.pid_drive_set(-12_in, 110);
    chassis.pid_wait();
    if (full_match) {
      field(true, false);
      ticks(300);  // disabled 3 s
    }
  }
  if (full_match) field(false, false);  // driver control begins

  double l0 = sim.left().position_in, r0 = sim.right().position_in, h0 = sim.heading_deg();
  Outcome o{0, 0, 0};
  for (int i = 0; i < 100; i++) {
    drive_one(chassis, style);
    if (i == 0) {
      for (auto& m : chassis.left_motors) o.first_pass_mv = std::fmax(o.first_pass_mv, std::fabs(m.fake().voltage));
      for (auto& m : chassis.right_motors) o.first_pass_mv = std::fmax(o.first_pass_mv, std::fabs(m.fake().voltage));
    }
    pros::delay(util::DELAY_TIME);
    o.moved_in = std::fmax(o.moved_in, std::fmax(std::fabs(sim.left().position_in - l0), std::fabs(sim.right().position_in - r0)));
    o.turned_deg = std::fmax(o.turned_deg, std::fabs(sim.heading_deg() - h0));
  }
  return o;
}
}  // namespace

TEST_CASE("active brake at the start of driver control: a full field match does not lurch (arcade and tank)") {
  for (double kp : {2.0, 4.0}) {
    for (Style s : {Style::Arcade, Style::Tank}) {
      Outcome o = run(kp, s, /*full_match=*/true);
      CAPTURE(kp);
      CAPTURE(s == Style::Arcade);
      CAPTURE(o.moved_in);
      CAPTURE(o.turned_deg);
      CAPTURE(o.first_pass_mv);
      CHECK(o.moved_in < 0.5);
      CHECK(o.turned_deg < 1.0);
    }
  }
}

TEST_CASE("active brake at the start of driver control: the first driver pass does not command the old auton distance") {
  Outcome o = run(4.0, Style::Arcade, /*full_match=*/true);
  CAPTURE(o.first_pass_mv);
  CHECK(o.first_pass_mv < 1000.0);  // start commit: -12000 mV on the left side
}

TEST_CASE("control: active brake off never lurches") {
  Outcome o = run(0.0, Style::Arcade, /*full_match=*/true);
  CHECK(o.moved_in < 0.5);
  CHECK(o.turned_deg < 1.0);
}

TEST_CASE("control: a practice run (autonomous called from opcontrol) does not lurch") {
  for (double kp : {2.0, 4.0}) {
    Outcome o = run(kp, Style::Arcade, /*full_match=*/false);
    CAPTURE(kp);
    CHECK(o.moved_in < 0.5);
    CHECK(o.turned_deg < 1.0);
  }
}

TEST_CASE("control: two autons back to back with a disabled gap, then driver, does not lurch") {
  Outcome o = run(2.0, Style::Arcade, /*full_match=*/true, /*autons=*/2);
  CHECK(o.moved_in < 0.5);
  CHECK(o.turned_deg < 1.0);
}

// Regression guard: the fix must only change the FIRST re-aim of the brake target after autonomous, nothing else
// about driver control. A scripted stick sequence in driver (rest, full forward, release, full reverse, release,
// turn in place, release) with active brake on, run through the real ez_auto_task() and the sim, hashed over every
// motor voltage on every tick. The golden was captured on the start commit (a107ac8) of this branch; both the arcade
// and tank scripts must reproduce it exactly.
namespace {
// Captured on a107ac8 (arcade and tank script the same motion, so they hash the same).
std::uint64_t golden_hash(double kp, Style) { return kp == 2.0 ? 6792602578573120267ull : 16022465462488834807ull; }
std::uint64_t hash_script(double kp, Style style) {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  sim.use_real_auto_task(true);
  chassis.opcontrol_drive_activebrake_set(kp);

  auto set_sticks = [&](int fwd, int turn) {
    if (style == Style::Arcade) {
      master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = fwd;
      master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_X] = turn;
    } else {
      master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = fwd + turn;
      master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = fwd - turn;
    }
  };

  std::uint64_t h = 1469598103934665603ull;
  auto mix = [&](double v) {
    long long q = std::llround(v);
    h = (h ^ (std::uint64_t)q) * 1099511628211ull;
  };
  for (int t = 0; t < 700; t++) {
    int fwd = 0, turn = 0;
    if (t >= 100 && t < 200)
      fwd = 127;
    else if (t >= 300 && t < 400)
      fwd = -127;
    else if (t >= 500 && t < 600)
      turn = 100;
    set_sticks(fwd, turn);
    drive_one(chassis, style);
    for (auto& m : chassis.left_motors) mix(m.fake().voltage);
    for (auto& m : chassis.right_motors) mix(m.fake().voltage);
    pros::delay(util::DELAY_TIME);
  }
  set_sticks(0, 0);
  return h;
}
}  // namespace

TEST_CASE("regression guard: a scripted driver-control stick sequence with active brake on is unchanged") {
  for (double kp : {2.0, 4.0}) {
    for (Style s : {Style::Arcade, Style::Tank}) {
      std::uint64_t h = hash_script(kp, s);
      CAPTURE(kp);
      CAPTURE(s == Style::Arcade);
      CHECK(h == golden_hash(kp, s));
    }
  }
}
