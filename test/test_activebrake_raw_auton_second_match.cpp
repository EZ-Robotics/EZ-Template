// Active brake at the start of driver control after an autonomous that only uses raw drive_set.
//
// util::AUTON_RAN is what tells opcontrol_drive_sensors_reset() to re-aim the active brake targets to where the robot is
// now. ez_auto_task() only set it while a PID mode was running, and a raw drive_set leaves the mode at DISABLE, so an
// autonomous that never used a PID motion never set it. The flag starts true at boot and the first driver-control call
// clears it, so the first match was fine by accident; a second match without a reboot (or a practice run after driver
// control) never set it again. The brake target stayed at 0, left there by drive_sensor_reset() at the top of
// autonomous(), and with the sticks released the first driver pass drove the robot back toward it.
//
// light_fast, no sensor noise, the real ez_auto_task() on every tick. The field status is scripted the way a real
// match goes: disabled, autonomous, disabled gap, driver.
#include <cmath>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
struct Outcome {
  double moved_in;    // largest wheel travel from where driver control began, over the first second
  double turned_deg;  // largest heading change over the same second
};

void field(bool disabled, bool autonomous) {
  test_stub::g_competition.disabled = disabled;
  test_stub::g_competition.autonomous = autonomous;
}

void ticks(int n) {
  for (int i = 0; i < n; i++) pros::delay(util::DELAY_TIME);
}

// An autonomous that never enters a PID mode: sensor reset, then raw motor output for a second, then stop.
void raw_auton(Drive& chassis) {
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  chassis.drive_set(80, 80);
  ticks(100);
  chassis.drive_set(0, 0);
  ticks(50);
}

// A second of driver control with the sticks released, and how far the robot moved in it.
Outcome driver_second(Drive& chassis, sim::SimRobot& sim) {
  double l0 = sim.left().position_in, r0 = sim.right().position_in, h0 = sim.heading_deg();
  Outcome o{0, 0};
  for (int i = 0; i < 100; i++) {
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    pros::delay(util::DELAY_TIME);
    o.moved_in = std::fmax(o.moved_in, std::fmax(std::fabs(sim.left().position_in - l0), std::fabs(sim.right().position_in - r0)));
    o.turned_deg = std::fmax(o.turned_deg, std::fabs(sim.heading_deg() - h0));
  }
  return o;
}

struct Rig {
  sim::SimArchetype a = sim::archetype_light_fast();
  Drive chassis;
  sim::SimRobot sim;
  explicit Rig(double brake_kp) : chassis(make(a)), sim(chassis, a, sim::NoiseConfig{false, 1}) {
    util::AUTON_RAN = true;  // a fresh boot; it is a global, so an earlier test case would otherwise leave it cleared
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    sim.use_real_auto_task(true);
    chassis.opcontrol_drive_activebrake_set(brake_kp);
  }
  static Drive make(const sim::SimArchetype& arch) {
    test_stub::reset_all();
    return Drive({1, -2}, {-3, 4}, 5, arch.wheel_diameter_in, arch.cartridge_rpm);
  }
};
}  // namespace

TEST_CASE("a raw-drive_set-only autonomous in a second match does not lurch at the start of driver control") {
  for (double kp : {2.0, 4.0}) {
    Rig r(kp);
    // Match 1: the flag starts true at boot, so this one was always fine.
    field(true, false);
    ticks(50);
    field(false, true);
    raw_auton(r.chassis);
    field(true, false);
    ticks(300);
    field(false, false);
    Outcome first = driver_second(r.chassis, r.sim);
    // Match 2, no reboot: the first driver-control call above cleared the flag.
    field(true, false);
    ticks(50);
    field(false, true);
    raw_auton(r.chassis);
    field(true, false);
    ticks(300);
    field(false, false);
    Outcome second = driver_second(r.chassis, r.sim);
    CAPTURE(kp);
    CAPTURE(first.moved_in);
    CAPTURE(second.moved_in);
    CAPTURE(second.turned_deg);
    CHECK(first.moved_in < 0.5);
    CHECK(second.moved_in < 0.5);
    CHECK(second.turned_deg < 1.0);
  }
}

TEST_CASE("a raw-drive_set-only practice run (autonomous called from opcontrol) after driver control does not lurch") {
  for (double kp : {2.0, 4.0}) {
    Rig r(kp);
    // Driver control runs once first, which clears the flag; then autonomous() is called from opcontrol (B+DOWN).
    Outcome warmup = driver_second(r.chassis, r.sim);
    raw_auton(r.chassis);
    Outcome o = driver_second(r.chassis, r.sim);
    CAPTURE(kp);
    CAPTURE(warmup.moved_in);
    CAPTURE(o.moved_in);
    CHECK(o.moved_in < 0.5);
    CHECK(o.turned_deg < 1.0);
  }
}

TEST_CASE("control: active brake off never lurches after a raw-only autonomous") {
  Rig r(0.0);
  field(true, false);
  ticks(50);
  field(false, true);
  raw_auton(r.chassis);
  field(true, false);
  ticks(300);
  field(false, false);
  driver_second(r.chassis, r.sim);
  field(true, false);
  ticks(50);
  field(false, true);
  raw_auton(r.chassis);
  field(true, false);
  ticks(300);
  field(false, false);
  Outcome o = driver_second(r.chassis, r.sim);
  CHECK(o.moved_in < 0.5);
  CHECK(o.turned_deg < 1.0);
}
