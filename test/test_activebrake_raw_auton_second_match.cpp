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

// An autonomous that never enters a PID mode: sensor reset (unless told not to), then raw motor output for a second,
// then stop.
void raw_auton(Drive& chassis, bool sensor_reset = true) {
  if (sensor_reset) chassis.drive_sensor_reset();
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

// The field status alone has to be enough: an autonomous that does not even call drive_sensor_reset() leaves the brake
// target wherever the last driver control re-aimed it, and the robot has moved since.
TEST_CASE("a raw-drive_set-only autonomous that never resets the sensors does not lurch in a second match") {
  for (double kp : {2.0, 4.0}) {
    Rig r(kp);
    field(true, false);
    ticks(50);
    field(false, true);
    raw_auton(r.chassis, /*sensor_reset=*/false);
    field(true, false);
    ticks(300);
    field(false, false);
    driver_second(r.chassis, r.sim);
    field(true, false);
    ticks(50);
    field(false, true);
    raw_auton(r.chassis, /*sensor_reset=*/false);
    field(true, false);
    ticks(300);
    field(false, false);
    Outcome o = driver_second(r.chassis, r.sim);
    CAPTURE(kp);
    CHECK(o.moved_in < 0.5);
    CHECK(o.turned_deg < 1.0);
  }
}

// Driver code that runs while the field status says autonomous (a hybrid or skills routine, or a switch left in
// autonomous) must keep its brake. The flag is set once when autonomous begins, and the first opcontrol_* call consumes
// it; setting it on every pass of the period would make every opcontrol_* call re-aim the brake to wherever the robot
// is, so a robot shoved 3 in would never be pulled back.
TEST_CASE("control: driver code run while the status is autonomous still pulls a shoved robot back") {
  for (double kp : {2.0, 4.0}) {
    Rig r(kp);
    field(false, true);
    ticks(20);
    r.chassis.opcontrol_arcade_standard(ez::SPLIT);  // consumes the flag set when autonomous began
    ticks(20);
    r.sim.displace(3.0);
    double before = r.sim.left().position_in;
    for (int i = 0; i < 100; i++) {
      r.chassis.opcontrol_arcade_standard(ez::SPLIT);
      pros::delay(util::DELAY_TIME);
    }
    CAPTURE(kp);
    CHECK(before - r.sim.left().position_in > 0.3);  // pulled back toward where it was, as on dev
  }
}

// A practice run whose autonomous never calls drive_sensor_reset() and only uses raw drive_set: nothing but drive_set itself
// says the robot was moved by something other than the brake. (Motors written directly, bypassing drive_set and every PID
// mode, are not covered: a practice run blocks the driver loop exactly like a macro does, and re-aiming after every pause
// would let a steady push walk a robot downhill.)
TEST_CASE("a raw-drive_set-only practice run that never resets the sensors does not lurch after driver control") {
  for (double kp : {2.0, 4.0}) {
    Rig r(kp);
    driver_second(r.chassis, r.sim);  // driver control runs once first, which clears the flag
    raw_auton(r.chassis, /*sensor_reset=*/false);
    Outcome o = driver_second(r.chassis, r.sim);
    CAPTURE(kp);
    CAPTURE(o.moved_in);
    CHECK(o.moved_in < 0.5);
    CHECK(o.turned_deg < 1.0);
  }
}

// Driver control's own motor output goes through drive_set, so whatever marks a raw drive_set must not mark that one: a
// robot shoved 3 in in plain driver control, with no autonomous anywhere, is still pulled back.
TEST_CASE("control: plain driver control still pulls a shoved robot back") {
  for (double kp : {2.0, 4.0}) {
    Rig r(kp);
    driver_second(r.chassis, r.sim);
    r.sim.displace(3.0);
    double before = r.sim.left().position_in;
    for (int i = 0; i < 100; i++) {
      r.chassis.opcontrol_arcade_standard(ez::SPLIT);
      pros::delay(util::DELAY_TIME);
    }
    CAPTURE(kp);
    CHECK(before - r.sim.left().position_in > 0.3);
  }
}

// A defensive drive_set(0, 0) in the driver loop (or from a background task) does not move the robot, so it must not mark the
// brake's target as stale: a loop that runs it before opcontrol_* on every iteration still has to pull a shoved robot back.
TEST_CASE("control: drive_set(0, 0) every driver loop before opcontrol_* does not stop the brake pulling a shoved robot back") {
  for (double kp : {2.0, 4.0}) {
    Rig r(kp);
    driver_second(r.chassis, r.sim);
    r.sim.displace(3.0);
    double before = r.sim.left().position_in;
    for (int i = 0; i < 100; i++) {
      r.chassis.drive_set(0, 0);
      r.chassis.opcontrol_arcade_standard(ez::SPLIT);
      pros::delay(util::DELAY_TIME);
    }
    CAPTURE(kp);
    CHECK(before - r.sim.left().position_in > 0.3);
  }
}
