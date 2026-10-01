// Every opcontrol mode goes through the joystick deadzone and re-aims the active brake after an autonomous.
//
// The deadzone (default 3) and the first-pass brake re-aim (opcontrol_drive_sensors_reset(), which only does anything
// while util::AUTON_RAN is set) are written out by hand in each mode, so each mode can lose one without any other mode
// noticing. test_joystick_default_deadzone.cpp only drives tank, and test_activebrake_driver_start.cpp only tank and
// arcade standard. Here every mode gets both, split and single sticks alike, so a mode that skips either shows up as
// itself. Tank drives only.
#include <array>
#include <cmath>
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
struct Mode {
  const char* name;
  std::function<void(Drive&)> drive;
};

const Mode kModes[] = {
    {"tank", [](Drive& c) { c.opcontrol_tank(); }},
    {"arcade standard split", [](Drive& c) { c.opcontrol_arcade_standard(ez::SPLIT); }},
    {"arcade standard single", [](Drive& c) { c.opcontrol_arcade_standard(ez::SINGLE); }},
    {"arcade flipped split", [](Drive& c) { c.opcontrol_arcade_flipped(ez::SPLIT); }},
    {"arcade flipped single", [](Drive& c) { c.opcontrol_arcade_flipped(ez::SINGLE); }},
    {"curvature standard split", [](Drive& c) { c.opcontrol_arcade_curvature_standard(ez::SPLIT); }},
    {"curvature standard single", [](Drive& c) { c.opcontrol_arcade_curvature_standard(ez::SINGLE); }},
    {"curvature flipped split", [](Drive& c) { c.opcontrol_arcade_curvature_flipped(ez::SPLIT); }},
    {"curvature flipped single", [](Drive& c) { c.opcontrol_arcade_curvature_flipped(ez::SINGLE); }},
};

// Every axis any mode reads, so each of a mode's stick reads is covered whichever axes it picks.
void all_sticks(int ly, int lx, int ry, int rx) {
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = ly;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_X] = lx;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = ry;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_X] = rx;
}

double max_mv(Drive& c) {
  double m = 0;
  for (auto& x : c.left_motors) m = std::fmax(m, std::fabs(x.fake().voltage));
  for (auto& x : c.right_motors) m = std::fmax(m, std::fabs(x.fake().voltage));
  return m;
}
}  // namespace

TEST_CASE("every opcontrol mode reads sticks at 1 and 2 as released") {
  for (const Mode& m : kModes) {
    for (auto sticks : {std::array<int, 4>{2, 2, 2, 2}, {-2, -2, -2, -2}, {1, -1, 1, -1}, {2, -2, -1, 2}, {-1, 2, 2, -2}}) {
      test_stub::reset_all();
      Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
      chassis.pid_print_toggle(false);
      test_stub::g_clock.now_ms = 5000;
      all_sticks(sticks[0], sticks[1], sticks[2], sticks[3]);
      m.drive(chassis);
      CAPTURE(m.name);
      CAPTURE(sticks[0]);
      CAPTURE(sticks[1]);
      CAPTURE(sticks[2]);
      CAPTURE(sticks[3]);
      CHECK(max_mv(chassis) == doctest::Approx(0.0));
    }
    all_sticks(0, 0, 0, 0);
  }
}

TEST_CASE("control: every opcontrol mode still moves at a stick of 3") {
  for (const Mode& m : kModes) {
    test_stub::reset_all();
    Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
    chassis.pid_print_toggle(false);
    test_stub::g_clock.now_ms = 5000;
    all_sticks(3, 3, 3, 3);
    m.drive(chassis);
    CAPTURE(m.name);
    CHECK(max_mv(chassis) > 100.0);
    all_sticks(0, 0, 0, 0);
  }
}

TEST_CASE("every opcontrol mode with a stick resting at 1 or 2 and active brake on pulls a pushed robot back") {
  for (const Mode& m : kModes) {
    test_stub::reset_all();
    auto a = sim::archetype_light_fast();
    Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
    chassis.opcontrol_drive_activebrake_set(2.0);
    all_sticks(2, -2, 1, 2);
    for (int t = 0; t < 30; t++) {
      m.drive(chassis);
      pros::delay(util::DELAY_TIME);
    }
    sim.displace(3.0);
    m.drive(chassis);
    CAPTURE(m.name);
    // Braking against the push: negative on both sides, not the stick's own few hundred mV forward.
    CHECK(chassis.left_motors[0].fake().voltage < -300.0);
    CHECK(chassis.right_motors[0].fake().voltage < -300.0);
    all_sticks(0, 0, 0, 0);
  }
}

// After an autonomous that used PID the brake target still holds where the drive was when autonomous began. The first
// driver pass has to re-aim it at where the robot is now, or it drives back to the start of autonomous.
TEST_CASE("every opcontrol mode re-aims the active brake on its first pass after a PID autonomous") {
  for (const Mode& m : kModes) {
    test_stub::reset_all();
    auto a = sim::archetype_light_fast();
    Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
    sim.use_real_auto_task(true);
    chassis.opcontrol_drive_activebrake_set(4.0);
    all_sticks(0, 0, 0, 0);

    chassis.drive_sensor_reset();
    chassis.pid_drive_set(24_in, 110);
    chassis.pid_wait();
    REQUIRE(util::AUTON_RAN);  // set by the auto task while a PID motion ran
    REQUIRE(sim.left().position_in > 20.0);

    m.drive(chassis);
    CAPTURE(m.name);
    CHECK(max_mv(chassis) < 300.0);  // not the ~-12000 mV of kp * 24 in back to the old target
    CHECK_FALSE(util::AUTON_RAN);
  }
}

TEST_CASE("opcontrol_joystick_threshold_set stores the absolute value") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  chassis.pid_print_toggle(false);
  test_stub::g_clock.now_ms = 5000;
  chassis.opcontrol_joystick_threshold_set(-5);
  CHECK(chassis.opcontrol_joystick_threshold_get() == 5);
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = 4;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = 4;
  chassis.opcontrol_tank();
  CHECK(chassis.left_motors[0].fake().voltage == doctest::Approx(0.0));
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = 5;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = 5;
  chassis.opcontrol_tank();
  CHECK(chassis.left_motors[0].fake().voltage > 100.0);
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = 0;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = 0;
}
