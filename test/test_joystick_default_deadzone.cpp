// Default joystick deadzone of 3.
//
// Active brake only runs when both processed sticks are exactly 0, and the default JOYSTICK_THRESHOLD was 0, so a
// controller whose stick rests at 1 or 2 never braked: the robot could be pushed around with active brake on. A stick
// flickering 0/1 at rest also kept re-aiming the brake target. The default is now 3, so sticks reading 0, 1 and 2 read
// 0 and 3 passes through; opcontrol_joystick_threshold_set(0) gives the old behavior back exactly.
#include <cmath>
#include <utility>
#include <vector>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
using Script = std::vector<std::pair<int, int>>;  // (left stick, right stick) held for `kSegment` ticks each
constexpr int kSegment = 20;

// FNV-1a over every drive motor's voltage, every tick, rounded to the millivolt.
std::uint64_t run_script(const Script& script, double kp, bool zero_threshold) {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  if (kp > 0) chassis.opcontrol_drive_activebrake_set(kp);
  if (zero_threshold) chassis.opcontrol_joystick_threshold_set(0);

  std::uint64_t h = 1469598103934665603ull;
  for (auto [l, r] : script) {
    master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = l;
    master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = r;
    for (int t = 0; t < kSegment; t++) {
      chassis.opcontrol_tank();
      for (auto& m : chassis.left_motors) h = (h ^ (std::uint64_t)std::llround(m.fake().voltage)) * 1099511628211ull;
      for (auto& m : chassis.right_motors) h = (h ^ (std::uint64_t)std::llround(m.fake().voltage)) * 1099511628211ull;
      pros::delay(util::DELAY_TIME);
    }
  }
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = 0;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = 0;
  return h;
}

// Full stick, small corrections at 3 to 10, release, reversals, turn in place: every value is 0 or magnitude 3+.
const Script kAtOrAboveThree = {{0, 0}, {127, 127},  {5, 5},    {3, 3},  {0, 0}, {-127, -127}, {-10, -10},
                                {0, 0}, {127, -127}, {10, -10}, {-3, 3}, {0, 0}, {100, 60},    {0, 0}};

// The same idea with a resting stick's 1s and 2s mixed in.
const Script kWithOnesAndTwos = {{0, 0},   {1, 1},       {127, 127}, {2, -2}, {5, 5},      {1, 0},  {0, 2},
                                 {-1, -1}, {-127, -127}, {2, 2},     {0, 0},  {127, -127}, {-2, 1}, {0, 0}};

// kWithOnesAndTwos with every 1 and 2 replaced by 0: what the default deadzone should turn it into.
Script zeroed(Script s) {
  for (auto& [l, r] : s) {
    if (std::abs(l) < 3) l = 0;
    if (std::abs(r) < 3) r = 0;
  }
  return s;
}

// Captured on a107ac8, where the default threshold was 0: kWithOnesAndTwos, and what opcontrol_joystick_threshold_set(0)
// must keep giving exactly.
constexpr std::uint64_t kGoldenKp0 = 2540916659744836995ull, kGoldenKp2 = 2540916659744836995ull;
std::uint64_t golden_ones_and_twos(double kp) { return kp == 0.0 ? kGoldenKp0 : kGoldenKp2; }
}  // namespace

TEST_CASE("the default joystick threshold is 3") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  CHECK(chassis.opcontrol_joystick_threshold_get() == 3);
  CHECK(chassis.JOYSTICK_THRESHOLD == 3);
  chassis.opcontrol_joystick_threshold_set(0);
  CHECK(chassis.opcontrol_joystick_threshold_get() == 0);
}

TEST_CASE("a stick resting at 1 and -2 with active brake on: a robot pushed 3 in is pulled back") {
  test_stub::reset_all();
  auto a = sim::archetype_light_fast();
  Drive chassis({1, -2}, {-3, 4}, 5, a.wheel_diameter_in, a.cartridge_rpm);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.pid_print_toggle(false);
  sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
  chassis.opcontrol_drive_activebrake_set(2.0);
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = 1;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = -2;
  for (int t = 0; t < 30; t++) {
    chassis.opcontrol_tank();
    pros::delay(util::DELAY_TIME);
  }
  double before = sim.left().position_in;
  sim.displace(3.0);
  chassis.opcontrol_tank();
  // The brake pushes back against the push: a negative voltage on both sides, nowhere near the stick's own ~+95/-190 mV.
  CHECK(chassis.left_motors[0].fake().voltage < -300.0);
  CHECK(chassis.right_motors[0].fake().voltage < -300.0);
  for (int t = 0; t < 100; t++) {
    pros::delay(util::DELAY_TIME);
    chassis.opcontrol_tank();
  }
  CAPTURE(sim.left().position_in - before);
  CHECK(sim.left().position_in - before < 2.9);  // it moved back toward where it was, not stayed where it was pushed to
}

TEST_CASE("a stick resting at 1 or 2 reads as released, so a script full of them is the same script with zeros") {
  for (double kp : {0.0, 2.0}) {
    CAPTURE(kp);
    CHECK(run_script(kWithOnesAndTwos, kp, false) == run_script(zeroed(kWithOnesAndTwos), kp, false));
  }
}

TEST_CASE("regression: a stick at 3 or more, or 0, drives exactly as it did with the threshold at 0") {
  for (double kp : {0.0, 2.0}) {
    CAPTURE(kp);
    CHECK(run_script(kAtOrAboveThree, kp, false) == run_script(kAtOrAboveThree, kp, true));
  }
}

TEST_CASE("regression: opcontrol_joystick_threshold_set(0) reproduces the outputs of a107ac8 exactly, including at 1 and 2") {
  for (double kp : {0.0, 2.0}) {
    CAPTURE(kp);
    MESSAGE("GOLDEN kp=", kp, " hash=", run_script(kWithOnesAndTwos, kp, true));
    CHECK(run_script(kWithOnesAndTwos, kp, true) == golden_ones_and_twos(kp));
  }
}

TEST_CASE("a stick at 3 passes through, at 2 reads zero") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 360);
  chassis.pid_print_toggle(false);
  test_stub::g_clock.now_ms = 5000;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = 2;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = 2;
  chassis.opcontrol_tank();
  CHECK(chassis.left_motors[0].fake().voltage == doctest::Approx(0.0));
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = 3;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = -3;
  chassis.opcontrol_tank();
  CHECK(chassis.left_motors[0].fake().voltage == doctest::Approx(3 * 12000.0 / 127.0).epsilon(0.005));
  CHECK(chassis.right_motors[0].fake().voltage == doctest::Approx(-3 * 12000.0 / 127.0).epsilon(0.005));
  master.fake_analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = 0;
  master.fake_analog[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = 0;
}
