// A motor handed to the PTO is not the drive's motor, so its current must not count toward the drive's mA exit.  The mA exits poll only the first
// motor of each side (left_motors[0] / right_motors[0], 3.2.2's behavior), which can never be PTO'd, so an intake or lift that stalls on a PTO'd motor
// cannot end a turn or swing with `mA Exit` and interfered.  The other motors of a side do not count either, PTO'd or not.
#include <cmath>
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "sim_physics.hpp"

using namespace ez;

namespace {
// Three motors a side; the last of each is the PTO'd one (the first index can never be).
Drive make_chassis(double wheel_in = 3.25, double rpm = 360) {
  test_stub::reset_all();
  return Drive({1, -2, 3}, {-4, 5, -6}, 7, wheel_in, rpm);
}

void pto_on(Drive& chassis) { chassis.pto_toggle({chassis.left_motors[2], chassis.right_motors[2]}, true); }

void over_current(Drive& chassis, int index, bool value) {
  chassis.left_motors[index].fake().over_current = value;
  chassis.right_motors[index].fake().over_current = value;
}

// Runs `wait` under a budget of `passes` delay calls. Returns whether it returned on its own.
bool returns_within(int passes, const std::function<void()>& wait) {
  test_stub::g_clock.delay_calls_until_stop = passes;
  bool returned = true;
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  return returned;
}

// mA is the only exit configured (100 ms), so only over-current can end a wait quickly.
void ma_only(Drive& chassis) {
  chassis.pid_print_toggle(false);
  chassis.pid_drive_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  chassis.pid_turn_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  chassis.pid_swing_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  chassis.pid_odom_drive_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
  chassis.pid_odom_turn_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
}

enum class Wait {
  Drive,
  DriveUntil,
  Turn,
  TurnUntil,
  Swing,
  Odom
};

void start_and_wait(Drive& chassis, Wait w) {
  switch (w) {
    case Wait::Drive:
      chassis.pid_drive_set(48.0, 100);
      chassis.pid_wait();
      break;
    case Wait::DriveUntil:
      chassis.pid_drive_set(48.0, 100);
      chassis.pid_wait_until(24.0);
      break;
    case Wait::Turn:
      chassis.pid_turn_set(90.0, 100);
      chassis.pid_wait();
      break;
    case Wait::TurnUntil:
      chassis.pid_turn_set(90.0, 100);
      chassis.pid_wait_until(45.0);
      break;
    case Wait::Swing:
      chassis.pid_swing_set(ez::LEFT_SWING, 90.0, 100);
      chassis.pid_wait();
      break;
    case Wait::Odom:
      chassis.pid_odom_set({{0.0, 24.0, ANGLE_NOT_SET}, fwd, 100});
      chassis.pid_wait();
      break;
  }
}

const char* name(Wait w) {
  switch (w) {
    case Wait::Drive:
      return "pid_wait DRIVE";
    case Wait::DriveUntil:
      return "pid_wait_until DRIVE";
    case Wait::Turn:
      return "pid_wait TURN";
    case Wait::TurnUntil:
      return "pid_wait_until TURN";
    case Wait::Swing:
      return "pid_wait SWING";
    case Wait::Odom:
      return "pid_wait ODOM";
  }
  return "";
}
}  // namespace

TEST_CASE("PTO'd motors over current do not end the drive, swing, turn or odom waits") {
  for (Wait w : {Wait::Drive, Wait::DriveUntil, Wait::Turn, Wait::TurnUntil, Wait::Swing, Wait::Odom}) {
    Drive chassis = make_chassis();
    chassis.pid_exit_speed_window_set(40);  // 40 ms: even number of ticks, see test_speed_window.cpp
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    ma_only(chassis);
    pto_on(chassis);
    over_current(chassis, 2, true);  // the PTO'd intake stalls
    CAPTURE(name(w));
    // 50 passes = 500 ms: past mA_timeout (100 ms) many times over, well short of the 1000 ms stuck allowance.
    bool returned = returns_within(50, [&] { start_and_wait(chassis, w); });
    CHECK_FALSE(returned);
  }
}

TEST_CASE("control: the first drive motor of each side over current still ends every wait on mA_EXIT at mA_timeout") {
  for (Wait w : {Wait::Drive, Wait::DriveUntil, Wait::Turn, Wait::TurnUntil, Wait::Swing, Wait::Odom}) {
    Drive chassis = make_chassis();
    chassis.pid_exit_speed_window_set(40);  // 40 ms: even number of ticks, see test_speed_window.cpp
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    ma_only(chassis);
    pto_on(chassis);
    over_current(chassis, 0, true);  // the first motor of each side jams: it can never be PTO'd
    CAPTURE(name(w));
    bool returned = returns_within(30, [&] { start_and_wait(chassis, w); });
    CHECK(returned);
    CHECK(chassis.interfered);
  }
}

TEST_CASE("control: only the first motor of each side counts toward the mA exit, a jam on another drive motor does not end the waits") {
  for (Wait w : {Wait::Drive, Wait::DriveUntil, Wait::Turn, Wait::TurnUntil, Wait::Swing, Wait::Odom}) {
    Drive chassis = make_chassis();
    chassis.pid_exit_speed_window_set(40);  // 40 ms: even number of ticks, see test_speed_window.cpp
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    ma_only(chassis);
    over_current(chassis, 1, true);  // a middle drive motor, not PTO'd, is over current
    CAPTURE(name(w));
    bool returned = returns_within(50, [&] { start_and_wait(chassis, w); });
    CHECK_FALSE(returned);
  }
}

TEST_CASE("sim: a 6 motor drive with a stalled PTO'd intake still finishes a 90 degree turn, uninterfered") {
  for (int rep = 0; rep < 2; rep++) {
    auto a = rep == 0 ? sim::archetype_light_fast() : sim::archetype_sticky_high_friction();
    Drive chassis = make_chassis(a.wheel_diameter_in, a.cartridge_rpm);
    chassis.pid_exit_speed_window_set(40);  // 40 ms: even number of ticks, see test_speed_window.cpp
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    chassis.pid_print_toggle(false);
    sim::SimRobot sim(chassis, a, sim::NoiseConfig{false, 1});
    pto_on(chassis);
    over_current(chassis, 2, true);  // stays set: the sim leaves PTO'd motors alone
    chassis.pid_turn_set(90.0, 90);
    bool returned = returns_within(3000, [&] { chassis.pid_wait(); });
    CAPTURE(a.name);
    REQUIRE(returned);
    CHECK_FALSE(chassis.interfered);
    CHECK(std::fabs(chassis.drive_angle_get() - 90.0) < 3.0);
  }
}
