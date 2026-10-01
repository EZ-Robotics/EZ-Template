// A motor handed to the PTO is not the drive's motor, so its current must not count toward the drive's mA exit. Since
// the mA exits started polling every motor on both sides (so a jam on any drive motor is caught, not just the
// front one) the list also included motors currently running an intake or lift, and an intake that stalls ended
// every turn and swing at about 520 ms with `mA Exit` and interfered. 3.2.2 only checked left_motors[0] /
// right_motors[0], which can never be PTO'd.
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

// mA is the only exit configured (100 ms), so only over-current can end a wait quickly. Each SingleStuckWatch /
// StuckWatch is still live, but only after its own 1000 ms start allowance plus its window.
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

TEST_CASE("control: a non-PTO'd drive motor over current still ends every wait on mA_EXIT at mA_timeout") {
  for (Wait w : {Wait::Drive, Wait::DriveUntil, Wait::Turn, Wait::TurnUntil, Wait::Swing, Wait::Odom}) {
    Drive chassis = make_chassis();
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    ma_only(chassis);
    pto_on(chassis);
    over_current(chassis, 1, true);  // a real drive motor jams
    CAPTURE(name(w));
    bool returned = returns_within(30, [&] { start_and_wait(chassis, w); });
    CHECK(returned);
    CHECK(chassis.interfered);
  }
}

TEST_CASE("control: after pto_toggle(false) the motor counts toward the mA exit again") {
  Drive chassis = make_chassis();
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  ma_only(chassis);
  pto_on(chassis);
  over_current(chassis, 2, true);
  chassis.pto_toggle({chassis.left_motors[2], chassis.right_motors[2]}, false);
  bool returned = returns_within(30, [&] { start_and_wait(chassis, Wait::Turn); });
  CHECK(returned);
  CHECK(chassis.interfered);
}

namespace {
Drive* g_toggle_chassis = nullptr;
int g_toggle_pass = 0;
int g_toggle_at = 0;
bool g_toggle_to = false;
void toggle_hook() {
  if (++g_toggle_pass == g_toggle_at) g_toggle_chassis->pto_toggle({g_toggle_chassis->left_motors[2], g_toggle_chassis->right_motors[2]}, g_toggle_to);
}
}  // namespace

TEST_CASE("a PTO toggled in the middle of a turn changes the mA list on the next pass") {
  // Toggled on at pass 5 while the motor is already over current: the list is rebuilt, so the motor stops
  // counting, the mA timer stops accumulating and the turn is no longer ended by it.
  {
    Drive chassis = make_chassis();
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    ma_only(chassis);
    over_current(chassis, 2, true);
    g_toggle_chassis = &chassis;
    g_toggle_pass = 0;
    g_toggle_at = 5;
    g_toggle_to = true;
    test_stub::g_clock.on_delay = &toggle_hook;
    bool returned = returns_within(50, [&] { start_and_wait(chassis, Wait::Turn); });
    test_stub::g_clock.on_delay = nullptr;
    CHECK_FALSE(returned);
  }
  // Toggled off at pass 5: the motor counts again from then on and the turn ends on mA about mA_timeout later.
  {
    Drive chassis = make_chassis();
    DriveTestAccess::imu_calibration_complete(chassis) = true;
    ma_only(chassis);
    pto_on(chassis);
    over_current(chassis, 2, true);
    g_toggle_chassis = &chassis;
    g_toggle_pass = 0;
    g_toggle_at = 5;
    g_toggle_to = false;
    test_stub::g_clock.on_delay = &toggle_hook;
    bool returned = returns_within(30, [&] { start_and_wait(chassis, Wait::Turn); });
    test_stub::g_clock.on_delay = nullptr;
    CHECK(returned);
    CHECK(chassis.interfered);
  }
}

TEST_CASE("sim: a 6 motor drive with a stalled PTO'd intake still finishes a 90 degree turn, uninterfered") {
  for (int rep = 0; rep < 2; rep++) {
    auto a = rep == 0 ? sim::archetype_light_fast() : sim::archetype_sticky_high_friction();
    Drive chassis = make_chassis(a.wheel_diameter_in, a.cartridge_rpm);
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
