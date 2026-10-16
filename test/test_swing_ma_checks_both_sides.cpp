// A swing's mA exit polls the first motor of the swinging side (as in 3.2.2): pid_wait()'s SWING branch and wait_until_turn_swing()'s
// swing path both pick left_motors[0] for a left swing and right_motors[0] for a right swing. An over-current on that motor ends the
// swing at mA_timeout, interfered.
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// Runs the wait for at most `cap` polls and returns whether it came back
template <typename F>
bool returns_within(F&& wait, int cap) {
  test_stub::g_clock.delay_calls_until_stop = cap;
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

TEST_CASE("pid_wait() SWING mA exit catches a stall on the swinging side's first motor, left and right") {
  for (ez::e_swing side : {ez::LEFT_SWING, ez::RIGHT_SWING}) {
    Drive chassis = make_chassis();
    chassis.pid_print_toggle(false);
    // mA exit only (100 ms); small, big and velocity off, so an over-current on the polled motor is the only thing that can end this swing
    chassis.pid_swing_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
    chassis.pid_swing_set(side, side == ez::LEFT_SWING ? 90.0 : -90.0, 100);
    (side == ez::LEFT_SWING ? chassis.left_motors[0] : chassis.right_motors[0]).fake().over_current = true;

    // mA_timeout (100 ms) is crossed on the 11th poll, inside the 15 poll budget
    bool returned = returns_within([&] { chassis.pid_wait(); }, 15);
    CAPTURE(side == ez::LEFT_SWING ? "left" : "right");
    REQUIRE(returned);
    CHECK(chassis.interfered);
  }
}

TEST_CASE("wait_until_turn_swing() SWING mA exit catches a stall on the swinging side's first motor, left and right") {
  for (ez::e_swing side : {ez::LEFT_SWING, ez::RIGHT_SWING}) {
    Drive chassis = make_chassis();
    chassis.pid_print_toggle(false);
    chassis.pid_swing_exit_condition_set(0, 0.0, 0, 0.0, 0, 100);
    chassis.pid_swing_set(side, side == ez::LEFT_SWING ? 90.0 : -90.0, 100);
    (side == ez::LEFT_SWING ? chassis.left_motors[0] : chassis.right_motors[0]).fake().over_current = true;

    bool returned = returns_within([&] { chassis.pid_wait_until(side == ez::LEFT_SWING ? 45_deg : -45_deg); }, 15);
    CAPTURE(side == ez::LEFT_SWING ? "left" : "right");
    REQUIRE(returned);
    CHECK(chassis.interfered);
  }
}
