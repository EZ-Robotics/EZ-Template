// The text the chassis setters print to the terminal, byte for byte.
//
// Each of these prints while the setter holds the chassis lock. That lock is going to be held at a raised task
// priority, and printing can block, so the prints are being moved to happen after the lock is released
// (print_after_unlock). Teams read this output, so moving it must not change a single character. This test
// was written against the code before the move, and has to pass both before and after.
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

template <typename F>
std::string printed_by(F&& action) {
  return test_stub::capture_stdout(std::forward<F>(action));
}
}  // namespace

TEST_CASE("pid_drive_set prints its target, and says so when slew is on") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_drive_set(12, 110); }) == "Drive Started... Target Value: 12.00\n");
  CHECK(printed_by([&] { chassis.pid_drive_set(-6.5, 110, true); }) == "Drive Started... Target Value: -6.50 with slew\n");
}

TEST_CASE("pid_turn_set prints its target") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_turn_set(90.0, 110); }) == "Turn Started... Target Value: 90.00\n");
}

TEST_CASE("pid_swing_set prints its target") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_swing_set(LEFT_SWING, 45.0, 110); }) == "Swing Started... Target Value: 45.00\n");
}

TEST_CASE("pid_turn_set to a point prints the point, then the turn it becomes") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_turn_set({0.0, 24.0, 0.0}, fwd, 110); }) ==
        "Turn to Point PID Started... Target Point: (0.00, 24.00) \n"
        "Turn Started... Target Value: 0.00\n");
}

TEST_CASE("pid_odom_ptp_set prints its target coordinates") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 110}); }) ==
        "Odom Motion Started... Target Coordinates: (0.00, 24.00, 0.00) \n");
}

TEST_CASE("pid_odom_ptp_set while pure pursuit is running starts its line with a space") {
  Drive chassis = make_chassis();
  chassis.pid_odom_pp_set({{{0.0, 12.0, 0.0}, fwd, 110}, {{0.0, 24.0, 0.0}, fwd, 110}});  // leaves the mode at pure pursuit
  CHECK(printed_by([&] { chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 110}); }) ==
        " Odom Motion Started... Target Coordinates: (0.00, 24.00, 0.00) \n");
}

TEST_CASE("pid_wait_quick_chain says so when there is no motion to chain") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.pid_wait_quick_chain(); }) == "Not in a supported drive mode!\n");
}

TEST_CASE("drive_imus_scalers_3600_set names the port of an imu it rejects") {
  Drive chassis = make_chassis();
  CHECK(printed_by([&] { chassis.drive_imus_scalers_3600_set({50.0}); }) ==
        "EZ-Template: drive_imus_scalers_3600_set rejected 50 for imu on port 5, value must be the imu's reading after "
        "physically turning the robot 3600 degrees (about 3600)\n");
}

TEST_CASE("nothing is printed when print_toggle is off") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  CHECK(printed_by([&] {
          chassis.pid_drive_set(12, 110);
          chassis.pid_turn_set(90.0, 110);
          chassis.pid_swing_set(LEFT_SWING, 45.0, 110);
          chassis.pid_turn_set({0.0, 24.0, 0.0}, fwd, 110);
          chassis.pid_odom_ptp_set({{0.0, 24.0, 0.0}, fwd, 110});
        })
            .empty());
}
