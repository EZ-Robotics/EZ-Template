// A motion with its velocity and mA exit times both 0 has nothing to end its wait when the robot is pinned short of its target.  Setting that
// prints one warning per Drive.  Any other exit constants print nothing.
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "stdout_capture.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}
int count_of(const std::string& text, const std::string& what) {
  int n = 0;
  for (size_t at = text.find(what); at != std::string::npos; at = text.find(what, at + 1)) n++;
  return n;
}
}  // namespace

TEST_CASE("velocity and mA exit times of 0 warn once, and any other exit times do not") {
  Drive chassis = make_chassis();
  std::string quiet = test_stub::capture_stdout([&] {
    chassis.pid_drive_exit_condition_set(90, 1.0, 250, 3.0, 500, 0);
    chassis.pid_turn_exit_condition_set(90, 1.0, 250, 3.0, 0, 500);
  });
  CHECK(count_of(quiet, "Warning") == 0);
  std::string loud = test_stub::capture_stdout([&] {
    chassis.pid_swing_exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
    chassis.pid_odom_drive_exit_condition_set(90, 1.0, 250, 3.0, 0, 0);
  });
  CHECK(count_of(loud, "velocity and mA exit times of 0") == 1);
}
