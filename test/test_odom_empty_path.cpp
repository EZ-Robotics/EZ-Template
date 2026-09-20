// pid_odom_set(path), pid_odom_pp_set(path), and pid_odom_smooth_pp_set(path)
// all guard against an empty path, but their (path, slew_on) overloads
// didn't -- pid_odom_set(path, slew_on) and pid_odom_smooth_pp_set(path,
// slew_on) reach inject_points(), which indexes imovements[0] with nothing
// to check first, and pid_odom_injected_pp_set(path, slew_on) does the same
// directly. Any caller that passes slew_on explicitly (or a path that
// happens to be empty, e.g. built in a loop) with an empty vector hit
// undefined behavior instead of the same "empty path" message the other
// overloads print.
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}
}  // namespace

TEST_CASE("pid_odom_set(empty path, slew_on) doesn't crash and doesn't start a motion") {
  Drive chassis = make_chassis();
  std::vector<odom> empty_path;

  CHECK_NOTHROW(chassis.pid_odom_set(empty_path, true));
  CHECK(DriveTestAccess::pp_movements(chassis).empty());
}

TEST_CASE("pid_odom_injected_pp_set(empty path, slew_on) doesn't crash and doesn't start a motion") {
  Drive chassis = make_chassis();
  std::vector<odom> empty_path;

  CHECK_NOTHROW(chassis.pid_odom_injected_pp_set(empty_path, false));
  CHECK(DriveTestAccess::pp_movements(chassis).empty());
}

TEST_CASE("pid_odom_smooth_pp_set(empty path, slew_on) doesn't crash and doesn't start a motion") {
  Drive chassis = make_chassis();
  std::vector<odom> empty_path;

  CHECK_NOTHROW(chassis.pid_odom_smooth_pp_set(empty_path, true));
  CHECK(DriveTestAccess::pp_movements(chassis).empty());
}
