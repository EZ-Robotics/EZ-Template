// Path smoothing repeats until a pass moves the points by less than the tolerance, but stops at 5000 passes so
// constants that settle very slowly can't stall the caller. Constants that are accepted (weight_data + 2 *
// weight_smooth < 2) but settle slowly, such as a weight_data of 0 on a long path, hit that limit and the caller was
// handed a path that is less smooth than its constants ask for without being told. It now prints one line when that
// happens. The path that comes back is the same as before.
#include <cmath>
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

// A 256 point path, the kind pid_odom_smooth_pp_set builds from four waypoints 24 inches apart.
std::vector<odom> injected_path(Drive& chassis) {
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  return DriveTestAccess::inject_points(chassis, {{{24, 24}, fwd, 127}, {{48, 0}, fwd, 127}, {{72, 24}, fwd, 127}, {{96, 48}, fwd, 127}});
}

const char* const pass_limit_message = "EZ-Template: path smoothing stopped at 5000 passes before it settled, so the path is less smooth than the constants ask for\n";

template <typename F>
std::string printed_by(F&& action) {
  return test_stub::capture_stdout(std::forward<F>(action));
}
}  // namespace

TEST_CASE("smooth_path says so once when the constants settle too slowly for the pass limit") {
  Drive chassis = make_chassis();
  auto path = injected_path(chassis);
  REQUIRE(path.size() > 200);

  // weight_data + 2 * weight_smooth is 1, well inside what is accepted, but with no weight on the original points
  // a path this long needs far more than 5000 passes to settle.
  CHECK(printed_by([&] { DriveTestAccess::smooth_path(chassis, path, 0.5, 0.0, 0.0001); }) == pass_limit_message);
}

TEST_CASE("smooth_path still returns the smoothed path when it stops at the pass limit") {
  Drive chassis = make_chassis();
  auto path = injected_path(chassis);

  std::vector<odom> output;
  printed_by([&] { output = DriveTestAccess::smooth_path(chassis, path, 0.5, 0.0, 0.0001); });

  REQUIRE(output.size() == path.size());
  CHECK(output.front().target.x == doctest::Approx(path.front().target.x));
  CHECK(output.back().target.x == doctest::Approx(path.back().target.x));
  for (const auto& point : output) {
    CHECK(std::isfinite(point.target.x));
    CHECK(std::isfinite(point.target.y));
  }
}

TEST_CASE("smooth_path prints nothing when the constants settle before the pass limit") {
  Drive chassis = make_chassis();
  auto path = injected_path(chassis);

  // The constants a chassis starts with, and the pair the other smoothing tests use.
  auto defaults = chassis.odom_path_smooth_constants_get();
  REQUIRE(defaults.size() == 3);
  CHECK(printed_by([&] { DriveTestAccess::smooth_path(chassis, path, defaults[0], defaults[1], defaults[2]); }) == "");
  CHECK(printed_by([&] { DriveTestAccess::smooth_path(chassis, path, 0.95, 0.05, 0.0001); }) == "");
}

TEST_CASE("smooth_path prints only the rejection, not the pass limit message, for constants that would diverge") {
  Drive chassis = make_chassis();
  auto path = injected_path(chassis);

  std::string printed = printed_by([&] { DriveTestAccess::smooth_path(chassis, path, 0.99, 0.03, 0.0001); });
  CHECK(printed.find("EZ-Template: path smoothing skipped") == 0);
  CHECK(printed.find("stopped at") == std::string::npos);
}
