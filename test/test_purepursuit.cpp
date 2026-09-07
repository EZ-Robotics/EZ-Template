// pure pursuit [H8, L10]: inject_points on a 24 in straight path at 0.5 in
// spacing yields the expected count and injected_pp_index of size 2;
// smooth_path on a 600-point path returns 600 points with endpoints
// unchanged (this would have overflowed the old arrays);
// new_turn_target_compute table for raw/cw/ccw/shortest/longest from current
// 10 to targets 200, -160, 350, 0.
#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}
}  // namespace

TEST_CASE("pure pursuit [H8, L10] inject_points on a 24in straight path at 0.5in spacing") {
  Drive chassis = make_chassis();
  chassis.odom_xyt_set(0.0, 0.0, 0.0);

  std::vector<odom> path = {{{24, 0}, fwd, 127}};
  auto output = DriveTestAccess::inject_points(chassis, path);

  // 24in / 0.5in default SPACING = 48 candidate slots between the injected
  // start point and the target; points within ODOM_LOOK_AHEAD (7in default)
  // of either end are skipped, so the actual count is derived by running
  // the injection loop, not just 24/0.5.
  CHECK(output.size() == 36);
  CHECK(DriveTestAccess::injected_pp_index(chassis).size() == 2);
}

TEST_CASE("pure pursuit [L10] smooth_path on a 600-point path returns 600 points with endpoints unchanged") {
  Drive chassis = make_chassis();

  std::vector<odom> path;
  path.reserve(600);
  for (int i = 0; i < 600; i++) {
    path.push_back({{(double)i, 0.0}, fwd, 127});
  }

  auto output = DriveTestAccess::smooth_path(chassis, path, 0.1, 0.5, 0.0001);

  REQUIRE(output.size() == 600);
  CHECK(output.front().target.x == doctest::Approx(path.front().target.x));
  CHECK(output.front().target.y == doctest::Approx(path.front().target.y));
  CHECK(output.back().target.x == doctest::Approx(path.back().target.x));
  CHECK(output.back().target.y == doctest::Approx(path.back().target.y));
}

TEST_CASE("pure pursuit [L10] new_turn_target_compute from current 10") {
  Drive chassis = make_chassis();
  const double current = 10;

  struct Row {
    double target;
    double raw, cw, ccw, shortest, longest;
  };
  // clang-format off
  Row rows[] = {
    // target,  raw,    cw,    ccw,   shortest, longest
    {   200,    200,    200,   -160,   -160,     200 },
    {  -160,   -160,    200,   -160,   -160,     200 },
    {   350,    350,    350,    -10,    -10,     350 },
    {     0,      0,    360,      0,      0,     360 },
  };
  // clang-format on

  for (auto& row : rows) {
    CAPTURE(row.target);
    CHECK(DriveTestAccess::new_turn_target_compute(chassis, row.target, current, raw) == doctest::Approx(row.raw));
    CHECK(DriveTestAccess::new_turn_target_compute(chassis, row.target, current, cw) == doctest::Approx(row.cw));
    CHECK(DriveTestAccess::new_turn_target_compute(chassis, row.target, current, ccw) == doctest::Approx(row.ccw));
    CHECK(DriveTestAccess::new_turn_target_compute(chassis, row.target, current, shortest) == doctest::Approx(row.shortest));
    CHECK(DriveTestAccess::new_turn_target_compute(chassis, row.target, current, longest) == doctest::Approx(row.longest));
  }
}
