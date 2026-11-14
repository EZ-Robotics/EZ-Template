// #621: smoothing pulled the waypoint where a path turns from fwd to rev, or back, inside the fold and piled points up beside it.
#include <cmath>
#include <string>

#include "../doctest.h"
#include "../drive_test_access.hpp"
#include "robot.hpp"

TEST_CASE("#621: a smoothed path reverses at the waypoint the team wrote, with its points still spaced out") {
  struct Shape {
    std::string name;
    std::vector<ez::pose> waypoints;  // the robot starts at (0, 0), so waypoints are in look aheads
    std::vector<ez::drive_directions> directions;
    std::vector<int> reversals;  // indexes into waypoints
  };
  const std::vector<Shape> shapes = {
      {"out and back along one line", {{0, 4}, {0, 1}}, {ez::fwd, ez::rev}, {0}},
      {"V reversal", {{0, 4}, {2, 1}}, {ez::fwd, ez::rev}, {0}},
      {"back and forth twice", {{0, 4}, {0, 2}, {0, 5}}, {ez::fwd, ez::rev, ez::fwd}, {0, 1}},
  };

  for (const Shape& shape : shapes) {
    CAPTURE(shape.name);
    v2::Built built(v2::EXAMPLE_ROBOT);
    auto& c = built.chassis();
    const double look_ahead = c.odom_look_ahead_get();
    const double spacing = c.odom_path_spacing_get();

    std::vector<ez::odom> path;
    for (std::size_t i = 0; i < shape.waypoints.size(); i++)
      path.push_back({{shape.waypoints[i].x * look_ahead, shape.waypoints[i].y * look_ahead, ez::ANGLE_NOT_SET}, shape.directions[i], 110});

    const std::vector<ez::odom> injected = ez::DriveTestAccess::inject_points(c, path);
    c.pid_odom_set(path);
    const std::vector<ez::odom>& smoothed = ez::DriveTestAccess::pp_movements(c);
    const std::vector<int>& waypoint_index = ez::DriveTestAccess::injected_pp_index(c);  // index 0 is the robot's start
    REQUIRE(smoothed.size() == injected.size());
    REQUIRE(waypoint_index.size() == path.size() + 1);

    // The robot turns around where the team asked it to
    for (int r : shape.reversals) {
      const ez::pose& at = smoothed[waypoint_index[r + 1]].target;
      CHECK(at.x == path[r].target.x);
      CHECK(at.y == path[r].target.y);
    }

    // Every leg here touches a reversal. Gaps inject_points() laid at the spacing stay within a tape measure's 1/8 in of it.
    for (std::size_t i = 0; i + 1 < injected.size(); i++) {
      if (std::fabs(ez::util::distance_to_point(injected[i].target, injected[i + 1].target) - spacing) > 1e-6) continue;
      CAPTURE(i);
      CHECK(std::fabs(ez::util::distance_to_point(smoothed[i].target, smoothed[i + 1].target) - spacing) <= 0.125);
    }
  }
}
