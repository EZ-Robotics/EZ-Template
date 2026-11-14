// #619: smoothing moved the point a boomerang target leaves through, so the robot left off the heading the team asked for.
#include <cmath>

#include "../doctest.h"
#include "../drive_test_access.hpp"
#include "robot.hpp"

TEST_CASE("#619: a smoothed boomerang target followed by a turn still leaves along the target's heading") {
  v2::Built built(v2::EXAMPLE_ROBOT);
  auto& c = built.chassis();
  const double look_ahead = c.odom_look_ahead_get();

  // A boomerang target facing 0 deg, then a waypoint 90 deg to its right. It is past the first look ahead, so the path around it is smoothed.
  const ez::pose target = {0, 2 * look_ahead, 0};
  const std::vector<ez::odom> path = {{target, ez::fwd, 110}, {{2 * look_ahead, 3 * look_ahead, ez::ANGLE_NOT_SET}, ez::fwd, 110}};

  // inject_points() puts the lead-out one look ahead past the target along its heading
  const double heading = target.theta * M_PI / 180.0;
  const ez::pose lead_out = {target.x + look_ahead * std::sin(heading), target.y + look_ahead * std::cos(heading), ez::ANGLE_NOT_SET};

  // Where the lead-out sits in the path before smoothing
  const std::vector<ez::odom> injected = ez::DriveTestAccess::inject_points(c, path);
  int lead_out_index = -1;
  for (std::size_t i = 0; i < injected.size(); i++)
    if (ez::util::distance_to_point(injected[i].target, lead_out) < 0.01) lead_out_index = i;
  REQUIRE(lead_out_index > 0);

  c.pid_odom_set(path);
  const std::vector<ez::odom>& smoothed = ez::DriveTestAccess::pp_movements(c);
  REQUIRE(smoothed.size() == injected.size());
  CHECK(ez::util::distance_to_point(smoothed[lead_out_index].target, lead_out) <= 0.125);
}
