// #570: a custom tracker that writes its pose with the pose setters made odom waits read the robot as stopped while it drove.
#include <cmath>
#include <functional>

#include "../doctest.h"
#include "robot.hpp"

TEST_CASE("#570: a custom tracker using the pose setters ends an odom move stopped, inside big_error") {
  // How a team's own tracker might write the pose it measured. Here it measures the robot's true position. The heading comes
  // from the IMU as in odom_tracking_set()'s example, since the heading setters are not for calling every pass.
  const std::vector<std::pair<const char*, std::function<void(ez::Drive&, double, double)>>> SETTERS{
      {"odom_xy_set", [](ez::Drive& c, double x, double y) { c.odom_xy_set(x, y); }},
      {"odom_x_set and odom_y_set",
       [](ez::Drive& c, double x, double y) {
         c.odom_x_set(x);
         c.odom_y_set(y);
       }},
  };

  for (const auto& [name, write_pose] : SETTERS) {
    const std::string row = name;
    INFO(row);
    v2::Built built(v2::EXAMPLE_ROBOT);
    auto& c = built.chassis();
    auto& s = built.sim();
    c.odom_tracking_set([&] {
      write_pose(c, s.x(), s.y());
      c.odom_current.theta = c.drive_angle_get();
    });

    c.pid_odom_set({{12_in, 24_in}, ez::fwd, 110}, true);
    s.give_up(3000, 10000);
    bool returned = true;
    try {
      c.pid_wait();
    } catch (sim::WaitNeverReturned&) {
      returned = false;
    }
    s.give_up(0, 0);
    REQUIRE(returned);
    REQUIRE_FALSE(c.interfered);

    CHECK(std::hypot(s.x() - 12, s.y() - 24) <= c.xyPID.exit.big_error);
    double x = s.x(), y = s.y();
    int window = c.pid_exit_speed_window_get();
    pros::delay(window);
    CHECK(std::hypot(s.x() - x, s.y() - y) * 1000.0 / window <= c.pid_distance_exit_stop_speed_get());
  }
}
