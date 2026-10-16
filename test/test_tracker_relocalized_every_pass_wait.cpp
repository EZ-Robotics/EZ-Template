// A custom tracker (a GPS, say) that writes the true pose through odom_xy_set(), and team code that relocalizes on every pass, leaves xy's movement
// unknown on every pass: a pose set from team code is not movement, and the tracker writes its own pose over it.  xyPID then reads the robot as
// stopped while it drives, so its velocity exit must not end the wait.  The robot has to get to its point and the wait must not be interfered.
#include <cmath>

#include "odom_origin_rig.hpp"

using namespace ez;

namespace {
// The robot's true field position, from the sim's wheels along the IMU heading (what a perfect GPS would read)
struct Truth {
  sim::SimRobot& sim;
  Drive& chassis;
  double x = 0, y = 0, last = 0;
  Truth(sim::SimRobot& s, Drive& d) : sim(s), chassis(d) { last = wheels(); }
  double wheels() const { return (sim.left().position_in + sim.right().position_in) / 2.0; }
  void reset(double px, double py) {
    x = px;
    y = py;
    last = wheels();
  }
  void step() {
    double now = wheels();
    double th = chassis.drive_angle_get() * M_PI / 180.0;
    x += (now - last) * std::sin(th);
    y += (now - last) * std::cos(th);
    last = now;
  }
};
}  // namespace

TEST_CASE("xy's velocity exit does not end a wait while team code relocalizes on every pass") {
  for (int passes_per_poll : {1, 2, 3}) {
    origin::Rig r(sim::archetype_light_fast(), passes_per_poll - 1);
    r.tight_exits();
    Truth truth(r.sim, r.chassis);
    r.chassis.odom_tracking_set([&] {
      truth.step();
      r.chassis.odom_xy_set(truth.x, truth.y);
      r.chassis.odom_current.theta = r.chassis.drive_angle_get();
    });
    truth.reset(0, 0);
    r.start_at(0, 0, 0);
    r.chassis.pid_odom_ptp_set(origin::O(0, 24, fwd, 90));
    r.hook = [&](int n) {
      if (n >= 20) r.chassis.odom_xy_set(truth.x + 1.5, truth.y + 1.5);
    };
    origin::Outcome o = r.run([&] { r.chassis.pid_wait(); }, 1500);
    CAPTURE(passes_per_poll);
    REQUIRE(o.returned);
    CHECK_FALSE(o.interfered);
    CHECK(std::hypot(truth.x, truth.y - 24) < 1.0);
  }
}
