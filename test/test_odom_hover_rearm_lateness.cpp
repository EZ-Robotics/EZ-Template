// A robot that settles at the edge of its small error band, on a noisy sensor, crosses that band's edge on and off after the wait has latched a
// small exit. Each time the stuck verdict found the latched exit's error back outside the band, it took the exit back and gave the watch a fresh
// clock (up to STUCK_WATCH_REARM_CAP times), so the wait came back up to 1.5 s after the robot stopped, where before it came back 0.3 to 0.9 s
// after. A robot that is stopped and still well inside its big error is settled, and it does not take a new clock to say so.
#include <cmath>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

struct Lag {
  bool returned;
  bool interfered;
  double ms;
  double lag;
};

Lag run(const sim::SimArchetype& arch, int passes, bool noise, std::uint32_t seed, bool both) {
  Rig r(arch, passes, noise, seed);
  r.chassis.pid_odom_drive_exit_condition_set(100, 0.5, 300, 2.0, 300, 1000);
  if (both) r.chassis.pid_odom_turn_exit_condition_set(100, 0.5, 300, 2.0, 300, 1000);
  r.chassis.pid_odom_ptp_set({{0.0, 48.0, ANGLE_NOT_SET}, fwd, 110});
  Lag l{};
  l.returned = r.wait([&] { r.chassis.pid_wait(); }, 3000, &l.ms);
  l.interfered = r.chassis.interfered;
  // Truly stopped: the last sample whose preceding 90 ms of wheel travel was over the floor
  double moving_until = 0;
  for (size_t i = 1; i < r.trace.size(); i++) {
    double window_start = r.trace[i].t_ms - 90;
    size_t j = i;
    while (j > 0 && r.trace[j - 1].t_ms >= window_start) j--;
    double path = 0;
    for (size_t k = j + 1; k <= i; k++) path += std::fmax(std::fabs(r.trace[k].left - r.trace[k - 1].left), std::fabs(r.trace[k].right - r.trace[k - 1].right));
    if (path / 0.09 >= r.drive_floor(90)) moving_until = r.trace[i].t_ms;
  }
  l.lag = l.ms - moving_until;
  return l;
}

}  // namespace

TEST_CASE("a stopped robot hovering at the edge of its small error band does not hold a noisy odom wait for seconds") {
  // A sticky robot on a noisy sensor, small error 0.5 in and big error 2 in. It is truly stopped, and wherever it comes to rest is inside the big
  // error, so how long the wait takes to say so does not depend on which side of 0.5 in it is on.
  for (bool both : {false, true})
    for (std::uint32_t seed = 1; seed <= 12; seed++) {
      Lag l = run(sim::archetype_sticky_high_friction(), 3, true, seed, both);
      CAPTURE(both);
      CAPTURE(seed);
      REQUIRE(l.returned);
      CHECK_FALSE(l.interfered);
      CHECK_MESSAGE(l.lag <= 1000.0, "came back " << l.lag << " ms after the robot stopped");
    }
}
