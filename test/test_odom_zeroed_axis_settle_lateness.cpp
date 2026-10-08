// An odom axis whose small and big error are both 0 has no window exits, so a wait on a robot that has come to rest on its target ends only on the
// stuck watch's settled verdict. On noisy sensors the heading never reads stopped over the team's window (the noise alone adds more path than the stop
// speed allows), so the verdict waits for the backstop: no new low for step / stop speed, counted from the last time the settle clock was credited
// (a robot creeping the last inch home credits it). That is later than a wait with the exits set, which is let out by the exit's own short window, but
// it is a bounded time and the verdict is the right one, which is what this pins: a finished motion is reported clean, and within 1.5 s of the robot
// stopping, with either the turn axis or both axes zeroed.
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

Lag run(const sim::SimArchetype& arch, int passes, std::uint32_t seed, bool xy_zeroed) {
  Rig r(arch, passes, true, seed);
  if (xy_zeroed)
    r.chassis.pid_odom_drive_exit_condition_set(90, 0.0, 250, 0.0, 500, 750);
  else
    r.chassis.pid_odom_drive_exit_condition_set(100, 0.5, 300, 2.0, 300, 1000);
  r.chassis.pid_odom_turn_exit_condition_set(90, 0.0, 250, 0.0, 500, 750);
  r.chassis.pid_odom_ptp_set({{0.0, 48.0, ANGLE_NOT_SET}, fwd, 110});
  Lag l{};
  l.returned = r.wait([&] { r.chassis.pid_wait(); }, 4000, &l.ms);
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

TEST_CASE("a robot at rest on its target is reported clean, and within 1.5 s, by an odom axis with its small and big error at 0 on noisy sensors") {
  for (bool xy_zeroed : {false, true})
    for (std::uint32_t seed = 1; seed <= 20; seed++) {
      Lag l = run(sim::archetype_light_fast(), 1, seed, xy_zeroed);
      CAPTURE(xy_zeroed);
      CAPTURE(seed);
      REQUIRE(l.returned);
      CHECK_FALSE(l.interfered);
      CHECK_MESSAGE(l.lag <= 1500.0, "came back " << l.lag << " ms after the robot stopped");
    }
}
