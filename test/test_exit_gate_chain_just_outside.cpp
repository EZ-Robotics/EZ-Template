// A second chain wait on a point to point move that already settled must read clean when the robot rests a little outside big_error
// of the target the chain pushed. The stuck watch calls a robot that stopped just outside big_error, after having been inside it,
// settled. pid_wait_until_point() used the strict inside-both-big-errors check for the verdict and never looked at that, so once the
// watch was fed the pushed target the second chain wait read the same healthy arrival as interfered.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

sim::SimArchetype four_inch(sim::SimArchetype a) {
  a.wheel_diameter_in = 4.0;
  return a;
}

struct Case {
  const char* name;
  sim::SimArchetype arch;
  int passes;
  double small_error;
  double big_error;
  bool boomerang_first;  // the move before it: a long point to point move, or a reverse boomerang
};

}  // namespace

TEST_CASE("second chain wait on a settled point to point move reads clean just outside big_error of the pushed target") {
  const Case cases[] = {{"light, 1 pass, small exit off, big 0.5", four_inch(sim::archetype_light_fast()), 1, 0.0, 0.5, false},
                        {"sticky, 2 passes, small 0.5, big 2", four_inch(sim::archetype_sticky_high_friction()), 2, 0.5, 2.0, true}};
  for (const Case& cs : cases) {
    Rig r(cs.arch, cs.passes);
    Drive& c = r.chassis;
    int small_time = cs.small_error > 0 ? 100 : 0;
    c.pid_odom_drive_exit_condition_set(small_time, cs.small_error, 200, cs.big_error, 200, 500);
    c.pid_odom_turn_exit_condition_set(small_time, cs.small_error * 3.0, 200, cs.big_error * 3.0, 200, 500);
    bool ok_first = false, ok_a = false, ok_b = false;
    bool first_clean = false, a_clean = false;
    std::string out = test_stub::capture_stdout([&]() {
      if (cs.boomerang_first)
        c.pid_odom_boomerang_set(odom{pose{-10, -24, 200}, rev, 110});
      else
        c.pid_odom_ptp_set(odom{pose{30, 48}, fwd, 110});
      ok_first = r.wait([&] { c.pid_wait(); }, 1500);
      first_clean = !c.interfered;
      if (cs.boomerang_first)
        c.pid_odom_ptp_set(odom{pose{-12, -24}, rev, 110});
      else
        c.pid_odom_ptp_set(odom{pose{12, 24}, fwd, 110});
      ok_a = r.wait([&] { c.pid_wait_quick_chain(); }, 1500);
      a_clean = !c.interfered;
      ok_b = r.wait([&] { c.pid_wait_quick_chain(); }, 1500);
    });
    INFO(cs.name);
    REQUIRE(ok_first);
    REQUIRE(ok_a);
    REQUIRE(ok_b);
    CHECK(first_clean);
    CHECK(a_clean);
    CHECK_FALSE(c.interfered);
  }
}
