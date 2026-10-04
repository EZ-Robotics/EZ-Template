// A robot that got inside big_error, was held there by the speed gate because it was still moving, and then coasted to rest a hair
// outside it, is settled, not interfered. The start commit read it clean only because it left on the pass it was inside the band;
// the robot coasted to the same place. Something that stops outside big_error without ever having been inside it, or that was
// pushed well out of it, still reads interfered (test_exit_gate_disturbance_controls.cpp, test_shove_recovery.cpp).
#include <cmath>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

TEST_CASE("a noisy sticky odom point move that coasts to rest just outside a 1 in big_error is not interfered") {
  // The robot also rests a little outside the heading's big_error (the bearing to a point a little over an inch away moves by a
  // couple of degrees for a hair of sideways drift), and that has to be forgiven by the same one progress step the position is.
  // Seeds 1 to 6 plus the ones that read interfered without it: 4, 16, 248 and 315 under libstdc++'s noise, 392 and 394 under
  // MSVC's (std::normal_distribution is not the same sequence on every standard library, so a seed that rests there on one does
  // not on the other).
  for (unsigned seed : {1u, 2u, 3u, 4u, 5u, 6u, 16u, 248u, 315u, 392u, 394u}) {
    Rig r(sim::archetype_sticky_high_friction(), 1, true, seed);
    r.chassis.pid_odom_drive_exit_condition_set(40_ms, 0.5_in, 100_ms, 1_in, 500_ms, 1000_ms);
    r.chassis.pid_odom_turn_exit_condition_set(40_ms, 1.335_deg, 100_ms, 2.0_deg, 500_ms, 1000_ms);
    r.chassis.pid_odom_set({{12_in, 24_in}, fwd, 110});
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 4000));
    CAPTURE(seed);
    CHECK_FALSE(r.chassis.interfered);
    // it came to rest within 1.5 in of the point
    double rest = r.trace.back().avg;
    CHECK(std::fabs(std::hypot(12.0, 24.0) - rest) < 1.5);
  }
}
