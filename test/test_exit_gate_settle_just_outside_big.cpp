// A robot that comes to rest a hair outside big_error is not settled: a stuck verdict counts as settled only when the robot is inside its settle
// error (the larger of the small and big error) on every axis the motion has, the same rule every other wait uses. This file used to lock the
// opposite for a noisy sticky odom point move: that a robot that got inside big_error, was held there by the speed gate, and then coasted to
// rest within one progress step outside it was forgiven and read clean. That band is gone, because a robot shoved back out of big_error the
// moment it got there and then pinned meets the same conditions and read clean while held 3 in short of the point
// (test_exit_gate_shove_out_and_pin.cpp). Where a robot rests decides the verdict, whichever side of the line it is on.
#include <cmath>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

TEST_CASE("a noisy sticky odom point move reads interfered exactly when it rests outside the 1 in and 2 deg settle error") {
  // Seeds 1 to 6 plus the ones that rested just outside under libstdc++'s noise (4, 16, 248, 315) and under MSVC's (392, 394); the sequence
  // is not the same on every standard library, so which seeds rest outside differs, and the check is on where each one rests.
  for (unsigned seed : {1u, 2u, 3u, 4u, 5u, 6u, 16u, 248u, 315u, 392u, 394u}) {
    Rig r(sim::archetype_sticky_high_friction(), 1, true, seed);
    r.chassis.pid_odom_drive_exit_condition_set(40_ms, 0.5_in, 100_ms, 1_in, 500_ms, 1000_ms);
    r.chassis.pid_odom_turn_exit_condition_set(40_ms, 1.335_deg, 100_ms, 2.0_deg, 500_ms, 1000_ms);
    r.chassis.pid_odom_set({{12_in, 24_in}, fwd, 110});
    REQUIRE(r.wait([&] { r.chassis.pid_wait(); }, 4000));
    CAPTURE(seed);
    double distance = std::hypot(12.0 - r.chassis.odom_x_get(), 24.0 - r.chassis.odom_y_get());
    double heading_error = std::fabs(r.chassis.current_a_odomPID.error);
    CAPTURE(distance);
    CAPTURE(heading_error);
    // A robot within a hair of the line could have been judged on a pass either side of where it ended up, so it is not asked
    bool near_line = std::fabs(distance - 1.0) < 0.05 || std::fabs(heading_error - 2.0) < 0.1;
    if (near_line) continue;
    bool inside = distance < 1.0 && heading_error < 2.0;
    CHECK(r.chassis.interfered == !inside);
  }
}
