// What pid_wait() prints about how an odom move ended and the `interfered` flag it sets have to say the same thing. The odom branch used to
// print "counted as settled" for a robot at rest just outside big_error (a band the stuck watch forgave) while the mA verdict after it,
// which only counts a robot inside big_error, marked the same motion interfered. A stuck verdict counts as settled only inside the settle
// error on every axis, as the mA verdict does, so the two cannot disagree.
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

TEST_CASE("an odom drive on sticky_high_friction prints \"counted as settled\" exactly when it is not interfered") {
  // Seed 858 is the one that split under libstdc++'s noise (std::normal_distribution is not the same sequence on every standard library);
  // the others are the same motion with other noise
  for (unsigned seed = 1; seed <= 60; seed++) {
    unsigned s = seed == 60 ? 858u : seed;
    Rig r(sim::archetype_sticky_high_friction(), 1, true, s);
    r.chassis.pid_print_toggle(true);
    r.chassis.pid_odom_drive_exit_condition_set(300_ms, 1_in, 100_ms, 2_in, 250_ms, 100_ms);
    r.chassis.pid_odom_turn_exit_condition_set(250_ms, 1_deg, 500_ms, 2_deg, 1000_ms, 1000_ms);
    bool returned = false;
    std::string printed = test_stub::capture_stdout([&] {
      r.chassis.pid_odom_set(24_in, 127, true);
      returned = r.wait([&] { r.chassis.pid_wait(); }, 1500);
    });
    CAPTURE(s);
    REQUIRE(returned);
    bool says_settled = printed.find("counted as settled") != std::string::npos;
    CHECK_MESSAGE(says_settled == !r.chassis.interfered, "printed " << (says_settled ? "settled" : "not settled") << " but interfered=" << r.chassis.interfered << "\n"
                                                                    << printed);
  }
}
