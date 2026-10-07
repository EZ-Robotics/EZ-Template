// pid_wait() gives a side's stuck watch a fresh clock when its latched window exit is taken back (the side was shoved off its target, or
// started moving again after it latched): the watch stopped being fed the moment the side latched, so left alone its clock is stale and the
// first pass after the disturbance reads the side as stuck. pid_wait_until() on a drive, and so pid_wait_quick(), took the exit back
// without giving the watch that clock, so a push that landed after the side had latched, with the other side still waiting, was called
// stuck-but-settled at once, with the robot still moving at 4.4 in/s and 0.7 in short of where it was going.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

enum class Wait {
  Full,
  Quick,
  Until
};

const char* name(Wait w) { return w == Wait::Full ? "pid_wait()" : w == Wait::Quick ? "pid_wait_quick()" : "pid_wait_until(24_in)"; }

void run(Wait w, Rig& r) {
  switch (w) {
    case Wait::Full:
      r.chassis.pid_wait();
      break;
    case Wait::Quick:
      r.chassis.pid_wait_quick();
      break;
    case Wait::Until:
      r.chassis.pid_wait_until(24_in);
      break;
  }
}

}  // namespace

TEST_CASE("a push after both sides latched is not called settled while the robot is still moving, by every drive wait") {
  for (Wait w : {Wait::Full, Wait::Quick, Wait::Until}) {
    Rig r(archetype_classroom(), 1, false, 1);
    r.sim.push(-60.0, 1250, 150);
    r.chassis.pid_drive_set(24_in, 110);
    double ms = 0;
    bool ok = r.wait([&] { run(w, r); }, 3000, &ms);
    CAPTURE(std::string(name(w)));
    REQUIRE(ok);
    std::printf("  [%s] returned at %.0f ms, %s, %.2f in, %.2f in/s over 90 ms\n", name(w), ms, r.chassis.interfered ? "interfered" : "clean",
                r.trace.back().avg, r.drive_speed_over(90));
    if (!r.chassis.interfered) {
      CHECK_MESSAGE(r.drive_speed_over(90) < r.drive_floor(90), "returned clean while moving at " << r.drive_speed_over(90) << " in/s");
      CHECK(std::fabs(24.0 - r.trace.back().avg) < 3.0);
    }
  }
}
