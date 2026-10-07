// A side that latched a window exit and then moved is not settled, whatever the other side's stuck watch says.
//
// When one side is still waiting and its stuck watch gives up, the wait counts the robot as settled if every side is inside its big error. A side
// that had already latched an exit is not looked at again before that: it only stopped being fed to its watch when it latched, and the wait
// trusted the exit it gave. The double-latch recheck (both sides latched, one since shoved or moving) never runs on this path, because one side
// is still waiting. So a robot shoved by a push and spun on top of it came back clean at 680 ms, still moving at 5.9 in/s and 1.2 in short of
// its target, by every drive wait.
//
// The latched side is rechecked now, before it counts as settled: its live error still inside the band its exit was given for, and the gate not
// seeing the side move since. If either fails the exit is taken back and the wait goes on, with the side's watch given a fresh clock, as the
// recheck of a double latch does, bounded by the same cap.
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

const char* name(Wait w) { return w == Wait::Full ? "pid_wait()" : w == Wait::Quick ? "pid_wait_quick()" : "pid_wait_until(-24_in)"; }

void run(Wait w, Rig& r) {
  switch (w) {
    case Wait::Full:
      r.chassis.pid_wait();
      break;
    case Wait::Quick:
      r.chassis.pid_wait_quick();
      break;
    case Wait::Until:
      r.chassis.pid_wait_until(-24_in);
      break;
  }
}

}  // namespace

TEST_CASE("a side that latched and was then pushed and spun is not counted as settled by any drive wait while the robot is still moving") {
  for (Wait w : {Wait::Full, Wait::Quick, Wait::Until}) {
    Rig r(sim::archetype_light_fast(), 1, false, 1);
    // 2550R's exits
    r.chassis.pid_drive_exit_condition_set(90, 1, 200, 3, 100, 100);
    r.chassis.pid_turn_exit_condition_set(90, 1, 200, 3, 100, 100);
    r.sim.push(30.0, 392, 342);
    r.sim.carry(0.0, 52.0, 494, 84);
    r.chassis.pid_drive_set(-24_in, 110);
    double ms = 0;
    bool ok = r.wait([&] { run(w, r); }, 3000, &ms);
    CAPTURE(std::string(name(w)));
    REQUIRE(ok);
    std::printf("  [%s] returned at %.0f ms, %s, %.2f in, %.2f in/s over 90 ms\n", name(w), ms, r.chassis.interfered ? "interfered" : "clean",
                r.trace.back().avg, r.drive_speed_over(90));
    if (!r.chassis.interfered) {
      CHECK_MESSAGE(r.drive_speed_over(90) < r.drive_floor(90), "returned clean while moving at " << r.drive_speed_over(90) << " in/s");
      CHECK(std::fabs(-24.0 - r.trace.back().avg) < 3.0);
    }
  }
}
