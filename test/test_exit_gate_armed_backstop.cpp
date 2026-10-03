// A speed gate may only be armed when the wait loop that uses it has a backstop that really ends the wait. pid_wait_until(distance) on
// an odom move ends through the left and right stuck watches, which read the drive exits of the left and right PIDs. When the team
// turns off the odom drive velocity and current exits those watches are off, so the xy gate must not be armed from the heading
// exits' window or a robot hunting about its target is held for ever. Every wait is bounded in sim time here, and "no return by T"
// is a failure, not a blocking call.
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

constexpr int CAP_TICKS = 1500;  // 15 s of sim time, the start commit returns in about 2 s

enum class Wait {
  Distance,   // pid_wait_until(distance) the robot does not cross
  Full,       // pid_wait()
  Quick,      // pid_wait_quick()
  Point,      // pid_wait_until(pose)
  IndexStart  // pid_wait_until_index_started(0)
};

// Returns whether the wait came back, and when
bool odom_wait_returns(const sim::SimArchetype& a, int passes, int mA, Wait w, double* elapsed, bool reverse = true) {
  Rig r(a, passes, false, 1);
  r.chassis.pid_drive_constants_set(3000.0, 0.0, 0.0);
  r.chassis.pid_turn_constants_set(60.0, 0.0, 0.0);
  r.chassis.pid_swing_constants_set(60.0, 0.0, 0.0);
  r.chassis.pid_odom_angular_constants_set(60.0, 0.0, 0.0);
  // Odom drive velocity exit off, current exit as given, heading exits left at their defaults
  r.chassis.pid_odom_drive_exit_condition_set(90, 1, 250, 3, 0, mA);
  double x = reverse ? -24.0 : 24.0;
  r.chassis.pid_odom_set(odom{pose{x, 0, ANGLE_NOT_SET}, reverse ? rev : fwd, 127});
  bool ok = false;
  test_stub::capture_stdout([&]() {
    ok = r.wait(
        [&]() {
          switch (w) {
            case Wait::Distance:
              r.chassis.pid_wait_until(reverse ? 12.0 : 1.2 * x);  // wrong sign for a reverse move, or past the path: never crossed
              break;
            case Wait::Full:
              r.chassis.pid_wait();
              break;
            case Wait::Quick:
              r.chassis.pid_wait_quick();
              break;
            case Wait::Point:
              r.chassis.pid_wait_until(pose{x, 0, 0});
              break;
            case Wait::IndexStart:
              r.chassis.pid_wait_until_index_started(0);
              break;
          }
        },
        CAP_TICKS, elapsed);
  });
  return ok;
}

struct Hunter {
  const char* name;
  sim::SimArchetype arch;
  int passes;
};

std::vector<Hunter> hunters() { return {{"sticky", sim::archetype_sticky_high_friction(), 1}, {"heavy_slow 2 passes", sim::archetype_heavy_slow(), 2}}; }

}  // namespace

TEST_CASE("odom pid_wait_until(distance) with the odom drive velocity and current exits off returns on a hunting robot") {
  for (auto& h : hunters()) {
    double el = 0;
    bool ok = odom_wait_returns(h.arch, h.passes, 0, Wait::Distance, &el);
    CHECK_MESSAGE(ok, std::string(h.name) << ": no return by " << el << " ms");
  }
}

TEST_CASE("control: a distance past a forward path, same exits, returns") {
  for (auto& h : hunters()) {
    double el = 0;
    bool ok = odom_wait_returns(h.arch, h.passes, 0, Wait::Distance, &el, false);
    CHECK_MESSAGE(ok, std::string(h.name) << ": no return by " << el << " ms");
  }
}

TEST_CASE("control: the same wait with the odom drive current exit on returns") {
  for (auto& h : hunters()) {
    double el = 0;
    bool ok = odom_wait_returns(h.arch, h.passes, 750, Wait::Distance, &el);
    CHECK_MESSAGE(ok, std::string(h.name) << ": no return by " << el << " ms");
  }
}

TEST_CASE("the other odom waits return on a hunting robot with the odom drive velocity and current exits off") {
  Wait waits[] = {Wait::Full, Wait::Quick, Wait::Point, Wait::IndexStart};
  const char* names[] = {"pid_wait", "pid_wait_quick", "pid_wait_until(pose)", "pid_wait_until_index_started"};
  for (auto& h : hunters())
    for (int i = 0; i < 4; i++) {
      double el = 0;
      bool ok = odom_wait_returns(h.arch, h.passes, 0, waits[i], &el, false);
      CHECK_MESSAGE(ok, std::string(h.name) << " " << std::string(names[i]) << ": no return by " << el << " ms");
    }
}
