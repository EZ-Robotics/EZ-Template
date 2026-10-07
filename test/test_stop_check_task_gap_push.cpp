// A push that lands while the auto task is stalled must not read as "stopped". The stop check measures how far the robot travelled over
// the exit window; when the task misses a stretch of time, the first sample after it sees the whole stretch's movement at once. That
// movement has to be charged to the window, not spread over the gap, or a robot being shoved at 3 in/s reads as stopped at 1.5 in/s.
//
// Judged against the sim's own positions, sampled every tick including the ticks the auto task did not run.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

// Hands the sim's tick to a wrapper that starves the auto task over [from_ms, to_ms) and records the true state on every tick
struct Starver {
  static inline Rig* rig = nullptr;
  static inline void (*inner)() = nullptr;
  static inline double from_ms = 0, to_ms = 0;
  static void tick() {
    rig->record();
    double t = rig->sim.now_ms();
    rig->sim.passes_per_tick(t >= from_ms && t < to_ms ? 0 : 1);
    inner();
  }
  static void install(Rig& r, double from, double to) {
    rig = &r;
    from_ms = from;
    to_ms = to;
    inner = test_stub::g_clock.on_delay;
    test_stub::g_clock.on_delay = &Starver::tick;
  }
};

}  // namespace

TEST_CASE("a push during a stalled auto task is not read as a stopped robot") {
  Rig r(archetype_classroom(), 1, false);
  r.chassis.pid_print_toggle(true);
  Starver::install(r, 870, 1170);
  r.sim.push(20.0, 1110, 60);  // the last 60 ms of the gap
  r.chassis.pid_drive_set(12_in, 110);
  double elapsed = 0;
  bool ok = false;
  std::string out = test_stub::capture_stdout([&]() { ok = r.wait([&]() { r.chassis.pid_wait(); }, 1500, &elapsed); });
  REQUIRE(ok);
  // Whichever exit ended the wait judges "stopped" over its own window: the small exit's 90 ms, the big exit's 250 ms, or the team's
  // velocity window (500 ms by default) when the stuck watch settles it inside the big error
  int window_ms = out.find("Small Exit") != std::string::npos ? 90 : out.find("Big Exit") != std::string::npos ? 250 : 500;
  double speed = r.drive_speed_over(window_ms);
  MESSAGE(out);
  MESSAGE("elapsed=", elapsed, " interfered=", r.chassis.interfered, " true speed over the ", window_ms, " ms exit window=", speed, " in/s");
  if (!r.chassis.interfered) CHECK(speed < r.drive_floor(window_ms));
}
