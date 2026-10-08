// A robot's own overshoot is not a shove, however far it goes.
//
// The progress watch behind every wait credits a shove once per disturbance: when the error is pushed a full step worse the no-progress clock
// restarts and the recovery from the peak counts step by step. A robot that overshoots its target on its own rises past it too, and a rule
// that called any rise of two steps or more past where the robot stood a shove spent that credit on the overshoot, so the first real shove after
// it got no restart and no recovery credit: the wait ended stuck while the robot was still being flung, or went on to call it stuck after it
// had been knocked tens of inches away.
//
// Part 1 scripts the error and watches pid_wait() on a drive: a robot that overshoots by two to five steps, settles, and is shoved. Part 2
// runs the same on a light robot through the sim, judged against the sim's own state. Part 3 is the guard that goes with it: a robot
// hunting back and forth across its target at a fixed amplitude, never getting anywhere, must still be ended.
#include <cmath>
#include <functional>
#include <string>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {

Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

Drive* g_chassis = nullptr;
int g_pass = 0;
std::function<void(Drive&, int)> g_script;

void on_delay() {
  ++g_pass;
  ez::detail::stats.auto_task_passes.fetch_add(1);
  g_script(*g_chassis, g_pass);
}

void set_error(Drive& c, double error, double derivative) {
  c.leftPID.error = error;
  c.leftPID.derivative = derivative;
  c.rightPID.error = error;
  c.rightPID.derivative = derivative;
}

// Close from 30 to 0.4 (0.2 per pass), then, if `depth` is not 0, go past the target to -depth over 12 passes and come back to 0 over 20 and sit
// there. At pass 220 a shove takes the error from 0 to 10.5 over 10 passes, and the robot recovers slowly (0.1 per pass) back to 0.
void shove_script(Drive& c, int n, double depth) {
  double error, derivative;
  if (n <= 148) {
    error = 30.0 - 0.2 * n;
    derivative = -0.2;
  } else if (n <= 180) {
    if (depth > 0.0)
      error = n <= 160 ? 0.4 - (0.4 + depth) * (n - 148) / 12.0 : -depth + depth * (n - 160) / 20.0;
    else
      error = 0.4 - 0.4 * (n - 148) / 32.0;
    derivative = -0.1;
  } else if (n <= 220) {
    error = 0.0;
    derivative = 0.0;
  } else if (n <= 230) {
    error = 1.05 * (n - 220);
    derivative = 1.05;
  } else {
    error = std::fmax(0.0, 10.5 - 0.1 * (n - 230));
    derivative = error > 0.0 ? -0.1 : 0.0;
  }
  set_error(c, error, derivative);
}

struct Result {
  bool returned;
  bool interfered;
  int passes;
};

Result run_script(std::function<void(Drive&, int)> script, int cap) {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.pid_drive_set(24, 100);
  g_chassis = &chassis;
  g_pass = 0;
  g_script = script;
  script(chassis, 0);
  test_stub::g_clock.on_delay = on_delay;
  test_stub::g_clock.delay_calls_until_stop = cap;
  bool returned = true;
  try {
    chassis.pid_wait();
  } catch (test_stub::StopLoop&) {
    returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  return {returned, chassis.interfered, g_pass};
}

}  // namespace

TEST_CASE("a shove after the robot overshot by more than two steps is recovered from, as it is without the overshoot") {
  Result without = run_script([](Drive& c, int n) { shove_script(c, n, 0.0); }, 600);
  REQUIRE(without.returned);
  CHECK_FALSE(without.interfered);
  for (double depth : {2.5, 3.0, 4.0, 5.0}) {
    Result with = run_script([=](Drive& c, int n) { shove_script(c, n, depth); }, 600);
    INFO("overshoot of ", depth, ": returned=", with.returned, " interfered=", with.interfered, " passes=", with.passes);
    REQUIRE(with.returned);
    CHECK_FALSE(with.interfered);
  }
}

TEST_CASE("a light robot that overshoots its drive target by more than two steps and is then shoved is waited out") {
  // Team constants (small error 0.5, step 0.5), 3 passes per poll, speed 127: the robot crosses its target at about 320 ms and overshoots by 1.6 in,
  // over three steps. A 150 N shove back from it, 300 ms long, starting 100 to 400 ms after the crossing, used to end the wait stuck with the robot 24 in
  // away and moving at 44 in/s.
  for (double start : {330.0, 380.0, 420.0, 500.0, 600.0, 700.0}) {
    Rig r(sim::archetype_light_fast(), 3, false);
    r.chassis.pid_print_toggle(false);
    r.chassis.pid_drive_exit_condition_set(100, 0.5, 300, 2, 300, 1000);
    r.chassis.pid_drive_set(24_in, 127);
    r.sim.push(-150.0, start, 300.0);
    double elapsed = 0;
    bool ok = r.wait([&]() { r.chassis.pid_wait(); }, 1500, &elapsed);
    double speed = r.drive_speed_over(100);
    double off = std::fabs(24.0 - r.trace.back().avg);
    INFO("shove at ", start, " ms: elapsed=", elapsed, " ms, interfered=", r.chassis.interfered, ", true speed=", speed, " in/s, true error=", off, " in");
    REQUIRE(ok);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(speed < r.drive_floor(100));
    CHECK(off < 2.0);
  }
}

TEST_CASE("a robot hunting across its target at a fixed amplitude is still ended after a deep first overshoot") {
  // Close to 0.3, then swing between +5 and -5 forever with a 30 pass period: it crosses the target twice a cycle and never gets anywhere
  auto hunt = [](Drive& c, int n) {
    double error, derivative;
    if (n <= 100) {
      error = 20.0 - 0.2 * n;
      derivative = -0.2;
    } else {
      int m = (n - 100) % 30;
      double t = m < 15 ? m / 15.0 : 2.0 - m / 15.0;
      error = 0.3 + 4.7 * std::sin(t * M_PI);
      if ((n - 100) % 60 >= 30) error = -error;
      derivative = 0.5;
    }
    set_error(c, error, derivative);
  };
  Result r = run_script(hunt, 1000);
  REQUIRE(r.returned);
  CHECK(r.interfered);
  CHECK(r.passes < 260);
}
