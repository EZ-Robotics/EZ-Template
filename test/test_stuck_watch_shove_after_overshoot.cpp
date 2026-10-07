// A robot that overshoots its target and comes back has not used up the allowance for being shoved.
//
// The progress watch behind every wait credits a shove once per disturbance: when the error is pushed a full step worse the no-progress
// clock restarts, and the recovery from the peak counts step by step, so a robot being pushed away and recovering fast is not called
// stuck while it does. Crossing the target is the robot's own overshoot, and used to switch that allowance off for good: the latch only
// cleared after real new headway, which a robot already at its target cannot make. The first real shove after any overshoot then got
// no restart and no recovery credit, and a robot recovering from it was called stuck, or one that was still moving was called
// settled.
//
// Part 1 scripts the error the way test_stuck_watch_second_shove_gets_fresh_leniency.cpp does and watches pid_wait() on a drive.
// Part 2 runs shoves through the sim on a swing and a turn with team constants, and judges the return against the
// sim's own state. Part 3 is the guard that goes with it: a robot hunting back and forth across its target at a fixed amplitude, never
// getting anywhere, must still be ended (and in the same time with or without the first overshoot being credited).
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

// Close from 30 to 0.4 (0.2 per pass), then, if `overshoot`, go past the target to -2 and come back to 0 and sit there. At pass 220 a shove
// takes the error from 0 to 10.5 over 10 passes, and the robot recovers slowly (0.1 per pass, the size of a 1 in/s recovery) back to 0.
// The error is the drive's error, so the sign is the side of the target the robot is on.
void shove_script(Drive& c, int n, bool overshoot) {
  double error, derivative;
  if (n <= 148) {
    error = 30.0 - 0.2 * n;  // 30 -> 0.4
    derivative = -0.2;
  } else if (n <= 180) {
    // Past the target to -2 and back to 0 (overshoot), or just on in to 0 (no overshoot)
    if (overshoot)
      error = n <= 160 ? 0.4 - 2.4 * (n - 148) / 12.0 : -2.0 + 2.0 * (n - 160) / 20.0;
    else
      error = 0.4 - 0.4 * (n - 148) / 32.0;
    derivative = -0.1;
  } else if (n <= 220) {
    error = 0.0;
    derivative = 0.0;
  } else if (n <= 230) {
    error = 1.05 * (n - 220);  // 0 -> 10.5
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

TEST_CASE("a shove after the robot overshot its target is recovered from, as it is without the overshoot") {
  Result with = run_script([](Drive& c, int n) { shove_script(c, n, true); }, 600);
  Result without = run_script([](Drive& c, int n) { shove_script(c, n, false); }, 600);
  MESSAGE("with overshoot: returned=", with.returned, " interfered=", with.interfered, " passes=", with.passes);
  MESSAGE("without overshoot: returned=", without.returned, " interfered=", without.interfered, " passes=", without.passes);
  REQUIRE(with.returned);
  REQUIRE(without.returned);
  // The slow recovery from a 10.5 shove is progress the whole way, so neither run is stuck, and the overshoot changes nothing about it
  CHECK_FALSE(without.interfered);
  CHECK_FALSE(with.interfered);
}

TEST_CASE("a swing or turn shoved while it settles ends clean at its target, at rest, not mid motion") {
  // Team constants (small exit time 100 ms, big exit time 300 ms, velocity window 300 ms, mA 1000 ms) with two pairs of errors. A 150 N shove
  // pushes the robot through its target and the wait has to see it come back: each row is a shove time (ms) that used to end the wait
  // while the robot was still on its way back, interfered 20 degrees off or clean at 10 degrees per second.
  struct Row {
    bool swing;
    double small_e, big_e;
    double shove_ms;
  };
  const Row rows[] = {{true, 1.0, 4.0, 350},  {true, 1.0, 4.0, 400},  {true, 1.0, 4.0, 450}, {false, 0.5, 2.0, 600},
                      {false, 0.5, 2.0, 650}, {false, 1.0, 4.0, 600}, {false, 1.0, 4.0, 700}};
  for (const Row& row : rows) {
    Rig r(sim::archetype_light_fast(), row.swing ? 3 : 1, false);
    r.chassis.pid_print_toggle(false);
    double target = row.swing ? 60.0 : 90.0;
    if (row.swing) {
      r.chassis.pid_swing_exit_condition_set(100, row.small_e, 300, row.big_e, 300, 1000);
      r.chassis.pid_swing_set(ez::LEFT_SWING, target, 110);
      r.sim.push(150.0, row.shove_ms, 150.0);
    } else {
      r.chassis.pid_turn_exit_condition_set(100, row.small_e, 300, row.big_e, 300, 1000);
      r.chassis.pid_turn_set(target, 110);
      r.sim.push(150.0, row.shove_ms, 500.0);
    }
    double elapsed = 0;
    bool ok = r.wait([&]() { r.chassis.pid_wait(); }, 1500, &elapsed);
    double speed = r.angle_speed_over(100);
    double off = std::fabs(target + r.sim.heading_deg());
    INFO(std::string(row.swing ? "swing" : "turn"), " small ", row.small_e, " big ", row.big_e, " shove at ", row.shove_ms, " ms: elapsed=", elapsed,
         " ms, interfered=", r.chassis.interfered, ", true speed=", speed, " deg/s, true error=", off, " deg");
    REQUIRE(ok);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(speed < r.angle_floor(100));
    CHECK(off < row.big_e);
  }
}

TEST_CASE("a robot hunting across its target at a fixed amplitude is still ended, overshoot credited or not") {
  // Close to 0.3, then swing between +5 and -5 forever with a 30 pass period: it crosses the target twice a cycle and never gets anywhere
  auto hunt = [](Drive& c, int n) {
    double error, derivative;
    if (n <= 100) {
      error = 20.0 - 0.2 * n;
      derivative = -0.2;
    } else {
      int m = (n - 100) % 30;
      double t = m < 15 ? m / 15.0 : 2.0 - m / 15.0;  // 0..1..0
      error = 0.3 + 4.7 * std::sin(t * M_PI);
      if ((n - 100) % 60 >= 30) error = -error;
      derivative = 0.5;
    }
    set_error(c, error, derivative);
  };
  Result r = run_script(hunt, 1000);
  MESSAGE("hunting: returned=", r.returned, " interfered=", r.interfered, " passes=", r.passes);
  REQUIRE(r.returned);
  // Ended as stuck, and in a few windows (166 passes when written), not re-armed by every crossing
  CHECK(r.interfered);
  CHECK(r.passes < 260);
}
