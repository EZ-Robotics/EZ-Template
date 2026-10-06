// A held mA exit has to end. An mA exit is taken once the robot is stopped over the mA window, and a robot that never reads stopped while it
// stays over current would hold it for ever: one that limit cycles about its target (the tracker adds up every swing, so it never reads stopped
// however little it gets anywhere) or one that is dragged away from it. What the hold is for is a robot that is still GETTING somewhere (a heavy
// robot accelerating hard draws over current the whole way), so an mA exit that finds the robot not stopped is released as soon as, over one mA
// window, the error to the target has not come down by the stop speed times that window (1.5 in/s, 4 deg/s). A robot that is closing at the
// stop speed or faster is still held. Stopped still releases it at once.
//
// Every wait is capped in sim time, so "no return" is a failure and not a hung suite.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

// The auto task only runs every `period`th tick, and every drive motor reads over current on every tick
sim::SimRobot* g_sim = nullptr;
Drive* g_drive = nullptr;
void (*g_sim_tick)() = nullptr;
int g_period = 0, g_tick = 0;
bool g_over_current = false;

void hook() {
  if (g_period > 0) g_sim->passes_per_tick((g_tick++ % g_period) == 0 ? 1 : 0);
  g_sim_tick();
  if (g_over_current) {
    for (auto& m : g_drive->left_motors) m.fake().over_current = true;
    for (auto& m : g_drive->right_motors) m.fake().over_current = true;
  }
}

struct HookGuard {
  HookGuard(Rig& r, int period, bool over_current) {
    g_sim = &r.sim;
    g_drive = &r.chassis;
    g_sim_tick = test_stub::g_clock.on_delay;
    g_period = period;
    g_tick = 0;
    g_over_current = over_current;
    test_stub::g_clock.on_delay = &hook;
  }
  ~HookGuard() { test_stub::g_clock.on_delay = g_sim_tick; }
};

void set_all_exits(Drive& c, int st, double se, int bt, double be, int vt, int ma, int ast, double ase, int abt, double abe, int avt, int ama) {
  c.pid_drive_exit_condition_set(250, 1.5, 1000, 2.0, 0, 2000);
  c.pid_turn_exit_condition_set(40, 0.5, 500, 2.0, 2000, 100);
  c.pid_odom_drive_exit_condition_set(st, se, bt, be, vt, ma);
  c.pid_odom_turn_exit_condition_set(ast, ase, abt, abe, avt, ama);
}

}  // namespace

// The scenario the termination attack found (the heading limit cycles and creeps about 0.2 deg/s, outside its big_error): sticky_high_friction, noise seed 44, the auto task running every 5th tick, the motors reading over
// current on every tick, an odom point to (1, 1) with an end heading of 90 and a wall in front of it. xy exits (50, 0.75, 250, 3, 500, 2000) and
// angle exits (250, 0, 200, 1, 500, 100). The previous head returned at 2020 ms interfered (the mA exit); held for ever it never returned.
TEST_CASE("a held mA exit on a sticky robot whose task runs every 5th tick returns within two mA windows") {
  for (double wall : {0.5, 1.5, 2.5}) {
    Rig r(sim::archetype_sticky_high_friction(), 1, true, 44);
    set_all_exits(r.chassis, 50, 0.75, 250, 3.0, 500, 2000, 250, 0.0, 200, 1.0, 500, 100);
    r.chassis.pid_odom_set({{1_in, 1_in, 90_deg}, fwd, 90}, true);
    r.sim.wall(wall);
    HookGuard guard(r, 5, true);
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait(); }, 3000, &ms);
    CAPTURE(wall);
    CHECK_MESSAGE(ok, "pid_wait() did not return within 30 s");
    // The bound: the xy mA window (2000 ms) to the latch, then one window (2000 ms) without the error coming down by the stop speed times it
    CHECK_MESSAGE(ms < 5000.0, "returned after " << ms << " ms");
    CHECK(r.chassis.interfered);
  }
}

// heavy_slow, every exit off but a 100 ms mA timeout, an odom point, the robot pinned for 122 ms at 10% of the way and then carried backwards
// at 5 in/s for ever: it is moving the whole time, and over current. The previous head's hold took 10.7 s; the tip returned at 0.6 s.
TEST_CASE("a heavy robot pinned and then dragged away at 5 in/s with only a 100 ms mA exit returns within 1.5 s of contact") {
  for (int passes : {2, 3}) {
    Rig r(sim::archetype_heavy_slow(), passes, false, 1);
    r.chassis.pid_odom_drive_exit_condition_set(0, 0, 0, 0, 0, 100);
    r.chassis.pid_odom_turn_exit_condition_set(0, 0, 0, 0, 0, 100);
    r.chassis.pid_odom_set({{12_in, 24_in}, fwd, 110}, true);
    double length = std::hypot(12.0, 24.0);
    bool fired = false;
    double contact_ms = 0;
    r.sim.before_pass = [&](int) {
      if (fired) return;
      pose p = r.chassis.odom_pose_get();
      if (std::hypot(p.x, p.y) >= 0.1 * length) {
        fired = true;
        contact_ms = r.sim.now_ms();
        r.sim.pin(contact_ms, 122);
        r.sim.carry(-5.0, 0.0, contact_ms + 122, 1e7);
      }
    };
    double ms = 0;
    bool ok = r.wait([&] { r.chassis.pid_wait(); }, 1500, &ms);
    CAPTURE(passes);
    REQUIRE(fired);
    CHECK_MESSAGE(ok, "pid_wait() did not return within 15 s");
    CHECK_MESSAGE(ms - contact_ms < 1500.0, "returned " << ms - contact_ms << " ms after contact");
  }
}
