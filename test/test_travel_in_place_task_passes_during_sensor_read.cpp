// A wait asks whether the robot went nowhere over a window. When the auto task has missed a few passes the answer comes from the drive sensors, read live,
// and the task can pass while they are read. The tracker is then fresh, and the question is its to answer again: a robot that was cruising through the
// window and was then held for the length of the stall is not one that went nowhere, whatever the sensors said a moment before the sample landed.
#include "doctest.h"
#include "drive_test_access.hpp"
#include "exit_gate_rig.hpp"

using namespace ez;
using namespace gate;

namespace {
Rig* g_rig = nullptr;
int g_passes_landed = 0;

// The auto task passes once, in the middle of the first sensor read
void let_the_task_pass() {
  pros::motor_read_hook = nullptr;  // the pass reads the motors itself
  g_rig->sim.passes_per_tick(1);
  pros::delay(10);
  g_rig->sim.passes_per_tick(0);
  g_passes_landed++;
}
}  // namespace

TEST_CASE("a task pass landing while the sensors are read for a stale tracker does not turn a robot that was cruising into one that went nowhere") {
  Rig r(sim::archetype_light_fast(), 1);
  g_rig = &r;
  g_passes_landed = 0;
  r.chassis.pid_drive_set(48_in, 110);
  for (int i = 0; i < 30; i++) pros::delay(10);
  // Held still from here, the task passing a few times more and then not for 200 ms
  r.sim.pin(r.sim.now_ms(), 1e9);
  for (int i = 0; i < 3; i++) pros::delay(10);
  r.sim.passes_per_tick(0);
  for (int i = 0; i < 20; i++) pros::delay(10);

  // Without a pass the sensors show the robot where the tracker last saw it: the stale tracker has nothing to say, and it is taken as in place
  CHECK(DriveTestAccess::stale_state(r.chassis, true, true) == 0);

  pros::motor_read_hook = let_the_task_pass;
  bool in_place = DriveTestAccess::odom_travel_in_place(r.chassis, 300);
  pros::motor_read_hook = nullptr;
  g_rig = nullptr;
  REQUIRE(g_passes_landed == 1);
  CHECK_FALSE(in_place);
}
