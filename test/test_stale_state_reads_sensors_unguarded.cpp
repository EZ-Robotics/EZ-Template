// When the auto task has missed a few passes, the wait asks the drive sensors where the robot is now (Drive::stale_state(), and through it
// Drive::travel_in_place()). A sensor read can block on the sensor's port, which the daemon takes every time it runs, so no read may be made inside a
// KillSafeGuard: a competition task deleted while it holds the drive mutex leaves odometry and every motion stopped for good. The motor and IMU read hooks
// in the stub run where a task switch would land, inside the read, and see whether the drive mutex is held.
#include <algorithm>

#include "doctest.h"
#include "drive_test_access.hpp"
#include "exit_gate_rig.hpp"
#include "lock_test_access.hpp"

using namespace ez;
using namespace gate;

namespace {
Drive* g_chassis = nullptr;
int g_reads = 0;
int g_imu_reads = 0;
int g_deepest = 0;

void note_depth() { g_deepest = std::max(g_deepest, LockTestAccess::depth(DriveTestAccess::drive_mutex(*g_chassis))); }
void note_motor_read() {
  g_reads++;
  note_depth();
}
void note_imu_read() {
  g_imu_reads++;
  note_depth();
}

// A drive that is moving, whose auto task then stops passing for 250 ms: the stop tracker's newest sample is old enough for the sensors to be asked
struct StaleDrive {
  Rig r;
  StaleDrive() : r(sim::archetype_light_fast(), 1) {
    r.chassis.pid_drive_set(24_in, 110);
    for (int i = 0; i < 30; i++) pros::delay(10);
    r.sim.passes_per_tick(0);
    for (int i = 0; i < 25; i++) pros::delay(10);
    g_chassis = &r.chassis;
    g_reads = 0;
    g_imu_reads = 0;
    g_deepest = 0;
    pros::motor_read_hook = note_motor_read;
    pros::imu_read_hook = note_imu_read;
  }
  ~StaleDrive() {
    pros::motor_read_hook = nullptr;
    pros::imu_read_hook = nullptr;
    g_chassis = nullptr;
  }
};
}  // namespace

TEST_CASE("asking the sensors whether a stale tracker's robot has moved reads them with the drive mutex free") {
  StaleDrive s;
  int state = DriveTestAccess::stale_state(s.r.chassis, true, true);
  CAPTURE(state);
  REQUIRE(state != 0);
  REQUIRE(g_reads > 0);
  REQUIRE(g_imu_reads > 0);
  CHECK(g_deepest == 0);
}

TEST_CASE("asking whether the robot went nowhere over a window, with a stale tracker, reads the sensors with the drive mutex free") {
  StaleDrive s;
  DriveTestAccess::odom_travel_in_place(s.r.chassis, 300);
  REQUIRE(g_reads > 0);
  CHECK(g_deepest == 0);
}

TEST_CASE("asking the IMU whether a stale tracker's robot has turned reads it with the drive mutex free") {
  StaleDrive s;
  int state = DriveTestAccess::stale_state(s.r.chassis, false, true);
  CAPTURE(state);
  REQUIRE(g_imu_reads > 0);
  CHECK(g_deepest == 0);
}
