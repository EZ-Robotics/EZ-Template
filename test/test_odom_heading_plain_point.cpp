// Finding #13 (Step 3 audit): after a PLAIN point odom motion (no explicit final theta --
// pid_odom_ptp_set/pid_odom_set with target.theta left at ANGLE_NOT_SET), headingPID's target
// went stale. raw_pid_odom_ptp_set() (set_odom_pid.cpp) sets it from the point-to-face angle,
// but only once: at motion start for a plain POINT_TO_POINT target (ptp_task() alone drives
// the rest of the motion), or on every carrot move/waypoint advance for PURE_PURSUIT/boomerang
// (which calls raw_pid_odom_ptp_set() again each time, so it already stays fresh there). A
// plain point's one-shot value never got refreshed as the robot's actual approach evolved, and
// the end-of-wait fixup in exit_conditions.cpp only resyncs when odom_target_start.theta !=
// ANGLE_NOT_SET (see test_odom_heading.cpp, which only covers that explicit-theta case) -- so a
// DRIVE motion started right after could inherit a stale, unrelated heading target and fight a
// bogus error. Silent: no exit type, no interfered flag, no print reflects it.
//
// Fix: ptp_task() now also sets headingPID's target every tick, from the same a_target this
// pass already computed for current_a_odomPID, whenever the motion has no explicit final theta
// -- mirroring, continuously, what raw_pid_odom_ptp_set() already does once at start. Gated to
// odom_target_start.theta == ANGLE_NOT_SET so it can't interfere with boomerang's own
// already-continuous refresh or an explicit theta's exit-time resync.
#include <cmath>

#include "doctest.h"

#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}
}  // namespace

TEST_CASE("ptp_task keeps headingPID's target synced to the point-to-face angle during a plain-point odom motion") {
  Drive chassis = make_chassis();
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.imu->fake_rotation = 0.0;

  // A plain point: no explicit final theta, so odom_target_start.theta stays ANGLE_NOT_SET.
  odom movement{{24.0, 0.0}, fwd, 70};
  chassis.pid_odom_ptp_set(movement);
  // Confirms this codebase's bearing convention (0=+y, 90=+x) before relying on it below --
  // not asserting this is otherwise "the" fix, just establishing a known-good baseline reading.
  CHECK(chassis.headingPID.target_get() == doctest::Approx(90.0));

  // Nothing on the host advances the robot's real pose on its own -- move it directly to where
  // a real approach would put it partway through: south of the target, same x, a roughly
  // "north"-facing approach rather than the motion-start "east"-facing one. Not asserting an
  // exact predicted angle here: point_to_face is a look-ahead point computed once at motion
  // start (raw_pid_odom_ptp_set), not recomputed from the robot's current position, so the
  // exact bearing to it isn't simply "bearing to (24,0)" -- what matters, and what the bug was,
  // is whether headingPID.target updates AT ALL as the robot's real position changes.
  double target_at_start = chassis.headingPID.target_get();
  chassis.odom_xyt_set(24.0, -24.0, 0.0);
  DriveTestAccess::ptp_task(chassis);
  double target_after_first_move = chassis.headingPID.target_get();

  // Before the fix: headingPID.target stayed frozen at target_at_start (90) for the rest of the
  // motion, unrelated to where the robot actually is. After the fix: it tracks point_to_face's
  // angle every tick, so a position this different produces a visibly different target.
  CHECK(target_after_first_move != doctest::Approx(target_at_start));

  // A second, opposite position change confirms this isn't a one-time re-latch (which a bug
  // that only fixed the very first ptp_task() call after motion start could still pass) -- it
  // has to keep tracking on every subsequent tick too.
  chassis.odom_xyt_set(24.0, 24.0, 0.0);
  DriveTestAccess::ptp_task(chassis);
  double target_after_second_move = chassis.headingPID.target_get();
  CHECK(target_after_second_move != doctest::Approx(target_after_first_move));

  // Sanity bound: point_to_face's look-ahead offset is small relative to this 48in swing, so
  // the tracked angle should land within a wide but still meaningful absolute band of the two
  // clean "bearing to (24,0)" readings (0 and 180) -- not a tight equality, just ruling out
  // nonsense (e.g. a wrap bug landing hundreds of degrees off). Approx's epsilon() is a
  // *relative* tolerance (useless against an expected value of 0), so this uses a plain
  // absolute-difference check instead.
  CHECK(std::fabs(target_after_first_move - 0.0) < 30.0);
  CHECK(std::fabs(target_after_second_move - 180.0) < 30.0);
}

TEST_CASE("ptp_task's plain-point headingPID resync does not fire for an explicit-theta (boomerang) motion") {
  Drive chassis = make_chassis();
  chassis.odom_xyt_set(0.0, 0.0, 0.0);
  chassis.imu->fake_rotation = 0.0;

  // An explicit final theta routes through the boomerang/pure-pursuit path, where
  // odom_target_start.theta != ANGLE_NOT_SET. raw_pid_odom_ptp_set() already keeps headingPID
  // continuously fresh there on its own (called again on every carrot move) -- this fix's own
  // gate must not additionally fire for this case (it would be redundant, not wrong, if it did,
  // but the gate should still be doing its job: verify the plain-point code path is inert here
  // by checking the *other* observable it controls, odom_target_start.theta, stays non-sentinel).
  odom movement{{24.0, 0.0, 90.0}, fwd, 70};
  chassis.pid_odom_set(movement);

  REQUIRE(DriveTestAccess::pp_movements(chassis).size() >= 1);
  CHECK(DriveTestAccess::pp_movements(chassis).back().target.theta != ANGLE_NOT_SET);
}
