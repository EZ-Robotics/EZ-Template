// drive_sensor_left_raw()/drive_sensor_right_raw() (drive.cpp) and tracking_wheel::get_raw()
// (tracking_wheel.cpp) used to return whatever the underlying motor/rotation-sensor read
// produced with no PROS_ERR/PROS_ERR_F sentinel check -- unlike drive_imu_get(), which already
// falls back to a remembered last_good_angle on a bad imu read. A single failed read got
// integrated as a raw position delta by tracking.cpp with no bound, corrupting the persistent
// odometry pose by an enormous amount in one tick. Worse, drive_sensor_left_raw()/right_raw()
// return int, so a PROS_ERR_F motor read (a non-finite double) hit an implicit int conversion
// on the way out -- undefined behavior, not just a wrong number.
//
// Fix mirrors drive_imu_get()'s pattern: on a PROS_ERR/PROS_ERR_F/non-finite raw read, don't
// feed the sentinel (or cast it) into tracking math -- fall back to the last known-good raw
// reading for that sensor, remembered across calls. These tests cover all three guarded call
// sites directly (the tracking-wheel path, the plain-motor-encoder path, and the dedicated
// rotation-sensor path -- the three ways odometry actually consumes a raw reading), then
// reproduce the pose corruption end to end through ez_tracking_task(), then check that a
// *persistent* fault still eventually surfaces through an existing exit instead of the wait
// hanging forever or silently succeeding.
#include <cmath>
#include <cstdint>
#include <functional>

#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
Drive make_chassis() {
  test_stub::reset_all();
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}
}  // namespace

// --- tracking_wheel::get_raw() -----------------------------------------------------------

TEST_CASE("tracking_wheel get_raw() falls back to the last good reading on a PROS_ERR sentinel") {
  tracking_wheel tracker(1, 3.25, 0.0);  // Rotation-sensor variant
  tracker.smart_encoder.fake_position = 500;
  CHECK(tracker.get_raw() == doctest::Approx(500.0));
  CHECK(tracker.last_read_ok());

  tracker.smart_encoder.fake_position = INT32_MAX;  // PROS_ERR: the read itself failed
  CHECK(tracker.get_raw() == doctest::Approx(500.0));  // not INT32_MAX
  CHECK_FALSE(tracker.last_read_ok());
}

TEST_CASE("tracking_wheel get_raw() resumes real readings the pass after the fault clears") {
  tracking_wheel tracker(1, 3.25, 0.0);
  tracker.smart_encoder.fake_position = 500;
  tracker.get_raw();

  tracker.smart_encoder.fake_position = INT32_MAX;
  CHECK(tracker.get_raw() == doctest::Approx(500.0));

  tracker.smart_encoder.fake_position = 900;  // fault clears, sensor reads a genuine value again
  CHECK(tracker.get_raw() == doctest::Approx(900.0));
  CHECK(tracker.last_read_ok());
}

TEST_CASE("tracking_wheel get_raw() falls back to 0 when the very first read is already PROS_ERR") {
  tracking_wheel tracker(1, 3.25, 0.0);
  tracker.smart_encoder.fake_position = INT32_MAX;
  CHECK(tracker.get_raw() == doctest::Approx(0.0));
}

TEST_CASE("tracking_wheel reset() clears the remembered last-good reading along with the sensor") {
  tracking_wheel tracker(1, 3.25, 0.0);
  tracker.smart_encoder.fake_position = 500;
  tracker.get_raw();

  tracker.reset();  // zeroes the physical sensor
  tracker.smart_encoder.fake_position = INT32_MAX;  // then a fault hits before any post-reset read
  CHECK(tracker.get_raw() == doctest::Approx(0.0));  // falls back to the fresh zero, not the stale 500
}

// --- Drive::drive_sensor_left_raw()/right_raw(): the plain motor-encoder path -------------

TEST_CASE("drive_sensor_left_raw()/right_raw() fall back to the last good reading on a PROS_ERR_F motor read") {
  Drive chassis = make_chassis();
  chassis.left_motors[0].fake().position = 1000;
  chassis.right_motors[0].fake().position = 1000;
  CHECK(chassis.drive_sensor_left_raw() == 1000);
  CHECK(chassis.drive_sensor_right_raw() == 1000);

  chassis.left_motors[0].fake().disconnected = true;  // get_position() now returns PROS_ERR_F
  chassis.right_motors[0].fake().disconnected = true;
  CHECK(chassis.drive_sensor_left_raw() == 1000);  // not garbage from casting infinity to int
  CHECK(chassis.drive_sensor_right_raw() == 1000);
}

TEST_CASE("drive_sensor_left_raw()/right_raw() resume real readings once the fault clears") {
  Drive chassis = make_chassis();
  chassis.left_motors[0].fake().position = 1000;
  chassis.drive_sensor_left_raw();

  chassis.left_motors[0].fake().disconnected = true;
  CHECK(chassis.drive_sensor_left_raw() == 1000);

  chassis.left_motors[0].fake().disconnected = false;
  chassis.left_motors[0].fake().position = 1400;
  CHECK(chassis.drive_sensor_left_raw() == 1400);
}

// --- Drive::drive_sensor_left_raw()/right_raw(): the dedicated rotation-sensor path -------
// (is_tracker == DRIVE_ROTATION -- left_rotation/right_rotation stand in for the drive
// encoders entirely, so this path's sentinel is PROS_ERR from pros::Rotation, not
// PROS_ERR_F from a motor.)

TEST_CASE("drive_sensor_left_raw()/right_raw() fall back to the last good reading on a PROS_ERR rotation-sensor read") {
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 1.0, 6, 7);  // deprecated rotation-sensor constructor
  chassis.left_rotation.fake_position = 2000;
  chassis.right_rotation.fake_position = 2000;
  CHECK(chassis.drive_sensor_left_raw() == 2000);
  CHECK(chassis.drive_sensor_right_raw() == 2000);

  chassis.left_rotation.fake_position = INT32_MAX;  // PROS_ERR
  chassis.right_rotation.fake_position = INT32_MAX;
  CHECK(chassis.drive_sensor_left_raw() == 2000);  // not INT32_MAX
  CHECK(chassis.drive_sensor_right_raw() == 2000);
}

// --- End to end through tracking.cpp: the pose-corruption repro --------------------------

TEST_CASE("ez_tracking_task(): a single PROS_ERR tick from a tracking wheel does not corrupt the pose") {
  Drive chassis = make_chassis();
  tracking_wheel left_tracker(1, 3.25, 3.5);
  tracking_wheel back_tracker(2, 3.25, 2.0);
  chassis.odom_tracker_left_set(&left_tracker);
  chassis.odom_tracker_back_set(&back_tracker);

  chassis.odom_xyt_set(0.0, 0.0, 90.0);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.drive_sensor_reset();  // primes l_last/h_last/t_last to the current (good) readings
  chassis.ez_tracking_task();
  pose start = chassis.odom_pose_get();
  CHECK(std::hypot(start.x, start.y) == doctest::Approx(0.0).epsilon(1e-6));

  // One tick where the left tracker's read fails outright. Before the fix, this fed the raw
  // PROS_ERR ticks straight into tracking.cpp's position-delta math -- hundreds of thousands
  // of inches of "travel" from a single bad reading.
  left_tracker.smart_encoder.fake_position = INT32_MAX;
  chassis.ez_tracking_task();

  pose after_fault = chassis.odom_pose_get();
  double moved_during_fault = std::hypot(after_fault.x - start.x, after_fault.y - start.y);
  // Nowhere near a sentinel-driven jump -- a real single tick of travel is at most a few
  // inches, so anything past, say, 12in here means the sentinel still leaked through.
  CHECK(moved_during_fault < 12.0);

  // The fault clears next pass: the tracker reads a genuine, small, real advance again, and
  // that reading has to actually get used (not still stuck on the frozen fallback, and not a
  // second huge jump from comparing against the corrupted pose).
  left_tracker.smart_encoder.fake_position = 200;  // a genuine small advance, not another fault
  chassis.ez_tracking_task();
  pose after_recovery = chassis.odom_pose_get();
  double moved_during_recovery = std::hypot(after_recovery.x - after_fault.x, after_recovery.y - after_fault.y);
  CHECK(moved_during_recovery > 1e-6);  // the real reading actually moved the pose
  CHECK(moved_during_recovery < 1.0);   // by a normal small amount, not a jump
}

TEST_CASE("ez_tracking_task(): a single PROS_ERR_F tick from a plain drive motor encoder does not corrupt the pose") {
  // No dedicated tracking wheels here -- odometry runs straight off the drive motors'
  // integrated encoders, through drive_sensor_left()/right() -> drive_sensor_left_raw()/
  // right_raw(), the plain motor-encoder path guarded above. A 600-tick cartridge (rather
  // than make_chassis()'s 360) is used so the resulting jump is easy to sanity-check by hand.
  test_stub::reset_all();
  Drive chassis({1, -2}, {-3, 4}, 5, 3.25, 600, 1.0);
  chassis.odom_xyt_set(0.0, 0.0, 90.0);
  DriveTestAccess::imu_calibration_complete(chassis) = true;
  chassis.drive_sensor_reset();
  chassis.ez_tracking_task();
  pose start = chassis.odom_pose_get();

  chassis.left_motors[0].fake().disconnected = true;  // get_position() now returns PROS_ERR_F
  chassis.ez_tracking_task();
  pose after_fault = chassis.odom_pose_get();
  double moved_during_fault = std::hypot(after_fault.x - start.x, after_fault.y - start.y);
  CAPTURE(moved_during_fault);  // printed only on failure -- the pre-fix magnitude, for the record
  CHECK(moved_during_fault < 12.0);

  chassis.left_motors[0].fake().disconnected = false;
  double ticks = 0.2 * chassis.drive_tick_per_inch();  // a genuine, small real advance
  chassis.left_motors[0].fake().position = (std::int32_t)ticks;
  chassis.ez_tracking_task();
  pose after_recovery = chassis.odom_pose_get();
  double moved_during_recovery = std::hypot(after_recovery.x - after_fault.x, after_recovery.y - after_fault.y);
  CHECK(moved_during_recovery > 1e-6);
  CHECK(moved_during_recovery < 1.0);
}

// --- A PERSISTENT fault must still surface as stuck, not hang forever ---------------------
//
// These drive a real pid_wait()/wait_until_drive() DRIVE motion with both drive motors
// disconnected from before the motion starts, so drive_sensor_left()/right() run through the
// guarded call site on every pass for the whole test -- confirming the fix's frozen-but-finite
// fallback still lets an existing exit end the wait, rather than the fallback being read as
// permanently "fine" and the wait either hanging or silently succeeding. (A constant reading
// -- garbage or the fix's fallback -- reads as zero velocity either way, so this pair likely
// ends on the velocity exit both before and after the fix; they mainly guard against the fix
// regressing that. The mid-wait test below is the one that actually tells pre-fix and post-fix
// behavior apart.)

namespace {
Drive* g_chassis = nullptr;
int g_pass = 0;
int g_disconnect_at_pass = -1;  // -1: never (fault, if any, is set up before the wait starts)

// Mirrors the real ez_auto_task() pass this test isn't running: advances the auto-task pass
// counter StuckWatch/SingleStuckWatch key their starvation fallback off of, and recomputes
// leftPID/rightPID's error from the (guarded) sensor reads the same way drive_pid_task() would.
// When g_disconnect_at_pass is set, the fault starts mid-wait instead of being present from
// the first pass, so the wait's own sign-tracking (wait_until_drive()'s l_sgn/r_sgn, captured
// while the sensor was still healthy) sees the fault, not just the backstop.
void auto_task_pass() {
  ++g_pass;
  if (g_pass == g_disconnect_at_pass) {
    g_chassis->left_motors[0].fake().disconnected = true;
    g_chassis->right_motors[0].fake().disconnected = true;
  }
  ez::detail::stats.auto_task_passes.fetch_add(1, std::memory_order_relaxed);
  DriveTestAccess::drive_pid_task(*g_chassis);
}

struct Outcome {
  bool returned;
  int passes;
  bool interfered;
};

Outcome run_with_fault(Drive& chassis, int max_passes, const std::function<void()>& wait) {
  g_chassis = &chassis;
  g_pass = 0;
  test_stub::g_clock.on_delay = auto_task_pass;
  test_stub::g_clock.delay_calls_until_stop = max_passes;
  Outcome o{true, 0, false};
  try {
    wait();
  } catch (test_stub::StopLoop&) {
    o.returned = false;
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
  test_stub::g_clock.on_delay = nullptr;
  g_disconnect_at_pass = -1;
  o.passes = g_pass;
  o.interfered = chassis.interfered;
  return o;
}
}  // namespace

TEST_CASE("wait_until_drive(): a sensor fault starting mid-wait does not silently report a false success") {
  // Both motors are healthy until pass 5, then fail for the rest of the wait -- unlike the
  // "faulted from the start" tests above, wait_until_drive() captures l_sgn/r_sgn from a
  // genuinely healthy first reading, so this exercises whether a fault landing mid-motion can
  // still flip that sign comparison and read as "past the target" (a silent false success)
  // instead of being caught.
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  double left_before = chassis.drive_sensor_left();
  chassis.pid_drive_set(-24, 100);
  g_disconnect_at_pass = 5;

  Outcome o = run_with_fault(chassis, 3000, [&] { chassis.pid_wait_until(-12.0); });

  CHECK(o.returned);
  // Both motors were disconnected before the robot ever actually moved, so it never got
  // anywhere near -12in -- a "Success" here would be entirely spurious.
  CHECK(std::fabs(chassis.drive_sensor_left() - left_before) < 1.0);
  CHECK(o.interfered);    // must be flagged as stuck/interfered, not a silent false success
  CHECK(o.passes < 300);  // bounded, not a hang
}

TEST_CASE("pid_wait() DRIVE: a persistent sensor fault from the very start of the move still ends the wait as stuck, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.left_motors[0].fake().disconnected = true;
  chassis.right_motors[0].fake().disconnected = true;
  chassis.pid_drive_set(24, 100);

  Outcome o = run_with_fault(chassis, 3000, [&] { chassis.pid_wait(); });
  CHECK(o.returned);      // must not hang forever on the frozen value
  CHECK(o.interfered);    // a persistent fault is a real problem, not a silent false success
  CHECK(o.passes < 300);  // well under the STUCK_STARVED_WINDOWS wall-clock fallback's cap
}

TEST_CASE("wait_until_drive(): a persistent sensor fault from the very start of the move still ends the wait as stuck, not hung forever") {
  Drive chassis = make_chassis();
  chassis.pid_print_toggle(false);
  chassis.left_motors[0].fake().disconnected = true;
  chassis.right_motors[0].fake().disconnected = true;
  chassis.pid_drive_set(24, 100);

  Outcome o = run_with_fault(chassis, 3000, [&] { chassis.pid_wait_until(12.0); });
  CHECK(o.returned);
  CHECK(o.interfered);
  CHECK(o.passes < 300);
}
