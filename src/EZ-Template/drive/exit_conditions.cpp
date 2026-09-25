/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <cmath>

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"

using namespace ez;

namespace {
static constexpr int STUCK_START_ALLOWANCE_MS = 1000;  // PID::VELOCITY_ARM_FALLBACK, which is private
static constexpr int STUCK_STARVED_WINDOWS = 4;
std::uint32_t stuck_passes() { return ez::detail::stats.auto_task_passes.load(std::memory_order_relaxed); }
double stuck_step(PID& pid) {
  if (pid.exit.small_error > 0) return pid.exit.small_error;
  return pid.velocity_sensor_main_exit_get() * pid.exit.velocity_exit_time / util::DELAY_TIME;
}

// A single progress channel: has `size` (always >= 0, e.g. a distance or |error|) come down to a new low, a full
// `step` below the last one, since progress was last credited?  Going past the point (`error`'s sign flipping) and
// coming back counts, measured from how far past it went, and so does a straight shove that pushes `size` a full
// step worse without ever crossing the point -- either way only once per channel: a robot being spun or shoved
// back and forth crosses (or is pushed away from) its target over and over, and only the first excursion gets
// credited toward a full recovery being required.
struct Channel {
  double step, low;
  bool side, rebound = false, rebounded = false;
  Channel(double p_step, double size, double error) : step(p_step), low(size), side(error > 0) {}
  bool made(double size, double error) {
    // A NaN size/error -- e.g. a caller-supplied NaN target, making every pass' distance/error compute to
    // NaN -- must not read as progress.  Every comparison against NaN is false, so unguarded this fell
    // through the "still above the last low?" check below no matter how many times it ran, crediting a new
    // low every single pass AND overwriting that low with NaN, which together defeat this channel (and
    // whatever backstop owns it) for as long as NaN keeps arriving.  Bailing out first, before side/low/
    // rebound are touched, makes a NaN reading simply invisible to this channel: no progress credited, and
    // no corruption of its state -- exactly as if that pass hadn't happened. A finite reading right after
    // still cures it immediately, the same as before this guard existed.
    if (!std::isfinite(size) || !std::isfinite(error)) return false;
    bool overshot = (error > 0) != side;
    bool shoved = size > low + step;
    if ((overshot || shoved) && !rebounded) rebound = rebounded = true;
    side = error > 0;
    if (rebound) low = std::fmax(low, size);
    if (size >= low - step) return false;
    low = size;
    rebound = false;
    return true;
  }
};

// Tells an odom wait when the robot is stuck: no progress for the xy velocity exit's time.  Progress is pure
// pursuit moving onto a new point, or the distance to the point being driven to or the heading error coming down
// to a new low, a full step (that PID's small exit error) below the last one.  That holds at any heading error and
// whether or not something is turning or pushing the robot, and it can't keep a wait going forever: each new low is
// a step below the last, so a robot that isn't getting anywhere runs out of them.  Going past the point (the PID's
// error changing sign) and coming back counts, measured from how far past it went, but only once per point: a robot
// being spun or shoved back and forth crosses its target over and over.
//
// Until the robot has moved a step from where the motion started, it also gets the time the velocity exit gives a
// robot that hasn't moved yet (PID's VELOCITY_ARM_FALLBACK).  Pure pursuit steps through its first few points before
// the robot moves at all, so those don't count as moving; and a second wait on the same motion doesn't get the
// allowance again once the robot has moved.  With the velocity exit off the current exit's time is used, and with
// both off this never fires.
//
// The window has to pass on the clock and in ez_auto_task passes both: the errors only change when that task runs,
// so a task starved of time (a busy higher priority task) freezes them without the robot being stuck.  But a task
// that never runs again (blocked for good, or deleted) must not hold the wait forever, so past STARVED_WINDOWS windows
// on the clock alone it counts as stuck anyway.
class StuckWatch {
 public:
  // travelled and turned: how far the robot has moved and turned since the motion started
  StuckWatch(PID& xy, PID& angle, int index, double distance, double travelled, double turned)
      : xy_(stuck_step(xy), distance, xy.error), a_(stuck_step(angle), std::fabs(angle.error), angle.error), index_(index), window_(xy.exit.velocity_exit_time != 0 ? xy.exit.velocity_exit_time : xy.exit.mA_timeout), moved_(travelled > xy_.step || turned > a_.step) {
    int allowance = moved_ ? 0 : STUCK_START_ALLOWANCE_MS;
    last_progress_ = pros::millis() + allowance;
    last_progress_pass_ = stuck_passes() + allowance / util::DELAY_TIME;
  }

  // distance: how far the robot is from the point it's driving to.  xy_error and a_error: the PIDs' signed errors.
  bool stuck(int index, double distance, double xy_error, double a_error, double travelled, double turned) {
    if (window_ == 0) return false;
    std::uint32_t now = pros::millis();
    std::uint32_t pass = stuck_passes();
    bool progress = false;
    if (index != index_) {
      index_ = index;
      xy_ = Channel(xy_.step, distance, xy_error);
      a_ = Channel(a_.step, std::fabs(a_error), a_error);
      progress = true;
    }
    if (xy_.made(distance, xy_error)) progress = true;
    if (a_.made(std::fabs(a_error), a_error)) progress = true;
    if (!moved_ && (travelled > xy_.step || turned > a_.step)) moved_ = progress = true;
    // Before the robot has moved, progress can't cut the start allowance short
    if (progress && (moved_ || (std::int32_t)(now - last_progress_) > 0)) {
      last_progress_ = now;
      last_progress_pass_ = pass;
    }
    std::int32_t waited = now - last_progress_;
    if (waited <= window_) return false;
    // Confirming ez_auto_task really kept running (not just wall-clock time passing while it's starved or dead)
    // needs an expected pass count for window_.  This used to derive that count from this watch's own observed
    // passes-per-ms since it was constructed (elapsed real ms since construction / elapsed real passes since
    // construction).  Two problems with that, both real:
    //
    // Steady state: expected_passes (window_ / observed_delay) times observed_delay is window_ again, by
    // construction, whatever observed_delay is measured as -- so "real passes since last progress" exceeding
    // "expected_passes" reduces to exactly waited > window_, the same test just above, for any task ticking at
    // a roughly steady rate, healthy or merely busy alike. A task running a little slower than DELAY_TIME
    // (ordinary scheduling overhead, not dead) got confirmed about as fast as a perfectly healthy one, not
    // anywhere near the STUCK_STARVED_WINDOWS margin a fully dead task gets.
    //
    // A temporary gap (ez_auto_task busy with something else for a while, then resuming) is worse: while its
    // own pass counter is frozen, elapsed_passes freezes too, but elapsed_ms keeps climbing (pros::millis() is
    // read from this, the CALLING task's own un-starved loop) -- so observed_delay climbs and expected_passes
    // keeps shrinking, while the confirmation numerator (pass - last_progress_pass_) is frozen right along with
    // pass itself. Once the shrinking threshold drops below that frozen numerator, this fires on a gap that
    // produced zero confirming passes, on a robot that was never actually stuck, only unreported on for a
    // while. How dangerous a gap is scales with how many passes had already elapsed when it began (call it P):
    // roughly, it takes a gap proportional to P before the threshold can fall that far -- so a wait already
    // well established tolerates a LONGER gap than one still early on, the opposite of "gets worse the longer
    // the wait runs" (an earlier pass at this same mechanism said exactly that; re-deriving it here and
    // confirming by repro says otherwise -- early gaps are the dangerous ones).
    //
    // Whether prompt-but-tight detection for a merely-busy task was ever the original intent, or real extra
    // leniency for one was, is genuinely ambiguous from that alone; what's concrete is the audit's own repro:
    // a real, single ~1s gap during an otherwise-healthy, still-progressing 2.5in/s approach got confirmed
    // stuck. Comparing the real pass count against a fixed count of DELAY_TIME-long passes instead removes the
    // shrinking threshold entirely, so a frozen numerator can never catch up to it, at any P -- while it also
    // trades some promptness for leniency in the steady-state case: a task genuinely running slower than
    // nominal (without being dead) now needs fewer of its own passes to reach this fixed count, but each of
    // its passes represents more real wall-clock time, so a genuinely stuck robot under a merely-busy task is
    // now confirmed measurably later than before (real-world tolerance now scales with how busy the task is,
    // not fixed at ~window_) -- while a task that stops passing entirely still can't reach any positive count
    // at all and is
    // still caught by the STARVED_WINDOWS wall-clock fallback below, unchanged.  Flagging the latency tradeoff
    // for a design call, the same as the Channel rebound latch above.
    int expected_passes = (int)(window_ / (double)util::DELAY_TIME);
    return (std::int32_t)(pass - last_progress_pass_) > expected_passes || waited > STUCK_STARVED_WINDOWS * window_;
  }

 private:
  Channel xy_, a_;
  int index_;
  int window_;
  bool moved_;
  std::uint32_t last_progress_, last_progress_pass_;
};

// The same progress backstop as StuckWatch, but for a single PID with no odometry and no path index -- DRIVE,
// TURN and SWING have none of StuckWatch's protection today (only velocity/mA exits, both defeated by a sustained
// disturbance that never reads as "stopped" and never draws over current -- a spin, a defender holding the robot
// mid-turn, sensor jitter under contact).  "Moved" is judged from this same PID's own error closing by a step from
// where it started, since these modes have no separate odometry-derived travelled/turned to check against.
class SingleStuckWatch {
 public:
  SingleStuckWatch(PID& pid, double error)
      : ch_(stuck_step(pid), std::fabs(error), error), window_(pid.exit.velocity_exit_time != 0 ? pid.exit.velocity_exit_time : pid.exit.mA_timeout), moved_(false) {
    last_progress_ = pros::millis() + STUCK_START_ALLOWANCE_MS;
    last_progress_pass_ = stuck_passes() + STUCK_START_ALLOWANCE_MS / util::DELAY_TIME;
  }

  bool stuck(double error) {
    if (window_ == 0) return false;
    std::uint32_t now = pros::millis();
    std::uint32_t pass = stuck_passes();
    bool progress = ch_.made(std::fabs(error), error);
    if (!moved_ && progress) moved_ = true;
    if (progress && (moved_ || (std::int32_t)(now - last_progress_) > 0)) {
      last_progress_ = now;
      last_progress_pass_ = pass;
    }
    std::int32_t waited = now - last_progress_;
    if (waited <= window_) return false;
    // See the matching comment in StuckWatch::stuck() -- a fixed, nominal-DELAY_TIME pass count, not one
    // derived from this watch's own observed (and self-referential) cadence.
    int expected_passes = (int)(window_ / (double)util::DELAY_TIME);
    return (std::int32_t)(pass - last_progress_pass_) > expected_passes || waited > STUCK_STARVED_WINDOWS * window_;
  }

 private:
  Channel ch_;
  int window_;
  bool moved_;
  std::uint32_t last_progress_, last_progress_pass_;
};

// A velocity exit doesn't end an odom wait: a robot pivoting at a corner, or just slow, reads as stopped to it while
// it's still getting somewhere.  StuckWatch decides stuck instead.  Small, big and current exits end it as always.
exit_output without_velocity(exit_output e) { return e == VELOCITY_EXIT ? RUNNING : e; }

// wait_until_drive()'s own crossed check -- comparing how far the robot has actually driven to the distance it was
// asked to wait for -- is the only thing allowed to end an ODOM wait successfully.  On an odom move, leftPID/
// rightPID's own target is a fixed look-ahead point a few inches past where the motion started (raw_pid_odom_ptp_set,
// set_odom_pid.cpp), retargeted once and never again, so their SMALL_EXIT/BIG_EXIT below reflect settling on THAT
// near point, not on the distance actually being waited for -- letting either one end the wait would silently
// return short of it.  VELOCITY_EXIT and mA_EXIT stay: a stalled motor's velocity or current reads the same
// regardless of which target produced the error that triggered them, so those are still real stall signals here.
// DRIVE (a plain, non-odom move) is unaffected: leftPID/rightPID's target there already is the real drive target.
exit_output without_position_exits(exit_output e) { return (e == SMALL_EXIT || e == BIG_EXIT) ? RUNNING : e; }

// Every motor on both sides, for an mA exit that has to see whichever motor in the group is actually the one
// absorbing a stall -- checking only index 0 of each side missed a jam or pin that landed on a different motor.
std::vector<pros::Motor> both_sides(const std::vector<pros::Motor>& left, const std::vector<pros::Motor>& right) {
  // pros::Motor has no copy-assignment operator (it inherits pros::Device's const port/type members), so
  // vector::insert -- which needs assignment to shift/fill elements -- doesn't compile here even though
  // every element is only ever copy-constructed in practice.  push_back sidesteps that.
  std::vector<pros::Motor> all;
  all.reserve(left.size() + right.size());
  for (const auto& m : left) all.push_back(m);
  for (const auto& m : right) all.push_back(m);
  return all;
}
}  // namespace

// Feeds a PID's secondary velocity-exit channel from the imu's acceleration, but only when that PID's
// secondary channel is turned on.  It's off by default: acceleration reads near 0 during an ordinary
// constant-speed cruise too, so on its own it can't tell cruising from stalled (see PID::exit_condition
// and velocity_sensor_secondary_exit_set).  Skipping the read when the channel is off also avoids
// polling the imu over the smart port every loop just to have exit_condition() throw the value away.
void Drive::secondary_velocity_sensor_update(PID& pid) {
  if (!pid.velocity_sensor_secondary_toggle_get()) return;
  pid.velocity_sensor_secondary_set(drive_imu_accel_get());
}

// Holds xyPID's velocity exit while turn bias has fully zeroed xy_out (ptp_task(), pid_tasks.cpp) to
// prioritize turning.  xy_delta_fake reads ~0 then because the robot genuinely isn't translating, not
// because it's stalled, and xyPID can't tell those apart from the reading alone.  See
// PID::velocity_exit_hold_set() for the fallback that keeps this from being able to hang pid_wait().
void Drive::xy_velocity_exit_hold_update() {
  xyPID.velocity_exit_hold_set(xy_translation_bias_gated);
}

void Drive::pid_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu) {
  leftPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  rightPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  leftPID.velocity_sensor_secondary_toggle_set(use_imu);
  rightPID.velocity_sensor_secondary_toggle_set(use_imu);
  internal_leftPID.exit = leftPID.exit;
  internal_rightPID.exit = rightPID.exit;
}

void Drive::pid_drive_exit_condition_set(ez::QTime p_small_exit_time, ez::QLength p_small_error, ez::QTime p_big_exit_time, ez::QLength p_big_error, ez::QTime p_velocity_exit_time, ez::QTime p_mA_timeout, bool use_imu) {
  // Convert units to doubles
  double se = p_small_error.convert(ez::inch);
  double be = p_big_error.convert(ez::inch);
  int set = p_small_exit_time.convert(ez::millisecond);
  int bet = p_big_exit_time.convert(ez::millisecond);
  int vet = p_velocity_exit_time.convert(ez::millisecond);
  int mAt = p_mA_timeout.convert(ez::millisecond);

  pid_drive_exit_condition_set(set, se, bet, be, vet, mAt, use_imu);
}

void Drive::pid_turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu) {
  turnPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  turnPID.velocity_sensor_secondary_toggle_set(use_imu);
}

void Drive::pid_turn_exit_condition_set(ez::QTime p_small_exit_time, ez::QAngle p_small_error, ez::QTime p_big_exit_time, ez::QAngle p_big_error, ez::QTime p_velocity_exit_time, ez::QTime p_mA_timeout, bool use_imu) {
  // Convert units to doubles
  double se = p_small_error.convert(ez::degree);
  double be = p_big_error.convert(ez::degree);
  int set = p_small_exit_time.convert(ez::millisecond);
  int bet = p_big_exit_time.convert(ez::millisecond);
  int vet = p_velocity_exit_time.convert(ez::millisecond);
  int mAt = p_mA_timeout.convert(ez::millisecond);

  pid_turn_exit_condition_set(set, se, bet, be, vet, mAt, use_imu);
}

void Drive::pid_swing_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu) {
  swingPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  swingPID.velocity_sensor_secondary_toggle_set(use_imu);
}

void Drive::pid_swing_exit_condition_set(ez::QTime p_small_exit_time, ez::QAngle p_small_error, ez::QTime p_big_exit_time, ez::QAngle p_big_error, ez::QTime p_velocity_exit_time, ez::QTime p_mA_timeout, bool use_imu) {
  // Convert units to doubles
  double se = p_small_error.convert(ez::degree);
  double be = p_big_error.convert(ez::degree);
  int set = p_small_exit_time.convert(ez::millisecond);
  int bet = p_big_exit_time.convert(ez::millisecond);
  int vet = p_velocity_exit_time.convert(ez::millisecond);
  int mAt = p_mA_timeout.convert(ez::millisecond);

  pid_swing_exit_condition_set(set, se, bet, be, vet, mAt, use_imu);
}

void Drive::pid_odom_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu) {
  xyPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  xyPID.velocity_sensor_secondary_toggle_set(use_imu);
}

void Drive::pid_odom_drive_exit_condition_set(ez::QTime p_small_exit_time, ez::QLength p_small_error, ez::QTime p_big_exit_time, ez::QLength p_big_error, ez::QTime p_velocity_exit_time, ez::QTime p_mA_timeout, bool use_imu) {
  // Convert units to doubles
  double se = p_small_error.convert(ez::inch);
  double be = p_big_error.convert(ez::inch);
  int set = p_small_exit_time.convert(ez::millisecond);
  int bet = p_big_exit_time.convert(ez::millisecond);
  int vet = p_velocity_exit_time.convert(ez::millisecond);
  int mAt = p_mA_timeout.convert(ez::millisecond);

  pid_odom_drive_exit_condition_set(set, se, bet, be, vet, mAt, use_imu);
}

void Drive::pid_odom_turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu) {
  current_a_odomPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  current_a_odomPID.velocity_sensor_secondary_toggle_set(use_imu);
}

void Drive::pid_odom_turn_exit_condition_set(ez::QTime p_small_exit_time, ez::QAngle p_small_error, ez::QTime p_big_exit_time, ez::QAngle p_big_error, ez::QTime p_velocity_exit_time, ez::QTime p_mA_timeout, bool use_imu) {
  // Convert units to doubles
  double se = p_small_error.convert(ez::degree);
  double be = p_big_error.convert(ez::degree);
  int set = p_small_exit_time.convert(ez::millisecond);
  int bet = p_big_exit_time.convert(ez::millisecond);
  int vet = p_velocity_exit_time.convert(ez::millisecond);
  int mAt = p_mA_timeout.convert(ez::millisecond);

  pid_odom_turn_exit_condition_set(set, se, bet, be, vet, mAt, use_imu);
}

// User wrapper for exit condition
void Drive::pid_wait() {
  // Let the PID run at least 1 iteration
  pros::delay(util::DELAY_TIME);

  if (mode == DRIVE) {
    exit_output left_exit = RUNNING;
    exit_output right_exit = RUNNING;
    // A concurrent pid_drive_set() from another task retargets these same leftPID/rightPID objects mid-wait --
    // this loop has no lock on them (drive_mutex can't be held across its own pros::delay, and a per-pass
    // lock/unlock wouldn't close the window between passes either) so without this check it would just keep
    // polling whatever motion is live now and report a clean, uninterfered success on THAT one, silently, once
    // its exit fires -- not the motion this call was actually started for.  Catching a retarget by watching the
    // PID's own target is the smallest signal available without new locking or new shared state.  mode itself is
    // also watched: a concurrent setter from a DIFFERENT mode (e.g. pid_turn_set() while this is waiting on
    // DRIVE) doesn't touch leftPID/rightPID's target at all, so without this it would go unnoticed, freezing
    // this wait's view of the abandoned motion instead of ending it.
    e_mode mode_snapshot = mode;
    double left_target = leftPID.target_get();
    double right_target = rightPID.target_get();
    // DRIVE has no odometry to fall back on, so this is the same progress backstop StuckWatch gives odom waits,
    // built around each side's own error instead: neither velocity nor mA catches a sustained disturbance that
    // never reads as "stopped" and never draws over current (a continuous spin, a defender holding the robot,
    // sensor jitter under contact) -- see WAIT_BEHAVIOR_SPEC.md's JC-1.  A side only counts as "stuck" once IT is
    // still RUNNING and not progressing; a side that already exited cleanly can't drag the wait down.
    SingleStuckWatch left_watch(leftPID, leftPID.error), right_watch(rightPID, rightPID.error);
    bool stalled = false;
    while (left_exit == RUNNING || right_exit == RUNNING) {
      if (mode != mode_snapshot || leftPID.target_get() != left_target || rightPID.target_get() != right_target) {
        if (print_toggle) std::cout << "  Drive: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
        interfered = true;
        return;
      }
      secondary_velocity_sensor_update(leftPID);
      secondary_velocity_sensor_update(rightPID);
      left_exit = left_exit != RUNNING ? left_exit : leftPID.exit_condition(left_motors);
      right_exit = right_exit != RUNNING ? right_exit : rightPID.exit_condition(right_motors);
      bool left_stuck = left_exit == RUNNING && left_watch.stuck(leftPID.error);
      bool right_stuck = right_exit == RUNNING && right_watch.stuck(rightPID.error);
      // Stuck only when at least one side is still RUNNING and every side that's still RUNNING is stuck --
      // not when both sides just happened to exit normally on the same pass, which the (exit != RUNNING) half
      // of each clause would otherwise also satisfy.
      if ((left_exit == RUNNING || right_exit == RUNNING) && (left_exit != RUNNING || left_stuck) && (right_exit != RUNNING || right_stuck)) {
        // A side that's still RUNNING and already sitting inside its own big error window is where a big
        // exit would have left it: that's settled, not stuck. (A side hovering across its own small error
        // window can keep both its own exit timers from ever finishing.) A side that already exited cleanly
        // can't block settling.
        bool left_settled = left_exit != RUNNING || std::fabs(leftPID.error) < leftPID.exit.big_error;
        bool right_settled = right_exit != RUNNING || std::fabs(rightPID.error) < rightPID.exit.big_error;
        stalled = !(left_settled && right_settled);
        if (print_toggle) std::cout << "  Drive: " << (stalled ? "Stuck" : "Stuck, but stopped inside the big error windows, counted as settled") << ", error: L," << leftPID.error << " R," << rightPID.error << "\n";
        break;
      }
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle && !stalled) std::cout << "  Left: " << exit_to_string(left_exit) << " Exit, error: " << leftPID.error << "   Right: " << exit_to_string(right_exit) << " Exit, error: " << rightPID.error << "\n";

    if (stalled || left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
      interfered = true;
    }
  }

  // Odom Exits
  else if (mode == POINT_TO_POINT || mode == PURE_PURSUIT) {
    exit_output xy_exit = RUNNING;
    exit_output a_exit = RUNNING;

    // The point being driven to right now (a boomerang's target, not its carrot, which moves as the robot does).
    // Locked: a motion started from another task can replace pp_movements while this reads it.
    auto target_distance = [&]() {
      ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
      pose t = mode == PURE_PURSUIT && pp_index < (int)pp_movements.size() ? pp_movements[pp_index].target : odom_target;
      return util::distance_to_point(t, odom_pose_get());
    };
    auto travelled = [&]() { return util::distance_to_point(odom_start, odom_pose_get()); };
    auto turned = [&]() { return std::fabs(odom_theta_get() - odom_start.theta); };
    StuckWatch watch(xyPID, current_a_odomPID, pp_index, target_distance(), travelled(), turned());
    bool stalled = false;

    // A concurrent pid_odom_*_set() from another task retargets xyPID/current_a_odomPID (and resets pp_index
    // and pp_movements) mid-wait -- same hazard as the DRIVE branch above, just for odom.  odom_target_start
    // is set only where a NEW motion actually starts (pid_odom_ptp_set() and raw_pid_odom_pp_set(),
    // set_odom_pid.cpp), whether the retarget lands as another plain point, boomerang, or pure pursuit path,
    // so watching it catches a retarget regardless of what it lands in -- unlike leftPID/rightPID's target,
    // which raw_pid_odom_ptp_set() (the SAME motion's own per-waypoint advance, called again by pp_task() each
    // time pure pursuit steps onto a new point, pid_tasks.cpp) legitimately rewrites on every ordinary
    // waypoint advance and so can't be used here without false-firing on a healthy path.  mode is watched
    // too, the same reason as the DRIVE branch above: a concurrent setter from a different mode (e.g.
    // pid_turn_set() while this odom wait is still running) wouldn't touch odom_target_start at all.
    e_mode mode_snapshot = mode;
    pose retarget_target = odom_target_start;

    // Wait until pure pursuit is on the last point, then continue as normal.  xy's exit is checked every pass
    // and not kept: before the last point its target is only a look ahead away and keeps moving, so a small,
    // big or velocity exit here says nothing about the path, and one kept from a pause earlier on must not end
    // the wait later.  A current exit still ends it.  Angle's exit is kept across a pass the same way xy's isn't
    // meant to be -- but it must not carry past the point it settled at: a heading exit latched on an earlier,
    // unrelated point can't stand in for the final point's actual heading, so it's reset every time pp_index
    // moves to a new point, including the last move into the final point below.
    int a_exit_index = pp_index;
    if (mode == PURE_PURSUIT) {
      while (pp_index != (int)pp_movements.size() - 1) {
        if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
          if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of continuing on the wrong path.\n";
          interfered = true;
          return;
        }
        if (pp_index != a_exit_index) {
          a_exit = RUNNING;
          a_exit_index = pp_index;
        }
        secondary_velocity_sensor_update(xyPID);
        secondary_velocity_sensor_update(current_a_odomPID);
        xy_velocity_exit_hold_update();
        exit_output xy_pass = xyPID.exit_condition(both_sides(left_motors, right_motors));
        a_exit = a_exit != RUNNING ? a_exit : without_velocity(current_a_odomPID.exit_condition(both_sides(left_motors, right_motors)));

        if (xy_pass == mA_EXIT || watch.stuck(pp_index, target_distance(), xyPID.error, current_a_odomPID.error, travelled(), turned())) {
          stalled = true;
          if (print_toggle) std::cout << "  XY: " << (xy_pass == mA_EXIT ? exit_to_string(xy_pass) : "Stuck") << " Exited early at point " << pp_index << " of " << (int)pp_movements.size() - 1 << ", error: " << xyPID.error << ".   Angle error: " << current_a_odomPID.error << ".\n";
          break;
        }

        pros::delay(util::DELAY_TIME);
      }
    }
    if (pp_index != a_exit_index) a_exit = RUNNING;  // the final move onto the last point never re-enters the loop above

    // When we're at the last point in PP / we're just going to point
    while (!stalled && (xy_exit == RUNNING || a_exit == RUNNING)) {
      if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
        if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
        interfered = true;
        return;
      }
      secondary_velocity_sensor_update(xyPID);
      secondary_velocity_sensor_update(current_a_odomPID);
      xy_velocity_exit_hold_update();
      xy_exit = xy_exit != RUNNING ? xy_exit : without_velocity(xyPID.exit_condition(both_sides(left_motors, right_motors)));
      a_exit = a_exit != RUNNING ? a_exit : without_velocity(current_a_odomPID.exit_condition(both_sides(left_motors, right_motors)));
      if ((xy_exit == RUNNING || a_exit == RUNNING) && watch.stuck(pp_index, target_distance(), xyPID.error, current_a_odomPID.error, travelled(), turned())) {
        // Stopped inside both big error windows is where a big exit would have left it: that's settled, not stuck.
        // (A robot hovering across the small error window can keep both exit timers from ever finishing.)
        bool settled = target_distance() < xyPID.exit.big_error && std::fabs(current_a_odomPID.error) < current_a_odomPID.exit.big_error;
        stalled = !settled;
        if (print_toggle) std::cout << "  XY: " << exit_to_string(xy_exit) << ", error: " << xyPID.error << ".   Angle: " << exit_to_string(a_exit) << ", error: " << current_a_odomPID.error << (settled ? ".   Stopped inside the big error windows, counted as settled.\n" : ".   Stuck before settling on the target.\n");
        break;
      }
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle && !stalled && xy_exit != RUNNING && a_exit != RUNNING) std::cout << "  XY: " << exit_to_string(xy_exit) << " Exit, error: " << xyPID.error << ".   Angle: " << exit_to_string(a_exit) << " Exit, error: " << current_a_odomPID.error << ".\n";

    if (stalled || xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT || a_exit == mA_EXIT || a_exit == VELOCITY_EXIT) {
      interfered = true;
    }

    {
      ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
      // Store the heading as the equivalent angle nearest the IMU.  The raw target can be a full turn away from it
      // (IMU at 270, target -90), which the next drive or relative turn would read as a 360 degree error.
      //
      // Re-checked here, not just at the top of each loop above: this loop's own exit condition can become
      // satisfied on the very pass a concurrent retarget lands during THAT pass' own trailing pros::delay() --
      // after the pass' retarget check already ran clean, but before the loop re-enters to find both exits
      // non-RUNNING and fall out here.  A stale wait can reach this point having genuinely, cleanly finished
      // its OWN old motion while mode/odom_target_start already belong to a new one.  Without this re-check,
      // the write below would use whatever odom_target_start now holds -- the hijacking task's own in-flight
      // target -- corrupting shared PID state that task already relies on.  A stale wait must not touch shared
      // PID state on its way out once it notices it's been retargeted out from under it.
      bool retargeted_since_snapshot = mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta;
      if (!retargeted_since_snapshot && odom_target_start.theta != ANGLE_NOT_SET) headingPID.target_set(new_turn_target_compute(odom_target_start.theta, drive_angle_get(), shortest));
    }
  }

  // Turn Exit
  else if (mode == TURN || mode == TURN_TO_POINT) {
    exit_output turn_exit = RUNNING;
    SingleStuckWatch watch(turnPID, turnPID.error);  // see the DRIVE branch's comment on why -- same JC-1 gap.
    bool stalled = false;
    // Same concurrent-retarget guard as the DRIVE branch above.  turnPID.target is only ever rewritten by
    // turn_set_internal() (set_turn_pid.cpp) at the start of a new turn -- TURN_TO_POINT recomputes its own
    // aim point every pass through compute_error() without touching the PID's target, so this can't false-fire
    // on an ordinary turn-to-point motion, only on a real second pid_turn_set()/pid_turn_relative_set().  mode
    // is also watched -- see the DRIVE branch's comment on why a target-only check misses a cross-mode retarget.
    e_mode mode_snapshot = mode;
    double turn_target = turnPID.target_get();
    while (turn_exit == RUNNING) {
      if (mode != mode_snapshot || turnPID.target_get() != turn_target) {
        if (print_toggle) std::cout << "  Turn: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
        interfered = true;
        return;
      }
      secondary_velocity_sensor_update(turnPID);
      turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition(both_sides(left_motors, right_motors));
      if (turn_exit == RUNNING && watch.stuck(turnPID.error)) {
        // Same settled carve-out as the DRIVE branch above.
        bool settled = std::fabs(turnPID.error) < turnPID.exit.big_error;
        stalled = !settled;
        if (print_toggle) std::cout << "  Turn: " << (stalled ? "Stuck" : "Stuck, but stopped inside the big error window, counted as settled") << ", error: " << turnPID.error << "\n";
        break;
      }
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle && !stalled) std::cout << "  Turn: " << exit_to_string(turn_exit) << " Exit, error: " << turnPID.error << "\n";

    if (stalled || turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
      interfered = true;
    }
  }

  // Swing Exit
  else if (mode == SWING) {
    exit_output swing_exit = RUNNING;
    std::vector<pros::Motor>& sensor = current_swing == ez::LEFT_SWING ? left_motors : right_motors;
    SingleStuckWatch watch(swingPID, swingPID.error);  // see the DRIVE branch's comment on why -- same JC-1 gap.
    bool stalled = false;
    // Same concurrent-retarget guard as the DRIVE branch above -- swingPID.target is only rewritten by
    // swing_set_internal() (set_swing_pid.cpp) at the start of a new swing.  mode is also watched -- see the
    // DRIVE branch's comment on why a target-only check misses a cross-mode retarget.
    e_mode mode_snapshot = mode;
    double swing_target = swingPID.target_get();
    while (swing_exit == RUNNING) {
      if (mode != mode_snapshot || swingPID.target_get() != swing_target) {
        if (print_toggle) std::cout << "  Swing: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
        interfered = true;
        return;
      }
      secondary_velocity_sensor_update(swingPID);
      swing_exit = swing_exit != RUNNING ? swing_exit : swingPID.exit_condition(sensor);
      if (swing_exit == RUNNING && watch.stuck(swingPID.error)) {
        // Same settled carve-out as the DRIVE branch above.
        bool settled = std::fabs(swingPID.error) < swingPID.exit.big_error;
        stalled = !settled;
        if (print_toggle) std::cout << "  Swing: " << (stalled ? "Stuck" : "Stuck, but stopped inside the big error window, counted as settled") << ", error: " << swingPID.error << "\n";
        break;
      }
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle && !stalled) std::cout << "  Swing: " << exit_to_string(swing_exit) << " Exit, error: " << swingPID.error << "\n";

    if (stalled || swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) {
      interfered = true;
    }
  }
}

void Drive::wait_until_drive(double target) {
  pros::delay(10);

  // Make sure mode is correct
  if (!(mode == DRIVE || mode == POINT_TO_POINT || mode == PURE_PURSUIT)) {
    printf("Mode needs to be drive!\n");
    return;
  }

  // Calculate error between current and target (target needs to be an in between position)
  double l_tar = l_start + target;
  double r_tar = r_start + target;
  double l_error = l_tar - drive_sensor_left();
  double r_error = r_tar - drive_sensor_right();
  // The direction each side is expected to close from, taken from the requested target itself rather than from
  // this first live read: l_error/r_error above are already read after this function's own one-pass delay, so on
  // a very short target a fast robot could already be on the far side of it by the time they're read, which would
  // latch the "past it" sign as the starting one and make the crossed check below wait for a second flip that may
  // never come.  target's sign can't be stale this way, and both sides are offset from their own start by the same
  // target, so both are expected to close from the same direction.
  int l_sgn = util::sgn(target);
  int r_sgn = l_sgn;

  // An odom move's leftPID/rightPID target is a fixed look-ahead point set once at the start of the motion (see
  // without_position_exits()'s comment above); a plain DRIVE move's leftPID/rightPID target already is the real
  // drive target.  Only the odom case needs the exit-condition filtering and the real-distance-keyed backstop below
  // -- DRIVE's own target already tracks what this wait is actually waiting for.
  bool is_odom = mode == POINT_TO_POINT || mode == PURE_PURSUIT;

  exit_output left_exit = RUNNING;
  exit_output right_exit = RUNNING;
  // Same progress backstop as pid_wait()'s DRIVE branch -- this loop's own exits share that gap exactly (see the
  // comment there), and this function has no other protection against, say, a sustained spin.  On an odom move this
  // has to watch l_error/r_error (the real remaining distance to the real target) instead of leftPID/rightPID's own
  // error, for the same reason without_position_exits() strips SMALL_EXIT/BIG_EXIT out below: leftPID/rightPID's
  // error is measured against their frozen look-ahead target, not against `target`, so a healthy drive that has
  // simply driven past that near point reads to it as permanent non-progress and would be falsely flagged stuck.
  SingleStuckWatch left_watch(leftPID, is_odom ? l_error : leftPID.error), right_watch(rightPID, is_odom ? r_error : rightPID.error);

  // Same concurrent-retarget guard as pid_wait()'s branches.  In DRIVE mode leftPID/rightPID's target only
  // changes on a real second pid_drive_set(), same as pid_wait()'s DRIVE branch.  In odom modes (this function
  // also runs for POINT_TO_POINT/PURE_PURSUIT) leftPID/rightPID's target is the fixed look-ahead point and
  // legitimately gets rewritten on every ordinary waypoint advance (raw_pid_odom_ptp_set(), set_odom_pid.cpp),
  // so odom_target_start -- only touched by a top-level odom setter starting a genuinely new motion -- is used
  // instead, the same as pid_wait()'s odom branch.  mode is watched on top of both: a concurrent setter from a
  // mode that isn't even DRIVE/POINT_TO_POINT/PURE_PURSUIT (e.g. a real second pid_turn_set()) touches neither
  // leftPID/rightPID's target nor odom_target_start, so without this it would go unnoticed.
  e_mode mode_snapshot = mode;
  bool odom_mode = mode == POINT_TO_POINT || mode == PURE_PURSUIT;
  double left_target = leftPID.target_get();
  double right_target = rightPID.target_get();
  pose retarget_target = odom_target_start;

  while (true) {
    if (mode != mode_snapshot) {
      if (print_toggle) std::cout << "  Drive: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
      interfered = true;
      return;
    } else if (odom_mode) {
      if (odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
        if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
        interfered = true;
        return;
      }
    } else if (leftPID.target_get() != left_target || rightPID.target_get() != right_target) {
      if (print_toggle) std::cout << "  Drive: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
      interfered = true;
      return;
    }

    l_error = l_tar - drive_sensor_left();
    r_error = r_tar - drive_sensor_right();

    // Ends the wait successfully the moment EITHER side's own real distance driven (l_error/r_error's sign
    // flipping against l_sgn/r_sgn, both taken from target's own sign) reaches or passes what was asked for, not
    // only once both have.  Keeps waiting (and running the failsafes below) only while NEITHER side has gotten
    // there yet.
    if (util::sgn(l_error) == l_sgn && util::sgn(r_error) == r_sgn) {
      // An odom move ends on its xy exit, which only pid_wait() checks.  The left and right exits below are aimed
      // one look ahead from where the move started, so when the robot drives past that point they can never fire.
      // If the move ends before it reaches this target, return instead of waiting forever.
      bool on_last_point = mode == POINT_TO_POINT || (mode == PURE_PURSUIT && pp_index == (int)pp_movements.size() - 1);
      if (on_last_point) {
        secondary_velocity_sensor_update(xyPID);
        xy_velocity_exit_hold_update();
        exit_output xy_exit = xyPID.exit_condition(both_sides(left_motors, right_motors));
        if (xy_exit != RUNNING) {
          if (print_toggle) std::cout << "  XY: " << exit_to_string(xy_exit) << " Wait Until Exit Failsafe, the move ended before reaching " << target << "\n";
          if (xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT) interfered = true;
          return;
        }
      }

      if (left_exit == RUNNING || right_exit == RUNNING) {
        secondary_velocity_sensor_update(leftPID);
        secondary_velocity_sensor_update(rightPID);
        if (left_exit == RUNNING) left_exit = is_odom ? without_position_exits(leftPID.exit_condition(left_motors)) : leftPID.exit_condition(left_motors);
        if (right_exit == RUNNING) right_exit = is_odom ? without_position_exits(rightPID.exit_condition(right_motors)) : rightPID.exit_condition(right_motors);
        bool left_stuck = left_exit == RUNNING && left_watch.stuck(is_odom ? l_error : leftPID.error);
        bool right_stuck = right_exit == RUNNING && right_watch.stuck(is_odom ? r_error : rightPID.error);
        // See the matching comment in pid_wait()'s DRIVE branch -- both sides exiting normally on the same pass
        // must not read as stuck.
        if ((left_exit == RUNNING || right_exit == RUNNING) && (left_exit != RUNNING || left_stuck) && (right_exit != RUNNING || right_stuck)) {
          if (print_toggle) std::cout << "  Drive: Stuck Wait Until Exit Failsafe, triggered at " << drive_sensor_left() - l_start << " instead of " << target << "\n";
          interfered = true;
          return;
        }
        // No delay here -- the loop's own delay at the bottom already advances one DELAY_TIME per pass.  A second
        // delay here made every failsafe in this function take about twice its configured time in real time.
      } else {
        if (print_toggle) {
          std::cout << "  Left: " << exit_to_string(left_exit) << " Wait Until Exit Failsafe, triggered at " << drive_sensor_left() - l_start << " instead of " << target << "\n";
          std::cout << "  Right: " << exit_to_string(right_exit) << " Wait Until Exit Failsafe, triggered at " << drive_sensor_right() - r_start << " instead of " << target << "\n";
        }
        if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
          interfered = true;
        }
        return;
      }
    }
    // Once either side has reached or passed target, return
    else {
      if (print_toggle) printf("  Drive Wait Until Exit Success. Triggered at: L,R(%.2f, %.2f)  Target: L,R(%.2f, %.2f)\n", drive_sensor_left() - l_start, drive_sensor_right() - r_start, target, target);
      leftPID.timers_reset();
      rightPID.timers_reset();
      return;
    }

    pros::delay(util::DELAY_TIME);
  }
}

// Function to wait until a certain position is reached.  Wrapper for exit condition.
void Drive::wait_until_turn_swing(double target) {
  // Resolve using the motion's own behavior
  target = new_turn_target_compute(target, drive_angle_get(), current_angle_behavior);
  wait_until_turn_swing_internal(target);
}

// Expects an already-resolved absolute target.  Does not re-resolve behavior.
void Drive::wait_until_turn_swing_internal(double target) {
  // Make sure mode is correct
  if (!(mode == TURN || mode == SWING || mode == TURN_TO_POINT)) {
    printf("Mode needs to be swing or turn!\n");
    return;
  }

  // Calculate error between current and target (target needs to be an in between position)
  double g_error = target - drive_angle_get();
  int g_sgn = util::sgn(g_error);

  exit_output turn_exit = RUNNING;
  exit_output swing_exit = RUNNING;

  std::vector<pros::Motor>& sensor = current_swing == ez::LEFT_SWING ? left_motors : right_motors;
  // Same JC-1 progress backstop as pid_wait()'s TURN/SWING branches -- see the comment there.
  SingleStuckWatch turn_watch(turnPID, turnPID.error);
  SingleStuckWatch swing_watch(swingPID, swingPID.error);

  // Same concurrent-retarget guard as pid_wait()'s TURN/SWING branches -- this function had none, unlike
  // every other wait.  A concurrent pid_turn_set()/pid_turn_relative_set()/pid_swing_set() (or any other
  // motion setter) mid-wait retargets turnPID/swingPID and moves mode along with it, so both the specific
  // PID this call cares about and mode itself are snapshotted and checked every pass.
  e_mode mode_snapshot = mode;
  double turn_target = turnPID.target_get();
  double swing_target = swingPID.target_get();

  while (true) {
    if (mode != mode_snapshot || turnPID.target_get() != turn_target || swingPID.target_get() != swing_target) {
      if (print_toggle) std::cout << "  Turn/Swing: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
      interfered = true;
      return;
    }

    g_error = target - drive_angle_get();

    // If turning...
    if (mode == TURN || mode == TURN_TO_POINT) {
      // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
      if (util::sgn(g_error) == g_sgn) {
        if (turn_exit == RUNNING) {
          secondary_velocity_sensor_update(turnPID);
          turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition(both_sides(left_motors, right_motors));
          if (turn_exit == RUNNING && turn_watch.stuck(turnPID.error)) {
            if (print_toggle) std::cout << "  Turn: Stuck Wait Until Exit Failsafe, triggered at " << drive_angle_get() << " instead of " << target << "\n";
            interfered = true;
            return;
          }
          // No delay here -- see the matching comment in wait_until_drive().
        } else {
          if (print_toggle) std::cout << "  Turn: " << exit_to_string(turn_exit) << " Wait Until Exit Failsafe, triggered at " << drive_angle_get() << " instead of " << target << "\n";

          if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
            interfered = true;
          }
          return;
        }
      }
      // Once we've past target, return
      else if (util::sgn(g_error) != g_sgn) {
        if (print_toggle) printf("  Turn Wait Until Exit Success, triggered at %.2f.  Target: %.2f\n", drive_angle_get(), target);
        turnPID.timers_reset();
        return;
      }
    }

    // If swinging...
    else {
      // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
      if (util::sgn(g_error) == g_sgn) {
        if (swing_exit == RUNNING) {
          secondary_velocity_sensor_update(swingPID);
          swing_exit = swing_exit != RUNNING ? swing_exit : swingPID.exit_condition(sensor);
          if (swing_exit == RUNNING && swing_watch.stuck(swingPID.error)) {
            if (print_toggle) std::cout << "  Swing: Stuck Wait Until Exit Failsafe, triggered at " << drive_angle_get() << " instead of " << target << "\n";
            interfered = true;
            return;
          }
          // No delay here -- see the matching comment in wait_until_drive().
        } else {
          if (print_toggle) std::cout << "  Swing: " << exit_to_string(swing_exit) << " Wait Until Exit Failsafe, triggered at " << drive_angle_get() << " instead of " << target << "\n";

          if (swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) {
            interfered = true;
          }
          return;
        }
      }
      // Once we've past target, return
      else if (util::sgn(g_error) != g_sgn) {
        if (print_toggle) printf("  Swing Wait Until Exit Success, triggered at %.2f. Target: %.2f\n", drive_angle_get(), target);
        swingPID.timers_reset();
        return;
      }
    }

    pros::delay(util::DELAY_TIME);
  }
}

void Drive::pid_wait_until(ez::QLength target) {
  // If robot is driving...
  if (mode == DRIVE || mode == POINT_TO_POINT || mode == PURE_PURSUIT) {
    wait_until_drive(target.convert(ez::inch));
  } else {
    printf("QLength not supported for turn or swing!\n");
  }
}

void Drive::pid_wait_until(ez::QAngle target) {
  // If robot is driving...
  if (mode == TURN || mode == SWING || mode == TURN_TO_POINT) {
    wait_until_turn_swing(target.convert(ez::degree));
  } else {
    printf("QAngle not supported for drive!\n");
  }
}

void Drive::pid_wait_until(double target) {
  // If driving...
  if (mode == DRIVE || mode == POINT_TO_POINT || mode == PURE_PURSUIT) {
    wait_until_drive(target);
  }
  // If turning or swinging...
  else if (mode == TURN || mode == SWING || mode == TURN_TO_POINT) {
    wait_until_turn_swing(target);
  } else {
    printf("Not in a valid drive mode!\n");
  }
}

void Drive::pid_wait_until_point(pose target) {
  pros::delay(10);

  // Make sure mode is correct.  Without this, xyPID/current_a_odomPID are whatever an earlier odom motion left
  // them at -- not RUNNING for this call -- since only ptp_task()/boomerang_task() (POINT_TO_POINT/PURE_PURSUIT)
  // update them.
  if (!(mode == POINT_TO_POINT || mode == PURE_PURSUIT)) {
    printf("Mode needs to be an odom mode (point to point or pure pursuit)!\n");
    return;
  }

  int xy_sgn = util::sgn(is_past_target(target, odom_pose_get()));

  exit_output xy_exit = RUNNING;
  exit_output a_exit = RUNNING;
  StuckWatch watch(xyPID, current_a_odomPID, pp_index, util::distance_to_point(target, odom_pose_get()), util::distance_to_point(odom_start, odom_pose_get()), std::fabs(odom_theta_get() - odom_start.theta));

  // Same concurrent-retarget guard as pid_wait()'s odom branch -- this function had none, unlike every
  // other wait_until_*.  odom_target_start is only touched by a top-level odom setter starting a genuinely
  // new motion (see the comment on pid_wait()'s odom branch), so watching it catches a retarget regardless
  // of what it lands in.  mode is watched too, for a concurrent setter from a non-odom mode.
  e_mode mode_snapshot = mode;
  pose retarget_target = odom_target_start;

  while (true) {
    if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
      if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
      interfered = true;
      return;
    }
    secondary_velocity_sensor_update(xyPID);
    secondary_velocity_sensor_update(current_a_odomPID);
    xy_velocity_exit_hold_update();
    xy_exit = xy_exit != RUNNING ? xy_exit : without_velocity(xyPID.exit_condition(both_sides(left_motors, right_motors)));
    a_exit = a_exit != RUNNING ? a_exit : without_velocity(current_a_odomPID.exit_condition(both_sides(left_motors, right_motors)));

    // Same stuck check as pid_wait(), for a robot that is stuck but moving, which the exits above miss
    if (watch.stuck(pp_index, util::distance_to_point(target, odom_pose_get()), xyPID.error, current_a_odomPID.error, util::distance_to_point(odom_start, odom_pose_get()), std::fabs(odom_theta_get() - odom_start.theta))) {
      if (print_toggle) std::cout << "  Stuck before reaching (" << target.x << ", " << target.y << "), at (" << odom_x_get() << ", " << odom_y_get() << ")\n";
      interfered = true;
      return;
    }

    if (xy_exit != RUNNING && a_exit != RUNNING) {
      if (print_toggle) {
        std::cout << "  XY: " << exit_to_string(xy_exit) << " Wait Until Exit Failsafe, triggered at (" << odom_x_get() << ", " << odom_y_get() << ") instead of (" << target.x << ", " << target.y << ")\n";
        xyPID.timers_reset();
        current_a_odomPID.timers_reset();
      }
      if (xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT || a_exit == mA_EXIT || a_exit == VELOCITY_EXIT) {
        interfered = true;
      }
      return;
    }

    if (util::sgn((is_past_target(target, odom_pose_get()))) != xy_sgn) {
      if (print_toggle) printf("  XY Wait Until Exit Success, triggered at (%.2f, %.2f).  Target: (%.2f, %.2f)\n", odom_x_get(), odom_y_get(), target.x, target.y);
      xyPID.timers_reset();
      current_a_odomPID.timers_reset();
      return;
    }

    pros::delay(util::DELAY_TIME);
  }
}

void Drive::pid_wait_until_point(united_pose target) { pid_wait_until_point(util::united_pose_to_pose(target)); }
void Drive::pid_wait_until(pose target) { pid_wait_until_point(target); }
void Drive::pid_wait_until(united_pose target) { pid_wait_until_point(target); }

// wait for pp
void Drive::pid_wait_until_index_started(int index) {
  // Let the PID run at least 1 iteration
  pros::delay(util::DELAY_TIME);

  // injected_pp_index and pp_index are only ever built/advanced for a pure pursuit path -- in any other mode
  // this function's own while condition (pp_index < injected_pp_index[index]) can never become false, and unlike
  // pid_wait_until_point() there's no independent live-odometry crossed-check to fall back on.
  if (mode != PURE_PURSUIT) {
    printf("Mode needs to be pure pursuit!\n");
    return;
  }

  // Snapshotted once, locked: a concurrent motion can rebuild injected_pp_index (a whole-vector move assignment --
  // see set_odom_pid.cpp / purepursuit_math.cpp) while this function is reading it. Reading a vector mid-reassignment
  // is undefined behavior, not just stale data, so this takes one consistent copy instead of trusting each of the
  // several unlocked reads below to happen to land before or after the swap.
  std::vector<int> injected_pp_index_snapshot;
  {
    ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
    injected_pp_index_snapshot = injected_pp_index;
  }

  if (index < 0 || index > (int)injected_pp_index_snapshot.size() - 2) {
    printf("  Wait Until PP Error!  Index %i is not within range!  %i is max!\n", index, (int)injected_pp_index_snapshot.size() - 2);
    return;
  }
  index += 1;

  exit_output xy_exit = RUNNING;
  exit_output a_exit = RUNNING;
  // Locked: a motion started from another task can replace pp_movements while this reads it
  auto point_distance = [&]() {
    ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
    return pp_index < (int)pp_movements.size() ? util::distance_to_point(pp_movements[pp_index].target, odom_pose_get()) : 0.0;
  };
  StuckWatch watch(xyPID, current_a_odomPID, pp_index, point_distance(), util::distance_to_point(odom_start, odom_pose_get()), std::fabs(odom_theta_get() - odom_start.theta));
  while (pp_index < injected_pp_index_snapshot[index]) {
    secondary_velocity_sensor_update(xyPID);
    secondary_velocity_sensor_update(current_a_odomPID);
    xy_velocity_exit_hold_update();
    xy_exit = xy_exit != RUNNING ? xy_exit : without_velocity(xyPID.exit_condition(both_sides(left_motors, right_motors)));
    a_exit = a_exit != RUNNING ? a_exit : without_velocity(current_a_odomPID.exit_condition(both_sides(left_motors, right_motors)));

    // Same stuck check as pid_wait(), for a robot that is stuck but moving, which the exits above miss
    if (watch.stuck(pp_index, point_distance(), xyPID.error, current_a_odomPID.error, util::distance_to_point(odom_start, odom_pose_get()), std::fabs(odom_theta_get() - odom_start.theta))) {
      if (print_toggle) std::cout << "  Stuck before reaching point " << injected_pp_index_snapshot[index] << ", at (" << odom_x_get() << ", " << odom_y_get() << ")\n";
      interfered = true;
      break;
    }

    if (xy_exit != RUNNING && a_exit != RUNNING) {
      if (print_toggle) {
        // index points into injected_pp_index_snapshot, which holds where each waypoint sits in pp_movements
        std::cout << "  XY: " << exit_to_string(xy_exit) << " Wait Until Exit Failsafe, triggered at (" << odom_x_get() << ", " << odom_y_get() << ") instead of (" << pp_movements[injected_pp_index_snapshot[index]].target.x << ", " << pp_movements[injected_pp_index_snapshot[index]].target.y << ")\n";
        xyPID.timers_reset();
        current_a_odomPID.timers_reset();
      }
      if (xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT || a_exit == mA_EXIT || a_exit == VELOCITY_EXIT) {
        interfered = true;
      }
      break;
    }

    pros::delay(util::DELAY_TIME);
  }
}

void Drive::pid_wait_until_index(int index) {
  pid_wait_until_index_started(index);
  index += 1;
  std::vector<int> injected_pp_index_snapshot;
  {
    ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
    injected_pp_index_snapshot = injected_pp_index;
  }
  if (index < 0 || index >= (int)injected_pp_index_snapshot.size()) return;
  pose target = pp_movements[injected_pp_index_snapshot[index]].target;
  pid_wait_until_point(target);
}

// Pid wait, but quickly :)
void Drive::pid_wait_quick() {
  if (mode == PURE_PURSUIT) {
    int last_index;
    {
      ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
      last_index = (int)injected_pp_index.size() - 2;
    }
    pid_wait_until_index(last_index);
    {
      ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
      // Same as pid_wait(): store the equivalent angle nearest the IMU.
      if (odom_target_start.theta != ANGLE_NOT_SET) headingPID.target_set(new_turn_target_compute(odom_target_start.theta, drive_angle_get(), shortest));
    }
    return;
  } else if (mode == POINT_TO_POINT) {
    pid_wait_until_point(odom_target_start);
    {
      ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
      if (odom_target_start.theta != ANGLE_NOT_SET) headingPID.target_set(new_turn_target_compute(odom_target_start.theta, drive_angle_get(), shortest));
    }
    return;
  } else if (mode == TURN || mode == SWING || mode == TURN_TO_POINT) {
    // chain_target_start is already internal-frame and already behavior-resolved
    wait_until_turn_swing_internal(chain_target_start);
    return;
  } else if (mode != DRIVE) {
    printf("Not in a valid drive mode!\n");
    return;
  }

  // This is the target the user set, not the modified chained target
  pid_wait_until(chain_target_start);
}

// Set drive motion chain constants
void Drive::pid_drive_chain_constant_set(double input) {
  pid_drive_chain_forward_constant_set(input);
  pid_drive_chain_backward_constant_set(input);
}
void Drive::pid_drive_chain_forward_constant_set(double input) { drive_forward_motion_chain_scale = fabs(input); }
void Drive::pid_drive_chain_backward_constant_set(double input) { drive_backward_motion_chain_scale = fabs(input); }
void Drive::pid_drive_chain_constant_set(ez::QLength input) { pid_drive_chain_constant_set(input.convert(ez::inch)); }
void Drive::pid_drive_chain_forward_constant_set(ez::QLength input) { pid_drive_chain_forward_constant_set(input.convert(ez::inch)); }
void Drive::pid_drive_chain_backward_constant_set(ez::QLength input) { pid_drive_chain_backward_constant_set(input.convert(ez::inch)); }

// Set turn motion chain constants
void Drive::pid_turn_chain_constant_set(double input) { turn_motion_chain_scale = fabs(input); }
void Drive::pid_turn_chain_constant_set(ez::QAngle input) { pid_turn_chain_constant_set(input.convert(ez::degree)); }

// Set swing motion chain constants
void Drive::pid_swing_chain_constant_set(double input) {
  pid_swing_chain_forward_constant_set(input);
  pid_swing_chain_backward_constant_set(input);
}
void Drive::pid_swing_chain_forward_constant_set(double input) { swing_forward_motion_chain_scale = fabs(input); }
void Drive::pid_swing_chain_backward_constant_set(double input) { swing_backward_motion_chain_scale = fabs(input); }
void Drive::pid_swing_chain_constant_set(ez::QAngle input) { pid_swing_chain_constant_set(input.convert(ez::degree)); }
void Drive::pid_swing_chain_forward_constant_set(ez::QAngle input) { pid_swing_chain_forward_constant_set(input.convert(ez::degree)); }
void Drive::pid_swing_chain_backward_constant_set(ez::QAngle input) { pid_swing_chain_backward_constant_set(input.convert(ez::degree)); }

// Get motion chain constants
double Drive::pid_drive_chain_forward_constant_get() { return drive_forward_motion_chain_scale; }
double Drive::pid_drive_chain_backward_constant_get() { return drive_backward_motion_chain_scale; }
double Drive::pid_turn_chain_constant_get() { return turn_motion_chain_scale; }
double Drive::pid_swing_chain_forward_constant_get() { return swing_forward_motion_chain_scale; }
double Drive::pid_swing_chain_backward_constant_get() { return swing_backward_motion_chain_scale; }

// Pid wait that hold momentum into the next motion
void Drive::pid_wait_quick_chain() {
  {
    ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);

    // If driving, add drive_motion_chain_scale to target
    if (mode == DRIVE) {
      double chain_scale = motion_chain_backward ? drive_backward_motion_chain_scale : drive_forward_motion_chain_scale;
      used_motion_chain_scale = chain_scale * util::sgn(chain_target_start);
      leftPID.target_set(leftPID.target_get() + used_motion_chain_scale);
      rightPID.target_set(rightPID.target_get() + used_motion_chain_scale);
    }

    // If turning, add turn_motion_chain_scale to target
    else if (mode == TURN) {
      used_motion_chain_scale = turn_motion_chain_scale * util::sgn(chain_target_start - chain_sensor_start);
      turnPID.target_set(turnPID.target_get() + used_motion_chain_scale);
    }

    // If turning to a point, the turn task works out its target from the point every pass and never reads the
    // PID's target.  It adds used_motion_chain_scale to its error instead.
    else if (mode == TURN_TO_POINT) {
      used_motion_chain_scale = turn_motion_chain_scale * util::sgn(chain_target_start - chain_sensor_start);
    }

    // If swinging, add swing_motion_chain_scale to target
    else if (mode == SWING) {
      double chain_scale = motion_chain_backward ? swing_backward_motion_chain_scale : swing_forward_motion_chain_scale;
      used_motion_chain_scale = chain_scale * util::sgn(chain_target_start - chain_sensor_start);
      swingPID.target_set(swingPID.target_get() + used_motion_chain_scale);
    }

    // If odometrying, add drive_motion_chain_scale to the final target point
    // It'll be at the angle between the second to last point and the last point
    else if (mode == POINT_TO_POINT || mode == PURE_PURSUIT) {
      double chain_scale = current_drive_direction == REV ? drive_backward_motion_chain_scale : drive_forward_motion_chain_scale;
      used_motion_chain_scale = chain_scale;

      // Figure out what angle to use.
      // this will either by the angle between second to last point and last point,
      // or it'll be the boomerang end angle
      double angle = util::absolute_angle_to_point(odom_target_start, odom_second_to_last);
      if (odom_target_start.theta != ANGLE_NOT_SET) angle = odom_target_start.theta;

      // Create new point
      pose target = util::vector_off_point(used_motion_chain_scale, {odom_target_start.x, odom_target_start.y, angle});
      target.theta = odom_target_start.theta;

      // Replace target in ptp, add new final point if pp
      if (mode == POINT_TO_POINT)
        odom_target = target;
      else
        pp_movements.push_back({target,
                                pp_movements[pp_movements.size() - 1].drive_direction,
                                pp_movements[pp_movements.size() - 1].max_xy_speed});

    } else {
      drive_mutex.print_after_unlock("Not in a supported drive mode!\n");
      return;
    }
  }

  // Exit at the real target
  pid_wait_quick();
}
