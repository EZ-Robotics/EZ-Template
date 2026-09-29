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
// How many times pid_wait()'s DRIVE branch will reseed one side's SingleStuckWatch when its
// latched-side recheck un-latches it (see that recheck's own comment on the tradeoff this bounds).
// A realistic disturbance -- the issue #532 repro, or even a couple of genuinely distinct
// disturbances landing on the same side across one wait -- needs at most a handful of these. Past
// this many, that side falls back to the old, un-reseeded (frozen-clock) behavior for the rest of
// THIS wait: bounded, at the cost of losing the fresh-window benefit for any further relatches on
// that side. This only matters for a side that keeps relatching this many times over in a single
// wait, which a real, resolving disturbance doesn't do -- see
// test_drive_boundary_hover_resolves_via_rearm_cap.cpp for the pathological case this exists for.
static constexpr int STUCK_WATCH_REARM_CAP = 4;
// The ceiling stuck_step() ever hands a stuck watch, regardless of how loose a team's own small_error is --
// the shipped defaults (drive_defaults_set(), drive.cpp): 1 in for xy/DRIVE, 3 deg for TURN/SWING/angle. A
// stuck watch's floor speed is small_error/window (or, with small_error unset, the velocity exit's own noise
// floor -- unaffected by this cap either way), so an uncapped small_error lets a team's own tuning raise the
// speed a robot has to beat to not be called stuck. Loosening small_error to go faster is exactly what a team
// tuning for speed does; it must not also raise this floor. Shortening a wait's own velocity_exit_time (the
// window, not the step) still raises the floor -- deliberately left alone, a separate design call.
static constexpr double STUCK_STEP_DISTANCE_CAP = 1.0;
static constexpr double STUCK_STEP_ANGLE_CAP = 3.0;
// How close a wait_until() target has to be to the motion's actual final target to count as
// literally the same target, not just a waypoint short of it -- used only to decide whether
// wait_until_drive()/wait_until_turn_swing_internal() get the same "settled inside big error
// counts as done" exemption pid_wait() already has (see those two functions' own comments).
// There's no existing float-equality tolerance elsewhere in this codebase to match (the
// concurrent-retarget guards next to this compare with exact !=, which is checking identity of
// an unmodified double, not equality of two independently-computed target values); this is
// picked instead to comfortably clear ordinary double round-trip error (e.g. target computed via
// (a - b) + b, or through a QLength/QAngle unit conversion and back) while staying many orders of
// magnitude tighter than any real waypoint a caller would intentionally place near the final
// target -- inches/degrees are never meaningfully specified to this precision.
static constexpr double FINAL_TARGET_TOLERANCE = 1e-6;
std::uint32_t stuck_passes() { return ez::detail::stats.auto_task_passes.load(std::memory_order_relaxed); }
// The progress step a stuck watch's Channel uses when small_error isn't set: without it, step would be
// 0 and a Channel this size never gets more lenient, just a strict "any decrease at all is progress"
// check -- fine on its own, but multiplying the velocity exit's own noise floor (velocity_zero_main, the
// per-pass reading it already treats as "not really moving") by how many passes fit in the watch's window
// used to be here instead, turning that into a distance-per-window figure. Dividing back out by the
// window, that figure is exactly velocity_zero_main/DELAY_TIME -- the velocity exit's own gating speed --
// so a robot cruising steadily under that speed but still closing on its target for real read as making
// no progress for a whole window and false-aborted, the same speed floor DRIVE/TURN/SWING's own waits are
// no longer allowed to be gated on reappearing through the stuck watch's back door. Using
// velocity_zero_main directly, un-scaled by the window, keeps the same per-pass noise floor without
// reintroducing that speed gate: any single pass' worth of real forward motion above the noise floor
// still counts as a new low, at any cruise speed.
double stuck_step(PID& pid, double cap) {
  if (pid.exit.small_error > 0) return std::fmin(pid.exit.small_error, cap);
  return pid.velocity_sensor_main_exit_get();
}

// StuckWatch's stuck-detection window, in ms: xy's own velocity_exit_time if set, else xy's own
// mA_timeout, else -- a team can zero both of xy's own velocity and current exits, a legitimate,
// already-supported per-axis choice with no hard ceiling -- angle's own equivalent instead, so turning
// off xy's own exits can't silently take heading's stuck backstop down with it too. pid_odom_drive_exit_
// condition_set() and pid_odom_turn_exit_condition_set() are two separate public setters specifically so
// a team can configure each axis independently; the window they feed has to stay independently backed by
// each axis's own configuration for that independence to actually hold. Only if BOTH axes have zeroed
// both their own velocity and current exits does this collapse to 0 and stuck() go permanently inert --
// matching what a team who deliberately disabled every timeout on both axes is actually asking for.
int stuck_window(PID& xy, PID& angle) {
  int xy_window = xy.exit.velocity_exit_time != 0 ? xy.exit.velocity_exit_time : xy.exit.mA_timeout;
  if (xy_window != 0) return xy_window;
  return angle.exit.velocity_exit_time != 0 ? angle.exit.velocity_exit_time : angle.exit.mA_timeout;
}

// A single progress channel: has `size` (always >= 0, e.g. a distance or |error|) come down to a new low, a full
// `step` below the last one, since progress was last credited?  Going past the point (`error`'s sign flipping) and
// coming back counts, measured from how far past it went, and so does a straight shove that pushes `size` a full
// step worse without ever crossing the point -- either way only once per DISTURBANCE: a robot being spun or shoved
// back and forth crosses (or is pushed away from) its target over and over, and only the first excursion of a
// given disturbance gets credited toward a full recovery being required. A later, genuinely separate disturbance
// gets its own credit again, but only once this one has been recovered from for real -- see `anchor` below for
// exactly what that requires.
struct Channel {
  double step, low;
  bool side, rebound = false, rebounded = false;
  // The `low` this channel stood at right before its current disturbance began -- captured the moment `rebounded`
  // latches, before the disturbance is allowed to raise `low` at all. Recovering merely past the disturbance's OWN
  // peak (the `size >= low - step` check just below) is enough to stop counting it as still-ongoing and credit a
  // step of progress, but that alone isn't enough to consider the disturbance over and done: it's also exactly the
  // bar a channel oscillating right at the edge clears on every single cycle, by construction, so re-arming there
  // would re-arm on every poll and defeat the "only once per disturbance" guarantee the class comment above
  // promises. Instead, re-arming (below) requires `low` to fall a full extra step past THIS `anchor` -- real,
  // additional headway beyond where the channel already stood before the disturbance hit, not just recovery from
  // the disturbance itself. A fixed-amplitude oscillation never manufactures that (each cycle's trough lands
  // above, not below, its own cycle's anchor), while a genuinely separate later disturbance -- one preceded by
  // real further progress, per the issue's own repro -- clears it before that later disturbance ever begins.
  double anchor = 0;
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
    if ((overshot || shoved) && !rebounded) {
      rebound = rebounded = true;
      anchor = low;
    }
    side = error > 0;
    if (rebound) low = std::fmax(low, size);
    if (size >= low - step) return false;
    low = size;
    rebound = false;
    // The current disturbance is only now considered fully closed out -- eligible to let a LATER, separate
    // disturbance re-latch and get its own leniency -- once recovery has carried `low` a full step past where
    // this one started, not merely past its own peak. See `anchor`'s comment above for why that margin, not just
    // "recovered at all", is what keeps a channel oscillating at a fixed amplitude from re-arming itself every
    // poll.
    if (rebounded && low < anchor - step) rebounded = false;
    return true;
  }
};

// Tells an odom wait when the robot is stuck: no progress for the xy velocity exit's time.  Progress is pure
// pursuit moving onto a new point, or the distance to the point being driven to or the heading error coming down
// to a new low, a full step below the last one -- that PID's small exit error if it has one set, otherwise the
// velocity exit's own per-pass noise floor (stuck_step(), above).  That holds at any heading error and
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
      : xy_(stuck_step(xy, STUCK_STEP_DISTANCE_CAP), distance, xy.error), a_(stuck_step(angle, STUCK_STEP_ANGLE_CAP), std::fabs(angle.error), angle.error), index_(index), window_(stuck_window(xy, angle)), moved_(travelled > xy_.step || turned > a_.step), a_seed_pass_(stuck_passes()), a_seeded_(false) {
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
    // The angle channel's own construction-time seed (angle.error at that moment) can be a leftover
    // reading from the PREVIOUS motion: motion_reset()/timers_reset() never touch `error`, only a real
    // compute_error() does, so if this wait's own first tick lands before the background task has
    // ticked even once since construction, a_error here is still that stale value, not a real one.
    // Unlike xy_ (seeded from `distance`, a value this call's own caller recomputes fresh every time
    // from actual position, never from a stored PID field), the angle channel has nothing else to seed
    // from, so it needs the same freshness guard SingleStuckWatch's own `seeded_` already gives its
    // single channel: running a stale-to-real jump through made() would read the eventual real, fresh
    // reading as a shove away from a baseline that was never real, spending Channel's one-shot rebound
    // leniency on nothing before any genuine disturbance happens. Deferring trust until
    // stuck_passes() shows the background task has actually ticked since construction, and re-seeding
    // directly from that first confirmed-fresh reading instead of running it through made(), keeps this
    // channel's baseline -- and its rebound leniency -- meant for a real disturbance.
    if (!a_seeded_) {
      if (pass != a_seed_pass_) {
        a_ = Channel(a_.step, std::fabs(a_error), a_error);
        a_seeded_ = true;
      }
    } else if (a_.made(std::fabs(a_error), a_error)) {
      progress = true;
    }
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
  // stuck_passes() at construction, and whether the angle channel has re-seeded itself from the first
  // confirmed-fresh reading since -- see the matching comment in stuck() above.
  std::uint32_t a_seed_pass_;
  bool a_seeded_;
  std::uint32_t last_progress_, last_progress_pass_;
};

// The same progress backstop as StuckWatch, but for a single PID with no odometry and no path index -- DRIVE,
// TURN and SWING have none of StuckWatch's protection today (only velocity/mA exits, both defeated by a sustained
// disturbance that never reads as "stopped" and never draws over current -- a spin, a defender holding the robot
// mid-turn, sensor jitter under contact).  "Moved" is judged from this same PID's own error closing by a step from
// where it started, since these modes have no separate odometry-derived travelled/turned to check against.
class SingleStuckWatch {
 public:
  // `already_moved`: whether the motion this PID belongs to has already moved a real step's worth of
  // progress since ITS OWN start -- not since this particular wait call started -- so a wait chained
  // onto an already-moving motion doesn't pay the startup allowance again. Mirrors what StuckWatch's own
  // comment documents as the intended contract ("a second wait on the same motion doesn't get the
  // allowance again once the robot has moved") and already keeps for odom waits by seeding its own
  // `moved_` from real distance/heading travelled since the motion's true start; SingleStuckWatch has no
  // odometry of its own, so callers compute this the same way DRIVE/TURN/SWING already track a motion's
  // real start elsewhere (l_start/r_start, chain_sensor_start).
  // `cap`: STUCK_STEP_DISTANCE_CAP for a distance-type PID (DRIVE), STUCK_STEP_ANGLE_CAP for an angle-type
  // one (TURN/SWING) -- this PID's own small_error alone doesn't say which, so the caller (which already
  // knows) passes it in, same as StuckWatch's constructor already picks the right one for xy_ vs a_.
  SingleStuckWatch(PID& pid, double error, bool already_moved, double cap)
      : ch_(stuck_step(pid, cap), std::fabs(error), error), window_(pid.exit.velocity_exit_time != 0 ? pid.exit.velocity_exit_time : pid.exit.mA_timeout), moved_(already_moved), last_pass_(stuck_passes()), seeded_(false) {
    int allowance = moved_ ? 0 : STUCK_START_ALLOWANCE_MS;
    last_progress_ = pros::millis() + allowance;
    last_progress_pass_ = stuck_passes() + allowance / util::DELAY_TIME;
  }

  bool stuck(double error) {
    if (window_ == 0) return false;
    std::uint32_t now = pros::millis();
    std::uint32_t pass = stuck_passes();
    bool progress = false;
    // Only ever act on `error` on a pass where the background task has actually ticked since the last
    // time this checked (stuck_passes(), the same heartbeat the wall-clock/pass-count starvation check
    // below already trusts) -- the caller (this wait's own loop) and that background compute loop are two
    // independently scheduled loops that aren't lock-stepped, so `error` isn't guaranteed to be a new
    // value just because stuck() was called again. The first confirmed-fresh read re-seeds the channel's
    // baseline directly from it instead of running it through made(): whatever `error` this watch happened
    // to be constructed with -- right after pid_wait()'s own leading delay, which doesn't guarantee a real
    // tick landed during it either -- is never trusted as a real baseline on its own, so a stale-to-real
    // jump on the first real read can't misread as a shove and spend the underlying Channel's one-shot
    // rebound allowance on nothing. A pass with nothing new is skipped outright, not just left
    // un-credited: last_progress_/last_progress_pass_ (seeded from real construction time regardless,
    // below) are what still catch a task that stops ticking forever -- treating an unchanged reading as
    // "checked and still not stuck" would reset that clock for no reason every single pass.
    if (pass != last_pass_) {
      last_pass_ = pass;
      if (!seeded_) {
        ch_ = Channel(ch_.step, std::fabs(error), error);
        seeded_ = true;
      } else {
        progress = ch_.made(std::fabs(error), error);
      }
    }
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
  std::uint32_t last_pass_;
  bool seeded_;
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

// See drive.hpp's own comment on InterferedScope/motion_generation/interfered_generation for the concurrency
// hazard this exists to fix: every retarget guard below used to end a stale wait with a bare
// `interfered = true; return;`, landing on the single, un-scoped `interfered` bool no matter which motion a
// caller is actually about to read it for. Tagging the scope with the generation this wait's own motion was
// started under -- and only asserting a clean interfered=false result for that same generation on the way out
// -- keeps a stale wait's notice about ITS OWN abandoned motion from being misread as a report on a
// completely different, later motion that never had any problem of its own.
Drive::InterferedScope::InterferedScope(Drive& d) : d_(d) {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(d_.drive_mutex);
  generation_ = d_.motion_generation;
}

// Attributes an interfered=true write to this scope's own motion. Every retarget-guard / stuck / mA / velocity
// write in this file goes through this instead of writing `interfered` directly, so the destructor below can
// tell its own scope's writes apart from one a different motion made.
void Drive::InterferedScope::mark() {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(d_.drive_mutex);
  d_.interfered = true;
  d_.interfered_generation = generation_;
}

Drive::InterferedScope::~InterferedScope() {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(d_.drive_mutex);
  // Only assert a clean result for THIS motion if it's still the current one (a newer motion that has since
  // started owns the flag now, not this stale wait) and nothing has already spoken for it under its own name
  // (this scope's own mark() above, or an earlier wait on the very same motion, e.g. a chained call's own
  // first phase reporting stuck) -- that earlier true must survive a later, clean phase of the same motion.
  if (d_.motion_generation == generation_ && d_.interfered_generation != generation_) {
    d_.interfered = false;
    d_.interfered_generation = generation_;
  }
}

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
  // Snapshotted BEFORE the leading settle delay below, not after: every branch this function can
  // run (DRIVE/odom/TURN/SWING) decides which one even runs from `mode` read fresh once the delay
  // ends, so a concurrent motion setter from a DIFFERENT mode landing during this delay is
  // invisible to a snapshot taken afterward -- that snapshot already reflects the new mode, so this
  // call silently runs the wrong branch entirely (with that branch's own internal retarget guard
  // then comparing the hijacking motion against itself) instead of ending the wait it was actually
  // started for. Every branch's own family target is snapshotted here too, for the same reason: each
  // branch's own mid-loop guard below compares against these, and they need to reflect the motion
  // this call actually started for, not whatever a SAME-mode retarget already landed during this
  // same delay -- a concurrent pid_turn_set()/pid_swing_set()/pid_odom_*_set() of the same family
  // doesn't change `mode` at all, so the check just above can't catch it; only a branch reading its
  // own target from a snapshot taken before this delay (instead of re-reading the live PID/
  // odom_target_start afterward, which would already reflect the hijacking motion) can.
  e_mode entry_mode_snapshot = mode;
  double entry_left_target = leftPID.target_get();
  double entry_right_target = rightPID.target_get();
  double entry_turn_target = turnPID.target_get();
  double entry_swing_target = swingPID.target_get();
  pose entry_odom_target_start = odom_target_start;
  // ez_auto_task's pass count as of this call. The stuck backstop's "settled" carve-out below reads a PID's
  // `error`, which only a real compute changes -- motion_reset() leaves the previous motion's last error in
  // place -- so if the task has not run at all since this wait began (starved or dead), that error is still
  // the previous motion's, and it must not read as settled at this motion's target.
  const std::uint32_t entry_task_passes = stuck_passes();
  // Scopes every `interfered` write below to the motion this call was actually started for -- see
  // drive.hpp's comment on InterferedScope. Opened here, at the same instant as the snapshots above and
  // before any of them can go stale, so it tags the motion this wait is really waiting on, not whatever a
  // same-mode retarget already landed during the leading settle delay below.
  InterferedScope interfered_scope(*this);

  // Let the PID run at least 1 iteration
  pros::delay(util::DELAY_TIME);

  if (mode != entry_mode_snapshot) {
    if (print_toggle) std::cout << "  Drive: retargeted by a concurrent motion during the wait's own first pass, ending early instead of running the wrong branch.\n";
    interfered_scope.mark();
    return;
  }

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
    e_mode mode_snapshot = entry_mode_snapshot;
    double left_target = entry_left_target;
    double right_target = entry_right_target;
    // DRIVE has no odometry to fall back on, so this is the same progress backstop StuckWatch gives odom waits,
    // built around each side's own error instead: neither velocity nor mA catches a sustained disturbance that
    // never reads as "stopped" and never draws over current (a continuous spin, a defender holding the robot,
    // sensor jitter under contact) -- see WAIT_BEHAVIOR_SPEC.md's JC-1.  A side only counts as "stuck" once IT is
    // still RUNNING and not progressing; a side that already exited cleanly can't drag the wait down.  Whether
    // each side already moved is judged against l_start/r_start -- the motion's own real start, set once in
    // pid_drive_set() -- not against this particular wait call, so a wait chained onto an already-moving motion
    // (an early pid_wait_until() checkpoint followed by pid_wait() on the same still-running drive, TEAM_CORPUS.md's
    // ordinary "until then wait" pattern) doesn't pay SingleStuckWatch's startup allowance a second time.
    SingleStuckWatch left_watch(leftPID, leftPID.error, std::fabs(drive_sensor_left() - l_start) > stuck_step(leftPID, STUCK_STEP_DISTANCE_CAP), STUCK_STEP_DISTANCE_CAP),
        right_watch(rightPID, rightPID.error, std::fabs(drive_sensor_right() - r_start) > stuck_step(rightPID, STUCK_STEP_DISTANCE_CAP), STUCK_STEP_DISTANCE_CAP);
    // How many times the recheck below has reseeded each side's watch on an un-latch -- see
    // STUCK_WATCH_REARM_CAP's own comment. Local to this one pid_wait() call, same as the watches
    // themselves, so every new wait starts a fresh count regardless of how many times a previous
    // wait on this same motion used up its own cap.
    int left_stuck_watch_rearm_count = 0;
    int right_stuck_watch_rearm_count = 0;
    bool stalled = false;
    // A stuck-but-settled break below is its own, already-final decision (at least one side never
    // finished its own exit timer at all -- still RUNNING -- but the stuck watch gave up waiting on
    // it and every side that's still RUNNING is currently inside its own big error window anyway).
    // That is not the "both sides independently latched a window exit" case the recheck below is
    // for, and there is nothing for it to un-latch (a side that's still RUNNING was never latched in
    // the first place) -- falling into it here would just delay-and-loop forever, since neither the
    // settled check nor the stuck check resets. Stop here, same as the original code did -- see the
    // matching flag and comment in pid_wait()'s odom branch above.
    bool settled_via_stuck = false;
    while (true) {
      while (!stalled && (left_exit == RUNNING || right_exit == RUNNING)) {
        if (mode != mode_snapshot || leftPID.target_get() != left_target || rightPID.target_get() != right_target) {
          if (print_toggle) std::cout << "  Drive: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
          interfered_scope.mark();
          return;
        }
        secondary_velocity_sensor_update(leftPID);
        secondary_velocity_sensor_update(rightPID);
        // A genuinely slow (not stalled) DRIVE cruise can read as "stopped" to the velocity channel --
        // PID.cpp's velocity floor is a fixed constant, independent of gearing or wheel size, so a real,
        // low-gearing drivetrain cruising under that floor is not stalled, just slow. Odom already filters
        // this out (without_velocity(), above); DRIVE didn't. SingleStuckWatch (below) still catches a
        // genuine stall independently -- it watches PID error, not velocity -- so filtering this out can't
        // turn a real stall into a hang.
        left_exit = left_exit != RUNNING ? left_exit : without_velocity(leftPID.exit_condition(left_motors));
        right_exit = right_exit != RUNNING ? right_exit : without_velocity(rightPID.exit_condition(right_motors));
        bool left_stuck = left_exit == RUNNING && left_watch.stuck(leftPID.error);
        bool right_stuck = right_exit == RUNNING && right_watch.stuck(rightPID.error);
        // Stuck only when at least one side is still RUNNING and every side that's still RUNNING is stuck --
        // not when both sides just happened to exit normally on the same pass, which the (exit != RUNNING) half
        // of each clause would otherwise also satisfy.
        if ((left_exit == RUNNING || right_exit == RUNNING) && (left_exit != RUNNING || left_stuck) && (right_exit != RUNNING || right_stuck)) {
          // A side that's still RUNNING and already sitting inside its own big error window is where a big
          // exit would have left it: that's settled, not stuck. (A side hovering across its own small error
          // window can keep both its own exit timers from ever finishing.) A side that already latched an
          // exit is trusted here only if its OWN live error is still inside its big error window right now --
          // a side that latched early and has since been shoved or pinned off target must not count as
          // settled just because it once exited cleanly (see the recheck below, which applies this identical
          // rule to a full double latch instead of this stuck-detected path).
          bool left_settled = std::fabs(leftPID.error) < leftPID.exit.big_error;
          bool right_settled = std::fabs(rightPID.error) < rightPID.exit.big_error;
          // ...and only if ez_auto_task has run since this wait began (see entry_task_passes above); if it
          // has not, both errors are the previous motion's leftovers, not a reading of this one.
          bool settled = left_settled && right_settled && stuck_passes() != entry_task_passes;
          stalled = !settled;
          settled_via_stuck = settled;
          if (print_toggle) std::cout << "  Drive: " << (stalled ? "Stuck" : "Stuck, but stopped inside the big error windows, counted as settled") << ", error: L," << leftPID.error << " R," << rightPID.error << "\n";
          break;
        }
        pros::delay(util::DELAY_TIME);
      }
      if (stalled || settled_via_stuck) break;

      // Both sides read as exited (or this loop wouldn't have exited above). Right before trusting
      // a clean double-exit, recheck each side that latched a window exit (SMALL_EXIT/BIG_EXIT)
      // against the same window it exited through, using its own live error -- not exit_condition()
      // (that would restart its internal timers) -- the same treatment pid_wait()'s odom branch
      // already gives a clean double-exit (see the comment there). A side that has drifted back
      // outside since latching is un-latched (back to RUNNING) so the inner loop above keeps
      // genuinely watching it instead of trusting a stale result.
      //
      // Its own stuck watch IS reseeded here, right at the point of un-latching (accepted tradeoff,
      // see GitHub issue #532): a side stops being fed the moment it first latches, so once the
      // OTHER side keeps the wait going for a while, that side's watch clock is left frozen at
      // whatever it read back when it latched -- not resynced to "now". Left as-is, the very next
      // stuck() call on that side measures elapsed time against a clock that's been stale since
      // before this new disturbance even started, which can already exceed the watch's own window
      // purely from idle time spent waiting on the sibling -- an ordinary, self-resolving blip that
      // happens to land on this recheck gets flagged stuck instantly, with zero real grace period.
      // Reseeding here -- the same construction used when a motion's wait first starts, with
      // `already_moved=true` since this side plainly already moved to have latched in the first
      // place -- gives the newly-unlatched disturbance a genuinely fresh window instead.
      //
      // Bounded by STUCK_WATCH_REARM_CAP, not unconditional: reseeding every single relatch forever
      // measurably hangs pid_wait() outright for a side that keeps oscillating right at its own
      // small_error boundary (each un-latch here resets this watch's clock at the same moment PID's
      // OWN small/big-exit timers also reset on their own boundary crossing, so a relatch cadence
      // faster than this watch's window starves BOTH backstops indefinitely -- confirmed, not just
      // theorized: see the PR discussion on issue #532 for the measured repro). Past the cap, this
      // side stops being reseeded for the rest of THIS wait and falls back to the pre-fix un-reseeded
      // behavior -- its own frozen clock is what eventually ends a relatch loop like that, same as it
      // did before this fix existed, bounding the pathological case while a realistic, resolving
      // disturbance (which relatches at most a handful of times, not dozens) never comes close to the
      // cap and keeps getting the fresh-window fix in full. VELOCITY_EXIT is never latched here
      // (without_velocity() already maps it to RUNNING); mA_EXIT and ERROR_NO_CONSTANTS aren't
      // window exits and are left alone -- the final check below still catches mA_EXIT/VELOCITY_EXIT
      // regardless of this recheck.
      if (left_exit == SMALL_EXIT && std::fabs(leftPID.error) >= leftPID.exit.small_error) {
        left_exit = RUNNING;
        if (left_stuck_watch_rearm_count < STUCK_WATCH_REARM_CAP) {
          left_watch = SingleStuckWatch(leftPID, leftPID.error, true, STUCK_STEP_DISTANCE_CAP);
          ++left_stuck_watch_rearm_count;
        }
      } else if (left_exit == BIG_EXIT && std::fabs(leftPID.error) >= leftPID.exit.big_error) {
        left_exit = RUNNING;
        if (left_stuck_watch_rearm_count < STUCK_WATCH_REARM_CAP) {
          left_watch = SingleStuckWatch(leftPID, leftPID.error, true, STUCK_STEP_DISTANCE_CAP);
          ++left_stuck_watch_rearm_count;
        }
      }
      if (right_exit == SMALL_EXIT && std::fabs(rightPID.error) >= rightPID.exit.small_error) {
        right_exit = RUNNING;
        if (right_stuck_watch_rearm_count < STUCK_WATCH_REARM_CAP) {
          right_watch = SingleStuckWatch(rightPID, rightPID.error, true, STUCK_STEP_DISTANCE_CAP);
          ++right_stuck_watch_rearm_count;
        }
      } else if (right_exit == BIG_EXIT && std::fabs(rightPID.error) >= rightPID.exit.big_error) {
        right_exit = RUNNING;
        if (right_stuck_watch_rearm_count < STUCK_WATCH_REARM_CAP) {
          right_watch = SingleStuckWatch(rightPID, rightPID.error, true, STUCK_STEP_DISTANCE_CAP);
          ++right_stuck_watch_rearm_count;
        }
      }

      if (left_exit == RUNNING || right_exit == RUNNING) {
        pros::delay(util::DELAY_TIME);
        continue;
      }
      break;
    }
    // Guarded the same way the odom branch below already is: a settled-via-stuck break above leaves
    // both sides RUNNING (they never actually latched a window exit at all), so printing their
    // exit_to_string() here would print a nonsensical "Running Exit" right after the "counted as
    // settled" message the stuck check above already printed for this same pass.
    if (print_toggle && !stalled && left_exit != RUNNING && right_exit != RUNNING) std::cout << "  Left: " << exit_to_string(left_exit) << " Exit, error: " << leftPID.error << "   Right: " << exit_to_string(right_exit) << " Exit, error: " << rightPID.error << "\n";

    if (stalled || left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
      interfered_scope.mark();
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
    // mode_snapshot/retarget_target reuse the entry snapshot taken before this function's own leading
    // settle delay above -- see its comment -- not a fresh read here, so a SAME-family odom retarget
    // landing during that delay is caught on this branch's own first pass instead of being adopted as
    // this call's own baseline.
    e_mode mode_snapshot = entry_mode_snapshot;
    pose retarget_target = entry_odom_target_start;

    // Wait until pure pursuit is on the last point, then continue as normal.  xy's exit is checked every pass
    // and not kept: before the last point its target is only a look ahead away and keeps moving, so a small,
    // big or velocity exit here says nothing about the path, and one kept from a pause earlier on must not end
    // the wait later.  A current exit still ends it.  Angle's exit is kept across a pass the same way xy's isn't
    // meant to be -- but it must not carry past the point it settled at: a heading exit latched on an earlier,
    // unrelated point can't stand in for the final point's actual heading, so it's reset every time pp_index
    // moves to a new point, including the last move into the final point below.  Within a single point, a
    // latched axis is also rechecked right before a clean double-exit is trusted (below, in the final loop) --
    // see the comment there for why.
    int a_exit_index = pp_index;
    if (mode == PURE_PURSUIT) {
      while (pp_index != (int)pp_movements.size() - 1) {
        if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
          if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of continuing on the wrong path.\n";
          interfered_scope.mark();
          return;
        }
        if (pp_index != a_exit_index) {
          a_exit = RUNNING;
          a_exit_index = pp_index;
        }
        secondary_velocity_sensor_update(xyPID);
        secondary_velocity_sensor_update(current_a_odomPID);
        xy_velocity_exit_hold_update();
        // xyPID's target here is a moving look-ahead point, not the real path (see this loop's own
        // comment above): xy_pass below is only ever inspected for mA_EXIT, and SMALL_EXIT/BIG_EXIT/
        // VELOCITY_EXIT are discarded on purpose. But PID::exit_condition() calls
        // PID::timers_reset() whenever ANY channel latches, which wipes every channel's timer
        // together -- including this same call's own mA progress -- so a SMALL_EXIT on the
        // (discarded) look-ahead error would silently erase real, ongoing over-current progress
        // before it ever reaches mA_timeout (GitHub issue #527). Snapshot the mA state before the
        // call, using the exact same over-current predicate exit_condition(const
        // std::vector<pros::Motor>&) itself uses (a transient PROS_ERR is not itself over-current,
        // but a PROS_ERR paired with a non-finite position is a genuinely dead motor), and restore +
        // re-credit it only when the call's actual result is one of the three discarded channels that
        // call timers_reset() -- SMALL_EXIT, BIG_EXIT, or VELOCITY_EXIT -- the exact set of results
        // this loop's own wipe can happen under; every other result (RUNNING, or a genuine mA_EXIT
        // that already reset l correctly on its own and ends this loop below regardless) leaves the
        // mA state exactly as exit_condition() itself just left it, so a real over-current reading
        // racing between this check and exit_condition()'s own can never be second-guessed by this
        // restore. This is scoped to just this call site: PID::exit_condition()'s general contract
        // (every channel resets together on any latch) is unchanged for every other caller, including
        // xy's own final-point loop below, where every channel's result is actually used, not discarded.
        std::vector<pros::Motor> xy_motors = both_sides(left_motors, right_motors);
        bool xy_mA_tracked = xyPID.exit.mA_timeout != 0;
        bool xy_over_current = false;
        if (xy_mA_tracked) {
          for (auto& motor : xy_motors) {
            std::int32_t xy_over = motor.is_over_current();
            bool xy_dead = xy_over == PROS_ERR && !std::isfinite(motor.get_position());
            if (xy_over == 1 || xy_dead) {
              xy_over_current = true;
              break;
            }
          }
        }
        PID::MATimerSnapshot xy_mA_snapshot = xyPID.mA_timer_snapshot();
        exit_output xy_pass = xyPID.exit_condition(xy_motors);
        if (xy_mA_tracked && xy_over_current && (xy_pass == SMALL_EXIT || xy_pass == BIG_EXIT || xy_pass == VELOCITY_EXIT)) xyPID.mA_timer_restore_and_credit(xy_mA_snapshot);
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

    // When we're at the last point in PP / we're just going to point. The inner loop is exactly the
    // original per-pass loop (same checks, same single trailing delay per pass) -- wrapped in an
    // outer loop only so it can be re-entered below after a relatch, without changing its own timing
    // for the ordinary case where nothing needs relatching.
    bool settled_via_stuck = false;
    while (!stalled) {
      while (!stalled && (xy_exit == RUNNING || a_exit == RUNNING)) {
        if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
          if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
          interfered_scope.mark();
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
          settled_via_stuck = settled;
          if (print_toggle) std::cout << "  XY: " << exit_to_string(xy_exit) << ", error: " << xyPID.error << ".   Angle: " << exit_to_string(a_exit) << ", error: " << current_a_odomPID.error << (settled ? ".   Stopped inside the big error windows, counted as settled.\n" : ".   Stuck before settling on the target.\n");
          break;
        }
        pros::delay(util::DELAY_TIME);
      }
      // A stuck-but-settled break above is its own, already-final decision (at least one axis never
      // finished its own exit timer at all -- still RUNNING -- but StuckWatch gave up waiting on it and
      // both axes are currently inside their big error windows anyway). That is not the "both axes
      // independently latched a window exit" case the recheck below is for, and there is nothing for it
      // to un-latch (an axis that's still RUNNING was never latched in the first place) -- falling into
      // it here would just delay-and-loop forever, since neither the settled check nor the stuck check
      // resets. Stop here, same as the original code did.
      if (stalled || settled_via_stuck) break;

      // Both axes read as exited. The ternaries above only ever call exit_condition() again once an axis is
      // back to RUNNING -- once latched, an axis is never looked at again for the rest of the inner loop, so a
      // disturbance landing after it latched (most commonly angle, which typically settles first) would
      // otherwise go completely unnoticed: this could return a clean, uninterfered exit while the robot is
      // measurably off target on that axis. Recheck each axis that latched a window exit (SMALL_EXIT/
      // BIG_EXIT) against the same window it exited through, using its own live error -- not
      // exit_condition() (that would restart its internal timers) and not target_distance() (that would
      // change what's actually being measured for xy, including during a boomerang leg, where xy's own exit
      // is intentionally carrot-relative to the moving carrot, not the real waypoint). If it has drifted back
      // outside, un-latch it (back to RUNNING) and loop back into the inner loop above to keep waiting.
      // VELOCITY_EXIT is never latched here (without_velocity() already maps it to RUNNING); mA_EXIT and
      // ERROR_NO_CONSTANTS aren't window exits and already force interfered=true below, so they're left
      // alone. If the disturbance never resolves, StuckWatch above is what ends this, not an infinite relatch.
      if (xy_exit == SMALL_EXIT && std::fabs(xyPID.error) >= xyPID.exit.small_error) xy_exit = RUNNING;
      else if (xy_exit == BIG_EXIT && std::fabs(xyPID.error) >= xyPID.exit.big_error) xy_exit = RUNNING;
      if (a_exit == SMALL_EXIT && std::fabs(current_a_odomPID.error) >= current_a_odomPID.exit.small_error) a_exit = RUNNING;
      else if (a_exit == BIG_EXIT && std::fabs(current_a_odomPID.error) >= current_a_odomPID.exit.big_error) a_exit = RUNNING;

      if (xy_exit == RUNNING || a_exit == RUNNING) {
        pros::delay(util::DELAY_TIME);
        continue;
      }

      break;
    }
    if (print_toggle && !stalled && xy_exit != RUNNING && a_exit != RUNNING) std::cout << "  XY: " << exit_to_string(xy_exit) << " Exit, error: " << xyPID.error << ".   Angle: " << exit_to_string(a_exit) << " Exit, error: " << current_a_odomPID.error << ".\n";

    if (stalled || xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT || a_exit == mA_EXIT || a_exit == VELOCITY_EXIT) {
      interfered_scope.mark();
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
    // Moved-since-motion-start is judged against chain_sensor_start (set once in turn_set_internal()), not
    // this particular wait call -- see the DRIVE branch's comment above for why, and same JC-1 gap.
    SingleStuckWatch watch(turnPID, turnPID.error, std::fabs(drive_angle_get() - chain_sensor_start) > stuck_step(turnPID, STUCK_STEP_ANGLE_CAP), STUCK_STEP_ANGLE_CAP);
    bool stalled = false;
    // Same concurrent-retarget guard as the DRIVE branch above.  turnPID.target is only ever rewritten by
    // turn_set_internal() (set_turn_pid.cpp) at the start of a new turn -- TURN_TO_POINT recomputes its own
    // aim point every pass through compute_error() without touching the PID's target, so this can't false-fire
    // on an ordinary turn-to-point motion, only on a real second pid_turn_set()/pid_turn_relative_set().  mode
    // is also watched -- see the DRIVE branch's comment on why a target-only check misses a cross-mode retarget.
    // mode_snapshot/turn_target reuse the entry snapshot taken before this function's own leading
    // settle delay above, not a fresh read here -- see that snapshot's comment -- so a SAME-family
    // turn retarget landing during that delay is caught on this branch's own first pass too.
    e_mode mode_snapshot = entry_mode_snapshot;
    double turn_target = entry_turn_target;
    // A stuck-but-settled break below is its own, already-final decision (turn_exit never finished its
    // own exit timer at all -- still RUNNING -- but the stuck watch gave up waiting on it and it's
    // currently inside its own big error window anyway). That is not the "latched a window exit" case
    // the recheck below is for, and there is nothing for it to un-latch (still RUNNING was never
    // latched in the first place) -- falling into it here would just delay-and-loop forever, since
    // neither the settled check nor the stuck check resets. Stop here, same as the DRIVE branch's
    // matching flag and comment above.
    bool settled_via_stuck = false;
    while (true) {
      while (turn_exit == RUNNING) {
        if (mode != mode_snapshot || turnPID.target_get() != turn_target) {
          if (print_toggle) std::cout << "  Turn: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
          interfered_scope.mark();
          return;
        }
        secondary_velocity_sensor_update(turnPID);
        // See the matching comment in the DRIVE branch above -- a slow (not stalled) turn must not be
        // ended by the velocity channel alone.
        turn_exit = turn_exit != RUNNING ? turn_exit : without_velocity(turnPID.exit_condition(both_sides(left_motors, right_motors)));
        if (turn_exit == RUNNING && watch.stuck(turnPID.error)) {
          // Same settled carve-out as the DRIVE branch above.
          bool settled = std::fabs(turnPID.error) < turnPID.exit.big_error && stuck_passes() != entry_task_passes;
          stalled = !settled;
          settled_via_stuck = settled;
          if (print_toggle) std::cout << "  Turn: " << (stalled ? "Stuck" : "Stuck, but stopped inside the big error window, counted as settled") << ", error: " << turnPID.error << "\n";
          break;
        }
        pros::delay(util::DELAY_TIME);
      }
      if (stalled || settled_via_stuck) break;

      // turn_exit latched a window exit (SMALL_EXIT/BIG_EXIT) on the pass that just ended the loop
      // above -- but that loop's own trailing pros::delay() still ran once more after the pass that
      // latched, before the loop condition was re-checked, so a disturbance landing during that one
      // extra pass was never looked at again. Right before trusting this as a clean return, recheck it
      // against the same window it exited through, using its own live error -- not exit_condition()
      // (that would restart its internal timers) -- the same treatment the DRIVE and odom branches
      // above already give a clean exit. Un-latch (back to RUNNING) if it has drifted back outside,
      // falling through to keep genuinely watching it instead of trusting a stale result. The stuck
      // watch is deliberately NOT reseeded here: unlike DRIVE's two independent per-side watches
      // (which can sit idle for a long stretch waiting on a sibling side that's still running -- see
      // issue #532 and that recheck's own comment on why it now reseeds despite this same tradeoff),
      // TURN has only this one axis, so this watch is only ever idle for the single extra pass since
      // it latched before this recheck runs -- there's no analogous "idle the whole time a sibling
      // keeps going" gap to fix here. Reseeding on every relatch anyway would just hand a pathological
      // hover right at the window's edge (latch, drift out, un-latch, re-latch, ...) an unbounded
      // fresh grace period for free, with nothing real gained.
      // VELOCITY_EXIT is never latched here (without_velocity() already maps it to RUNNING); mA_EXIT
      // isn't a window exit and is handled below regardless, so it's left alone.
      if (turn_exit == SMALL_EXIT && std::fabs(turnPID.error) >= turnPID.exit.small_error) {
        turn_exit = RUNNING;
      } else if (turn_exit == BIG_EXIT && std::fabs(turnPID.error) >= turnPID.exit.big_error) {
        turn_exit = RUNNING;
      }

      if (turn_exit == RUNNING) {
        pros::delay(util::DELAY_TIME);
        continue;
      }
      break;
    }
    // See the DRIVE branch's matching comment above -- a settled-via-stuck break leaves turn_exit
    // RUNNING, so this must not print its exit_to_string() right after the "counted as settled"
    // message already printed for this same pass.
    if (print_toggle && !stalled && turn_exit != RUNNING) std::cout << "  Turn: " << exit_to_string(turn_exit) << " Exit, error: " << turnPID.error << "\n";

    if (stalled || turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
      interfered_scope.mark();
    }
  }

  // Swing Exit
  else if (mode == SWING) {
    exit_output swing_exit = RUNNING;
    // Moved-since-motion-start is judged against chain_sensor_start (set once in swing_set_internal()), not
    // this particular wait call -- see the DRIVE branch's comment above for why, and same JC-1 gap.
    SingleStuckWatch watch(swingPID, swingPID.error, std::fabs(drive_angle_get() - chain_sensor_start) > stuck_step(swingPID, STUCK_STEP_ANGLE_CAP), STUCK_STEP_ANGLE_CAP);
    bool stalled = false;
    // Same concurrent-retarget guard as the DRIVE branch above -- swingPID.target is only rewritten by
    // swing_set_internal() (set_swing_pid.cpp) at the start of a new swing.  mode is also watched -- see the
    // DRIVE branch's comment on why a target-only check misses a cross-mode retarget.
    // mode_snapshot/swing_target reuse the entry snapshot taken before this function's own leading
    // settle delay above, not a fresh read here -- same reason as the TURN branch's matching comment.
    e_mode mode_snapshot = entry_mode_snapshot;
    double swing_target = entry_swing_target;
    // Same stuck-but-settled carve-out as the TURN branch above -- see its comment for why this must
    // stop here rather than fall into the recheck below.
    bool settled_via_stuck = false;
    while (true) {
      while (swing_exit == RUNNING) {
        if (mode != mode_snapshot || swingPID.target_get() != swing_target) {
          if (print_toggle) std::cout << "  Swing: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
          interfered_scope.mark();
          return;
        }
        secondary_velocity_sensor_update(swingPID);
        // See the matching comment in the DRIVE branch above -- a slow (not stalled) swing must not be
        // ended by the velocity channel alone. Polls both sides' motors, not just the actively-swinging
        // side's, the same as the TURN branch above -- swing_pid_task() (pid_tasks.cpp) actively drives
        // the held (non-swinging) side with its own PID output whenever swing_opposite_speed is 0 (the
        // default), so it can genuinely stall/over-current too (e.g. a defender pinning it while the
        // swinging side is unobstructed); checking only the swinging side's motors missed that entirely.
        swing_exit = swing_exit != RUNNING ? swing_exit : without_velocity(swingPID.exit_condition(both_sides(left_motors, right_motors)));
        if (swing_exit == RUNNING && watch.stuck(swingPID.error)) {
          // Same settled carve-out as the DRIVE branch above.
          bool settled = std::fabs(swingPID.error) < swingPID.exit.big_error && stuck_passes() != entry_task_passes;
          stalled = !settled;
          settled_via_stuck = settled;
          if (print_toggle) std::cout << "  Swing: " << (stalled ? "Stuck" : "Stuck, but stopped inside the big error window, counted as settled") << ", error: " << swingPID.error << "\n";
          break;
        }
        pros::delay(util::DELAY_TIME);
      }
      if (stalled || settled_via_stuck) break;

      // Same relatch hazard, and the same recheck-and-un-latch treatment, as the TURN branch above --
      // see its comment for why the one pass between a latch and this loop noticing it is a real,
      // unwatched window.
      if (swing_exit == SMALL_EXIT && std::fabs(swingPID.error) >= swingPID.exit.small_error) {
        swing_exit = RUNNING;
      } else if (swing_exit == BIG_EXIT && std::fabs(swingPID.error) >= swingPID.exit.big_error) {
        swing_exit = RUNNING;
      }

      if (swing_exit == RUNNING) {
        pros::delay(util::DELAY_TIME);
        continue;
      }
      break;
    }
    // See the DRIVE branch's matching comment above -- a settled-via-stuck break leaves swing_exit
    // RUNNING, so this must not print its exit_to_string() right after the "counted as settled"
    // message already printed for this same pass.
    if (print_toggle && !stalled && swing_exit != RUNNING) std::cout << "  Swing: " << exit_to_string(swing_exit) << " Exit, error: " << swingPID.error << "\n";

    if (stalled || swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) {
      interfered_scope.mark();
    }
  }
}

void Drive::wait_until_drive(double target) {
  // Make sure mode is correct -- checked, and the retarget-detection snapshot below taken, BEFORE
  // the leading settle delay: a concurrent motion setter changing mode or retargeting during that
  // delay must be caught by the loop's own guard on its very first pass, not read afterward as
  // already-the-new-motion (silently adopting it) or as the wrong mode (returning with no
  // interfered signal at all) -- the same hazard wait_until_turn_swing_internal() and
  // pid_wait_until_index_started() are already fixed for.
  if (!(mode == DRIVE || mode == POINT_TO_POINT || mode == PURE_PURSUIT)) {
    printf("Mode needs to be drive!\n");
    return;
  }
  // An odom move's leftPID/rightPID target is a fixed look-ahead point set once at the start of the motion (see
  // without_position_exits()'s comment above); a plain DRIVE move's leftPID/rightPID target already is the real
  // drive target.  Only the odom case needs the exit-condition filtering and the real-distance-keyed backstop below
  // -- DRIVE's own target already tracks what this wait is actually waiting for.
  bool is_odom = mode == POINT_TO_POINT || mode == PURE_PURSUIT;
  // Same concurrent-retarget guard as pid_wait()'s branches.  In DRIVE mode leftPID/rightPID's target only
  // changes on a real second pid_drive_set(), same as pid_wait()'s DRIVE branch.  In odom modes (this function
  // also runs for POINT_TO_POINT/PURE_PURSUIT) leftPID/rightPID's target is the fixed look-ahead point and
  // legitimately gets rewritten on every ordinary waypoint advance (raw_pid_odom_ptp_set(), set_odom_pid.cpp),
  // so odom_target_start -- only touched by a top-level odom setter starting a genuinely new motion -- is used
  // instead, the same as pid_wait()'s odom branch.  mode is watched on top of both: a concurrent setter from a
  // mode that isn't even DRIVE/POINT_TO_POINT/PURE_PURSUIT (e.g. a real second pid_turn_set()) touches neither
  // leftPID/rightPID's target nor odom_target_start, so without this it would go unnoticed.
  e_mode mode_snapshot = mode;
  bool odom_mode = is_odom;
  double left_target = leftPID.target_get();
  double right_target = rightPID.target_get();
  pose retarget_target = odom_target_start;
  // See pid_wait()'s matching comment on InterferedScope -- opened here, before the leading settle delay
  // below, for the same reason the snapshots just above are taken here too.
  InterferedScope interfered_scope(*this);

  pros::delay(10);

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

  exit_output left_exit = RUNNING;
  exit_output right_exit = RUNNING;
  // Same progress backstop as pid_wait()'s DRIVE branch -- this loop's own exits share that gap exactly (see the
  // comment there), and this function has no other protection against, say, a sustained spin.  On an odom move this
  // has to watch l_error/r_error (the real remaining distance to the real target) instead of leftPID/rightPID's own
  // error, for the same reason without_position_exits() strips SMALL_EXIT/BIG_EXIT out below: leftPID/rightPID's
  // error is measured against their frozen look-ahead target, not against `target`, so a healthy drive that has
  // simply driven past that near point reads to it as permanent non-progress and would be falsely flagged stuck.
  // Moved-since-motion-start is judged against l_start/r_start (this motion's own real start), not this
  // particular wait_until() call -- same reasoning as pid_wait()'s DRIVE branch above.
  SingleStuckWatch left_watch(leftPID, is_odom ? l_error : leftPID.error, std::fabs(drive_sensor_left() - l_start) > stuck_step(leftPID, STUCK_STEP_DISTANCE_CAP), STUCK_STEP_DISTANCE_CAP),
      right_watch(rightPID, is_odom ? r_error : rightPID.error, std::fabs(drive_sensor_right() - r_start) > stuck_step(rightPID, STUCK_STEP_DISTANCE_CAP), STUCK_STEP_DISTANCE_CAP);

  // Whether this wait_until()'s own target IS (not just near) the motion's actual final target, not
  // some earlier waypoint the robot is meant to drive through. pid_wait()'s DRIVE branch already
  // treats "the no-progress watch fired, but every side that's still RUNNING is already sitting
  // inside its own big-error window" as settled, not stuck -- correct for a real final target,
  // where stopping an inch short really is a normal settle, but that exemption was deliberately
  // scoped away from this function, which is right for a genuine intermediate waypoint (stopping
  // short there really should report interfered=true) but wrong for the common case where this
  // wait_until()'s target IS the final target. Only DRIVE's own target already tracks the real
  // motion (see is_odom's own comment above) -- an odom move's leftPID/rightPID target is a fixed
  // look-ahead point, never the real final target, so this is unconditionally false for odom.
  // Derived from left_target/right_target above (the same pre-delay snapshot the retarget guard
  // uses), not a fresh target_get() call here, so this reflects this call's own original motion
  // even if a retarget already landed in the delay above -- though the retarget guard's first pass
  // returns before this value is ever consulted in that case anyway.
  bool at_final_target = !is_odom && std::fabs(l_tar - left_target) < FINAL_TARGET_TOLERANCE && std::fabs(r_tar - right_target) < FINAL_TARGET_TOLERANCE;

  while (true) {
    if (mode != mode_snapshot) {
      if (print_toggle) std::cout << "  Drive: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
      interfered_scope.mark();
      return;
    } else if (odom_mode) {
      if (odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
        if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
        interfered_scope.mark();
        return;
      }
    } else if (leftPID.target_get() != left_target || rightPID.target_get() != right_target) {
      if (print_toggle) std::cout << "  Drive: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
      interfered_scope.mark();
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
        // A slow (not stalled) cruise must not be ended by the velocity channel alone -- same
        // reasoning, and the same without_velocity() treatment, as every other odom xy exit check
        // in this file (pid_wait()'s odom branch, pid_wait_until_point(),
        // pid_wait_until_index_started()). xy_velocity_exit_hold_update() above only protects a
        // turn-bias pivot up to its own fallback; a genuinely slow, healthy, straight cruise still
        // needs this filter. SingleStuckWatch (via l_error/r_error below) remains the real stall
        // backstop.
        exit_output xy_exit = without_velocity(xyPID.exit_condition(both_sides(left_motors, right_motors)));
        if (xy_exit != RUNNING) {
          if (print_toggle) std::cout << "  XY: " << exit_to_string(xy_exit) << " Wait Until Exit Failsafe, the move ended before reaching " << target << "\n";
          if (xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT) interfered_scope.mark();
          return;
        }
      }

      if (left_exit == RUNNING || right_exit == RUNNING) {
        secondary_velocity_sensor_update(leftPID);
        secondary_velocity_sensor_update(rightPID);
        // Non-odom (plain DRIVE): a slow (not stalled) cruise must not be ended by the velocity channel
        // alone -- same reasoning as pid_wait()'s DRIVE branch. Odom: without_position_exits() strips
        // SMALL_EXIT/BIG_EXIT on purpose (leftPID/rightPID's target here is only a frozen look-ahead
        // point, not the real final target) but a stalled motor's velocity reading is real regardless of
        // which target produced the error -- except the same fixed 5in/s velocity floor that's too fast
        // for a genuinely slow, healthy cruise everywhere else in this file is exactly as blind to gearing
        // here, so it still needs without_velocity() on top, same as every other site; mA_EXIT is left
        // through unfiltered, since over-current is real regardless of target.
        if (left_exit == RUNNING) left_exit = without_velocity(is_odom ? without_position_exits(leftPID.exit_condition(left_motors)) : leftPID.exit_condition(left_motors));
        if (right_exit == RUNNING) right_exit = without_velocity(is_odom ? without_position_exits(rightPID.exit_condition(right_motors)) : rightPID.exit_condition(right_motors));
        bool left_stuck = left_exit == RUNNING && left_watch.stuck(is_odom ? l_error : leftPID.error);
        bool right_stuck = right_exit == RUNNING && right_watch.stuck(is_odom ? r_error : rightPID.error);
        // See the matching comment in pid_wait()'s DRIVE branch -- both sides exiting normally on the same pass
        // must not read as stuck.
        if ((left_exit == RUNNING || right_exit == RUNNING) && (left_exit != RUNNING || left_stuck) && (right_exit != RUNNING || right_stuck)) {
          // Same settled carve-out as pid_wait()'s DRIVE branch, gated to only apply when this
          // wait_until()'s target really is the motion's final target -- see at_final_target's comment.
          bool stalled = true;
          if (at_final_target) {
            // A side that already latched an exit is only trusted as settled here if its live error
            // is still inside the window that exit means -- a side that latched early and has since
            // been shoved or pinned off target must not count as settled just because it once exited
            // cleanly (same rule as the recheck in the `else` branch below, applied here to the
            // stuck-detected path instead).
            bool left_settled = std::fabs(leftPID.error) < leftPID.exit.big_error;
            bool right_settled = std::fabs(rightPID.error) < rightPID.exit.big_error;
            stalled = !(left_settled && right_settled);
          }
          if (print_toggle) std::cout << "  Drive: " << (stalled ? "Stuck" : "Stuck, but stopped inside the big error windows, counted as settled") << " Wait Until Exit Failsafe, triggered at " << drive_sensor_left() - l_start << " instead of " << target << "\n";
          if (stalled) interfered_scope.mark();
          return;
        }
        // No delay here -- the loop's own delay at the bottom already advances one DELAY_TIME per pass.  A second
        // delay here made every failsafe in this function take about twice its configured time in real time.
      } else {
        // Both sides have already latched a non-RUNNING exit on an earlier pass. Right before
        // trusting that as a clean return, recheck each latched side against the window it exited
        // through, using its own live error -- not exit_condition() (that would restart its
        // internal timers) -- the same treatment pid_wait()'s odom branch already gives a clean
        // double-exit. A side that has drifted back outside its own window since latching is
        // un-latched (back to RUNNING), falling through to keep waiting instead of trusting a stale
        // result. Its stuck watch is left un-reseeded here: it already stopped being fed the moment
        // this side first latched, so its own clock keeps reading time elapsed since well before it
        // ever latched, not since this un-latch.
        //
        // This has the identical shape as issue #532 (a side that finishes early, sits idle while
        // its sibling keeps running, then is un-latched here by a fresh disturbance can be measured
        // against a stale clock and read stuck instantly, with no real grace period) -- pid_wait()'s
        // DRIVE branch above was fixed for exactly this by reseeding its watch on un-latch, up to
        // STUCK_WATCH_REARM_CAP times per side per wait; past that, it falls back to exactly this
        // un-reseeded behavior for the rest of the wait too (unconditional reseeding measurably hung
        // a side oscillating right at its own window's edge -- see PR #543's own discussion for the
        // measured repro, not just a theoretical concern). That fix (cap included) was scoped to
        // pid_wait() only; this wait_until_drive() call site has the same gap, left as-is here.
        // VELOCITY_EXIT is never latched here (without_velocity() already maps it to RUNNING);
        // mA_EXIT isn't a window exit and is handled below regardless, so it's left alone.
        if (left_exit == SMALL_EXIT && std::fabs(leftPID.error) >= leftPID.exit.small_error) {
          left_exit = RUNNING;
        } else if (left_exit == BIG_EXIT && std::fabs(leftPID.error) >= leftPID.exit.big_error) {
          left_exit = RUNNING;
        }
        if (right_exit == SMALL_EXIT && std::fabs(rightPID.error) >= rightPID.exit.small_error) {
          right_exit = RUNNING;
        } else if (right_exit == BIG_EXIT && std::fabs(rightPID.error) >= rightPID.exit.big_error) {
          right_exit = RUNNING;
        }

        if (left_exit == RUNNING || right_exit == RUNNING) {
          // Un-latched -- fall through to the shared delay below and keep waiting instead of
          // returning on a stale result.
        } else {
          if (print_toggle) {
            std::cout << "  Left: " << exit_to_string(left_exit) << " Wait Until Exit Failsafe, triggered at " << drive_sensor_left() - l_start << " instead of " << target << "\n";
            std::cout << "  Right: " << exit_to_string(right_exit) << " Wait Until Exit Failsafe, triggered at " << drive_sensor_right() - r_start << " instead of " << target << "\n";
          }
          // A clean double window-exit (SMALL_EXIT/BIG_EXIT) only ends this wait_until() without
          // interfered=true when its own target really is the motion's final target -- see
          // at_final_target's comment and WAIT_BEHAVIOR_SPEC.md's settled-exemption entry. A
          // checkpoint short of the final target that the robot stopped short of past this point is
          // a real early exit, not a settle.
          bool stalled = !at_final_target;
          if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
            stalled = true;
          }
          if (stalled) interfered_scope.mark();
          return;
        }
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
  // The direction this checkpoint is expected to close from, taken from the requested target
  // relative to the motion's own real start (chain_sensor_start, set once when the turn/swing
  // itself was set -- see turn_set_internal()/swing_set_internal()) rather than from the live
  // g_error above: g_error still needs a live read (it's the actual crossing check, re-read every
  // pass below), but seeding its EXPECTED starting sign from a live read taken here, at the top of
  // this function, means an ordinary caller-side delay between pid_turn_set()/pid_swing_set() and
  // pid_wait_until() -- no shove or concurrent retarget needed -- can already have carried the
  // heading past a short checkpoint by the time this reads it, latching the "already past" sign as
  // the expected one and leaving the crossing check below waiting for a flip that may never come,
  // since the motion only keeps moving further from the checkpoint from there. target's own sign
  // relative to where the motion actually started can't go stale this way, exactly the same
  // reasoning wait_until_drive() already uses for its own l_sgn/r_sgn (see its comment) -- and
  // chain_sensor_start is this function's own equivalent of that function's l_start/r_start,
  // already used the same way by used_motion_chain_scale and the mid-loop stuck watches below.
  int g_sgn = util::sgn(target - chain_sensor_start);

  exit_output turn_exit = RUNNING;
  exit_output swing_exit = RUNNING;

  // Same concurrent-retarget guard as pid_wait()'s TURN/SWING branches -- this function had none, unlike
  // every other wait.  A concurrent pid_turn_set()/pid_turn_relative_set()/pid_swing_set() (or any other
  // motion setter) mid-wait retargets turnPID/swingPID and moves mode along with it, so both the specific
  // PID this call cares about and mode itself are snapshotted and checked every pass. Snapshotted here,
  // BEFORE the settle delay below (not after it): a concurrent retarget landing during that delay would
  // otherwise be captured as this call's own baseline instead of being noticed -- the same hazard
  // pid_wait_until_index_started() guards against for its own settle delay (see its comment).
  e_mode mode_snapshot = mode;
  double turn_target = turnPID.target_get();
  double swing_target = swingPID.target_get();
  // See pid_wait()'s matching comment on InterferedScope -- opened here, alongside the snapshots above and
  // before this function's own leading settle delay, for the same reason.
  InterferedScope interfered_scope(*this);

  // Whether this wait_until()'s own target IS (not just near) the motion's actual final target --
  // same reasoning as wait_until_drive()'s at_final_target (see its comment). Taken from turn_target/
  // swing_target above (the same pre-delay snapshot the retarget guard uses), not a fresh target_get()
  // call here, so this reflects this call's own original motion even if a retarget already landed in
  // the delay below -- though the retarget guard returns before this value is ever consulted in that
  // case anyway.
  //
  // TURN_TO_POINT needs its own rule rather than a plain numeric comparison against turnPID's own
  // static target: turn_pid_task() recomputes TURN_TO_POINT's live error every pass from the point
  // actually being faced, so turnPID's target isn't its real aim point the way it is for a plain TURN
  // -- but turn_set_internal() (called by both pid_turn_set() and pid_turn_set(pose), including for
  // TURN_TO_POINT) writes the SAME value to turnPID's static target and to chain_target_start at
  // motion start, so a wait_until() call chained onto the motion's own target (chain_target_start, as
  // pid_wait_quick()/pid_wait_quick_chain() pass) numerically matches turn_target for TURN_TO_POINT
  // too -- while an explicit checkpoint short of the real aim (a genuine intermediate target) still
  // numerically differs from it, exactly the distinction this gate exists to draw. So TURN_TO_POINT is
  // never excluded outright (that would report interfered=true on every ordinary turn-to-point settle,
  // chained or not, whether detected via the no-progress watch or via a clean latch) -- both paths use
  // this same numeric-plus-chain rule.
  //
  // The numeric comparison alone still can't tell a CHAINED turn-to-point call apart, though: a plain
  // TURN's own target really is bumped by used_motion_chain_scale when chained
  // (pid_wait_quick_chain(), a few hundred lines below), so target(==chain_target_start, unbumped)
  // and turn_target(bumped) numerically differ there already, correctly losing the exemption -- but
  // turn_pid_task() adds used_motion_chain_scale to TURN_TO_POINT's live error directly instead of
  // ever bumping turnPID's own target (see its own comment, a few hundred lines below), so
  // chain_target_start and turn_target stay numerically equal for a chained turn-to-point too. A
  // chained wait is supposed to get no settled exemption at all, matching every other chained wait in
  // this codebase, so TURN_TO_POINT additionally requires nothing having chained onto this motion.
  //
  // Used identically by both places in the TURN branch below that decide whether settling inside
  // big_error counts as an ordinary settle: the no-progress watch firing while still RUNNING, and a
  // clean SMALL_EXIT/BIG_EXIT latch rechecked on entry to the already-latched branch. There's no
  // reason for those two paths to disagree on what counts as "this call's target is the motion's own
  // final aim".
  bool turn_at_final_target = std::fabs(target - turn_target) < FINAL_TARGET_TOLERANCE && (mode != TURN_TO_POINT || used_motion_chain_scale == 0.0);
  bool swing_at_final_target = std::fabs(target - swing_target) < FINAL_TARGET_TOLERANCE;

  // Let the PID run at least 1 iteration before seeding the progress backstop from real error --
  // matching pid_wait() and wait_until_drive(), both of which delay before constructing their own
  // watch. Without this, a fresh Drive's very first turn or swing seeds SingleStuckWatch from a
  // leftover/zero error instead of a real, computed one; the jump from that artifact to the real
  // error then consumes Channel's one-shot rebound allowance (see Channel's own comment above) on
  // an artifact instead of a real disturbance, so the wait's first genuine disturbance can get
  // treated as a second one and false-stuck. g_error above is only read once here to have some
  // value before the loop below overwrites it fresh every pass with the real "have we crossed the
  // target" check; g_sgn (see its own comment above) no longer comes from a live read at all, so
  // unlike before that fix, moving this delay earlier or later can't change which starting sign it
  // latches.
  pros::delay(util::DELAY_TIME);

  // Same JC-1 progress backstop as pid_wait()'s TURN/SWING branches -- see the comment there. Moved-since-
  // motion-start is judged against chain_sensor_start (this motion's own real start), not this particular
  // wait_until() call, the same as those two branches.
  bool already_moved = std::fabs(drive_angle_get() - chain_sensor_start) > std::max(stuck_step(turnPID, STUCK_STEP_ANGLE_CAP), stuck_step(swingPID, STUCK_STEP_ANGLE_CAP));
  SingleStuckWatch turn_watch(turnPID, turnPID.error, already_moved, STUCK_STEP_ANGLE_CAP);
  SingleStuckWatch swing_watch(swingPID, swingPID.error, already_moved, STUCK_STEP_ANGLE_CAP);

  while (true) {
    if (mode != mode_snapshot || turnPID.target_get() != turn_target || swingPID.target_get() != swing_target) {
      if (print_toggle) std::cout << "  Turn/Swing: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
      interfered_scope.mark();
      return;
    }

    g_error = target - drive_angle_get();

    // If turning...
    if (mode == TURN || mode == TURN_TO_POINT) {
      // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
      if (util::sgn(g_error) == g_sgn) {
        if (turn_exit == RUNNING) {
          secondary_velocity_sensor_update(turnPID);
          // See the matching comment in pid_wait()'s DRIVE branch -- a slow (not stalled) turn must not
          // be ended by the velocity channel alone.
          turn_exit = turn_exit != RUNNING ? turn_exit : without_velocity(turnPID.exit_condition(both_sides(left_motors, right_motors)));
          if (turn_exit == RUNNING && turn_watch.stuck(turnPID.error)) {
            // Same settled carve-out as pid_wait()'s TURN branch, gated to only apply when this
            // wait_until()'s target really is the motion's final target -- see turn_at_final_target's
            // comment above.
            bool stalled = !turn_at_final_target || !(std::fabs(turnPID.error) < turnPID.exit.big_error);
            if (print_toggle) std::cout << "  Turn: " << (stalled ? "Stuck" : "Stuck, but stopped inside the big error window, counted as settled") << " Wait Until Exit Failsafe, triggered at " << drive_angle_get() << " instead of " << target << "\n";
            if (stalled) interfered_scope.mark();
            return;
          }
          // No delay here -- see the matching comment in wait_until_drive().
        } else {
          // turn_exit already latched a non-RUNNING exit on an earlier pass. Right before trusting
          // that as a clean return, recheck it against the window it exited through, using its own
          // live error -- same treatment as wait_until_drive()'s own else branch (see its comment,
          // including why its stuck watch is deliberately NOT reseeded here either). A latch that
          // has drifted back outside its own window since exiting is un-latched (back to RUNNING),
          // falling through to keep waiting instead of trusting a stale result.
          if (turn_exit == SMALL_EXIT && std::fabs(turnPID.error) >= turnPID.exit.small_error) {
            turn_exit = RUNNING;
          } else if (turn_exit == BIG_EXIT && std::fabs(turnPID.error) >= turnPID.exit.big_error) {
            turn_exit = RUNNING;
          }

          if (turn_exit != RUNNING) {
            if (print_toggle) std::cout << "  Turn: " << exit_to_string(turn_exit) << " Wait Until Exit Failsafe, triggered at " << drive_angle_get() << " instead of " << target << "\n";

            // Same settled-exemption gating as wait_until_drive()'s else branch, and the same
            // turn_at_final_target used by the no-progress watch path above -- a clean SMALL_EXIT/
            // BIG_EXIT latch only ends this without interfered=true when this wait_until()'s own
            // target really is the motion's final target -- see turn_at_final_target's own comment
            // for TURN_TO_POINT's rule.
            bool stalled = !turn_at_final_target;
            if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) stalled = true;
            if (stalled) interfered_scope.mark();
            return;
          }
          // Un-latched -- fall through to the shared delay below and keep waiting.
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
          // See the matching comment in pid_wait()'s DRIVE branch -- a slow (not stalled) swing must not
          // be ended by the velocity channel alone. Polls both sides' motors, not just the actively-swinging
          // side's -- see pid_wait()'s SWING branch for why the held side needs checking too.
          swing_exit = swing_exit != RUNNING ? swing_exit : without_velocity(swingPID.exit_condition(both_sides(left_motors, right_motors)));
          if (swing_exit == RUNNING && swing_watch.stuck(swingPID.error)) {
            // Same settled carve-out as pid_wait()'s SWING branch, gated to only apply when this
            // wait_until()'s target really is the motion's final target -- see swing_at_final_target's
            // comment above.
            bool stalled = !swing_at_final_target || !(std::fabs(swingPID.error) < swingPID.exit.big_error);
            if (print_toggle) std::cout << "  Swing: " << (stalled ? "Stuck" : "Stuck, but stopped inside the big error window, counted as settled") << " Wait Until Exit Failsafe, triggered at " << drive_angle_get() << " instead of " << target << "\n";
            if (stalled) interfered_scope.mark();
            return;
          }
          // No delay here -- see the matching comment in wait_until_drive().
        } else {
          // Same recheck as the TURN branch above -- see its comment (including why the stuck
          // watch is deliberately not reseeded).
          if (swing_exit == SMALL_EXIT && std::fabs(swingPID.error) >= swingPID.exit.small_error) {
            swing_exit = RUNNING;
          } else if (swing_exit == BIG_EXIT && std::fabs(swingPID.error) >= swingPID.exit.big_error) {
            swing_exit = RUNNING;
          }

          if (swing_exit != RUNNING) {
            if (print_toggle) std::cout << "  Swing: " << exit_to_string(swing_exit) << " Wait Until Exit Failsafe, triggered at " << drive_angle_get() << " instead of " << target << "\n";

            bool stalled = !swing_at_final_target;
            if (swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) stalled = true;
            if (stalled) interfered_scope.mark();
            return;
          }
          // Un-latched -- fall through to the shared delay below and keep waiting.
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
  // Same concurrent-retarget guard as pid_wait()'s odom branch -- this function had none, unlike every
  // other wait_until_*.  odom_target_start is only touched by a top-level odom setter starting a genuinely
  // new motion (see the comment on pid_wait()'s odom branch), so watching it catches a retarget regardless
  // of what it lands in.  mode is watched too, for a concurrent setter from a non-odom mode.
  //
  // Snapshotted here, BEFORE the settle delay below (not after it): a concurrent motion setter can
  // retarget the drive during this call's own first pros::delay(10), before anything else has taken a
  // baseline to compare against -- the same hazard wait_until_turn_swing_internal() and
  // pid_wait_until_index_started() already guard against for their own leading delays (see their
  // comments). Checking against a pre-delay snapshot right after the delay, below, catches that as a
  // retarget instead of silently treating the new motion as this call's own.
  e_mode mode_snapshot = mode;
  pose retarget_target = odom_target_start;
  // See pid_wait()'s matching comment on InterferedScope -- opened here, alongside the snapshot above and
  // before this call's own leading settle delay, for the same reason.
  InterferedScope interfered_scope(*this);

  pros::delay(10);

  if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
    if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
    interfered_scope.mark();
    return;
  }

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

  // Whether pure pursuit is still before its last point right now -- see pid_wait()'s own matching comment
  // (on the pre-last-point loop in its odom branch) for why xy's window exits mean nothing there: before the
  // last point, xyPID's target is only ever the moving look-ahead point, not `target` (this call's own,
  // real, non-moving target). On the last point, or in POINT_TO_POINT, xyPID's target IS the real target, so
  // today's clean-latch failsafe below is a real one and stays. Locked, matching how pid_wait()'s own
  // target_distance lambda reads pp_movements -- a motion started from another task can replace it while
  // this reads it.
  auto before_last_point = [&]() {
    ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
    return mode == PURE_PURSUIT && pp_index != (int)pp_movements.size() - 1;
  };

  while (true) {
    if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
      if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong target.\n";
      interfered_scope.mark();
      return;
    }
    secondary_velocity_sensor_update(xyPID);
    secondary_velocity_sensor_update(current_a_odomPID);
    xy_velocity_exit_hold_update();
    // Before the last point, xyPID's target is a moving look-ahead point (see before_last_point()'s own
    // comment): SMALL_EXIT/BIG_EXIT/VELOCITY_EXIT on it are discarded on purpose, only mA_EXIT (a stalled
    // motor's current is real regardless of target) can end the wait through this axis. The mA snapshot/
    // restore mirrors pid_wait()'s own pre-last-point loop exactly, for the identical reason (GitHub issue
    // #527): PID::exit_condition() calls PID::timers_reset() whenever ANY channel latches, wiping every
    // channel's timer together -- including this same call's own mA progress -- so a discarded SMALL_EXIT/
    // BIG_EXIT would silently erase real, ongoing over-current progress before it ever reaches mA_timeout.
    if (xy_exit == RUNNING) {
      bool xy_before_last_point = before_last_point();
      std::vector<pros::Motor> xy_motors = both_sides(left_motors, right_motors);
      bool xy_mA_tracked = xyPID.exit.mA_timeout != 0;
      bool xy_over_current = false;
      if (xy_before_last_point && xy_mA_tracked) {
        for (auto& motor : xy_motors) {
          std::int32_t xy_over = motor.is_over_current();
          bool xy_dead = xy_over == PROS_ERR && !std::isfinite(motor.get_position());
          if (xy_over == 1 || xy_dead) {
            xy_over_current = true;
            break;
          }
        }
      }
      PID::MATimerSnapshot xy_mA_snapshot = xyPID.mA_timer_snapshot();
      exit_output xy_pass = xyPID.exit_condition(xy_motors);
      if (xy_before_last_point) {
        if (xy_mA_tracked && xy_over_current && (xy_pass == SMALL_EXIT || xy_pass == BIG_EXIT || xy_pass == VELOCITY_EXIT)) xyPID.mA_timer_restore_and_credit(xy_mA_snapshot);
        if (xy_pass == mA_EXIT) xy_exit = mA_EXIT;
      } else {
        xy_exit = without_velocity(xy_pass);
      }
    }
    a_exit = a_exit != RUNNING ? a_exit : without_velocity(current_a_odomPID.exit_condition(both_sides(left_motors, right_motors)));

    // Same stuck check as pid_wait(), for a robot that is stuck but moving, which the exits above miss
    if (watch.stuck(pp_index, util::distance_to_point(target, odom_pose_get()), xyPID.error, current_a_odomPID.error, util::distance_to_point(odom_start, odom_pose_get()), std::fabs(odom_theta_get() - odom_start.theta))) {
      if (print_toggle) std::cout << "  Stuck before reaching (" << target.x << ", " << target.y << "), at (" << odom_x_get() << ", " << odom_y_get() << ")\n";
      interfered_scope.mark();
      return;
    }

    if (xy_exit != RUNNING && a_exit != RUNNING) {
      // Once an axis latches SMALL_EXIT/BIG_EXIT, exit_condition() above is never called on it again --
      // so unlike pid_wait()'s odom branch (see the comment on its own recheck, added for the identical
      // bug), a disturbance landing on an already-latched axis (most commonly angle, which typically
      // settles first) went completely unwatched for the rest of this wait. Recheck each latched axis
      // against the window it exited through, using its own live error -- not exit_condition() (that
      // would restart its internal timers). VELOCITY_EXIT is never latched here (without_velocity()
      // already maps it to RUNNING); mA_EXIT/ERROR_NO_CONSTANTS aren't window exits and are handled by
      // the interfered check below regardless, so they're left alone. A latched axis that has drifted
      // back outside its window is un-latched, falling through to keep waiting -- the stuck check above
      // remains the backstop if the disturbance never resolves.
      if (xy_exit == SMALL_EXIT && std::fabs(xyPID.error) >= xyPID.exit.small_error) xy_exit = RUNNING;
      else if (xy_exit == BIG_EXIT && std::fabs(xyPID.error) >= xyPID.exit.big_error) xy_exit = RUNNING;
      if (a_exit == SMALL_EXIT && std::fabs(current_a_odomPID.error) >= current_a_odomPID.exit.small_error) a_exit = RUNNING;
      else if (a_exit == BIG_EXIT && std::fabs(current_a_odomPID.error) >= current_a_odomPID.exit.big_error) a_exit = RUNNING;
    }

    if (xy_exit != RUNNING && a_exit != RUNNING) {
      if (print_toggle) {
        std::cout << "  XY: " << exit_to_string(xy_exit) << " Wait Until Exit Failsafe, triggered at (" << odom_x_get() << ", " << odom_y_get() << ") instead of (" << target.x << ", " << target.y << ")\n";
        xyPID.timers_reset();
        current_a_odomPID.timers_reset();
      }
      if (xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT || a_exit == mA_EXIT || a_exit == VELOCITY_EXIT) {
        interfered_scope.mark();
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
  // Snapshotted before the settle delay below, not after: a concurrent motion setter can retarget
  // the drive during this call's own first pros::delay(), before anything else has taken a baseline
  // to compare against. Checking against a pre-delay snapshot catches that as a retarget instead of
  // either silently treating the new motion as this call's own, or (since a cross-mode retarget away
  // from PURE_PURSUIT would otherwise reach the mode check below first) misreporting it as this call
  // never having been started in pure pursuit to begin with.
  e_mode mode_snapshot = mode;
  pose retarget_target = odom_target_start;
  // See pid_wait()'s matching comment on InterferedScope -- opened here, alongside the snapshot above and
  // before this call's own leading settle delay, for the same reason.
  InterferedScope interfered_scope(*this);

  // Let the PID run at least 1 iteration
  pros::delay(util::DELAY_TIME);

  if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
    if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of continuing on the wrong path.\n";
    interfered_scope.mark();
    return;
  }

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
  // The failsafe print below (triggered_at instead of pp_movements[...].target) used to read
  // pp_movements unlocked, after this snapshot but outside any lock -- a concurrent pid_odom_*_set()
  // replacing pp_movements/injected_pp_index between the snapshot above and that print could leave it
  // indexing into a pp_movements that's now shorter than injected_pp_index_snapshot expects. Taken
  // in the SAME lock acquisition as injected_pp_index_snapshot below so the two are always a
  // consistent pair -- injected_pp_index and pp_movements are only ever published together (see
  // set_odom_pid.cpp / purepursuit_math.cpp), so a snapshot of one taken alongside the other under
  // one lock can't straddle a publish the way two separate lock acquisitions could.
  std::vector<int> injected_pp_index_snapshot;
  pose failsafe_target{};
  {
    ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
    injected_pp_index_snapshot = injected_pp_index;
    if (index >= 0 && index <= (int)injected_pp_index_snapshot.size() - 2) {
      failsafe_target = pp_movements[injected_pp_index_snapshot[index + 1]].target;
    }
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

  // Same concurrent-retarget guard as pid_wait()'s odom branch -- this function had none, unlike every
  // other public wait in this file. A concurrent pid_odom_*_set() from another task resets pp_index to 0
  // and replaces injected_pp_index/pp_movements/odom_target_start with the new motion's, so without this
  // check the while condition below (pp_index < injected_pp_index_snapshot[index], a threshold taken from
  // the OLD motion) would just keep polling whatever path is live now and report a clean, uninterfered
  // success once the NEW path's pp_index happens to climb back past that same number -- not the motion
  // this call was actually started for. mode is watched too, for a concurrent setter from a non-PP mode.
  // (mode_snapshot/retarget_target were already snapshotted before this call's own settle delay above.)
  while (pp_index < injected_pp_index_snapshot[index]) {
    if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
      if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of continuing on the wrong path.\n";
      interfered_scope.mark();
      break;
    }
    secondary_velocity_sensor_update(xyPID);
    secondary_velocity_sensor_update(current_a_odomPID);
    xy_velocity_exit_hold_update();
    // This whole loop runs only before its own checkpoint (the while condition above), so xyPID's target is
    // always the moving look-ahead point here, never the real path -- the same reasoning as pid_wait()'s own
    // pre-last-point loop (see its comment). SMALL_EXIT/BIG_EXIT/VELOCITY_EXIT on it are discarded on
    // purpose; only mA_EXIT (a stalled motor's current is real regardless of target) can end the wait
    // through this axis. The mA snapshot/restore mirrors pid_wait()'s own pre-last-point loop exactly, for
    // the identical reason (GitHub issue #527): PID::exit_condition() calls PID::timers_reset() whenever ANY
    // channel latches, wiping every channel's timer together -- including this same call's own mA progress --
    // so a discarded SMALL_EXIT/BIG_EXIT would silently erase real, ongoing over-current progress before it
    // ever reaches mA_timeout.
    if (xy_exit == RUNNING) {
      std::vector<pros::Motor> xy_motors = both_sides(left_motors, right_motors);
      bool xy_mA_tracked = xyPID.exit.mA_timeout != 0;
      bool xy_over_current = false;
      if (xy_mA_tracked) {
        for (auto& motor : xy_motors) {
          std::int32_t xy_over = motor.is_over_current();
          bool xy_dead = xy_over == PROS_ERR && !std::isfinite(motor.get_position());
          if (xy_over == 1 || xy_dead) {
            xy_over_current = true;
            break;
          }
        }
      }
      PID::MATimerSnapshot xy_mA_snapshot = xyPID.mA_timer_snapshot();
      exit_output xy_pass = xyPID.exit_condition(xy_motors);
      if (xy_mA_tracked && xy_over_current && (xy_pass == SMALL_EXIT || xy_pass == BIG_EXIT || xy_pass == VELOCITY_EXIT)) xyPID.mA_timer_restore_and_credit(xy_mA_snapshot);
      if (xy_pass == mA_EXIT) xy_exit = mA_EXIT;
    }
    a_exit = a_exit != RUNNING ? a_exit : without_velocity(current_a_odomPID.exit_condition(both_sides(left_motors, right_motors)));

    // Same stuck check as pid_wait(), for a robot that is stuck but moving, which the exits above miss
    if (watch.stuck(pp_index, point_distance(), xyPID.error, current_a_odomPID.error, util::distance_to_point(odom_start, odom_pose_get()), std::fabs(odom_theta_get() - odom_start.theta))) {
      if (print_toggle) std::cout << "  Stuck before reaching point " << injected_pp_index_snapshot[index] << ", at (" << odom_x_get() << ", " << odom_y_get() << ")\n";
      interfered_scope.mark();
      break;
    }

    if (xy_exit != RUNNING && a_exit != RUNNING) {
      // xy_exit can only be RUNNING or mA_EXIT here (see above) -- there's nothing to recheck on that axis
      // in this loop. Angle can still latch SMALL_EXIT/BIG_EXIT on its own moving-look-ahead-independent
      // target, so it keeps the same recheck pid_wait_until_point() and pid_wait()'s odom branch give a
      // clean double-exit (see their own comments on this identical bug): against the window it exited
      // through, using its own live error -- not exit_condition() (that would restart its internal timers).
      // VELOCITY_EXIT is never latched here (without_velocity() already maps it to RUNNING). A latched axis
      // that has drifted back outside its window is un-latched (back to RUNNING), falling through to keep
      // waiting -- the stuck check above remains the backstop if the disturbance never resolves.
      if (a_exit == SMALL_EXIT && std::fabs(current_a_odomPID.error) >= current_a_odomPID.exit.small_error) a_exit = RUNNING;
      else if (a_exit == BIG_EXIT && std::fabs(current_a_odomPID.error) >= current_a_odomPID.exit.big_error) a_exit = RUNNING;
    }

    if (xy_exit != RUNNING && a_exit != RUNNING) {
      if (print_toggle) {
        // failsafe_target was snapshotted alongside injected_pp_index_snapshot above, under the same
        // lock -- see that snapshot's comment for why reading pp_movements directly here, unlocked,
        // is not safe.
        std::cout << "  XY: " << exit_to_string(xy_exit) << " Wait Until Exit Failsafe, triggered at (" << odom_x_get() << ", " << odom_y_get() << ") instead of (" << failsafe_target.x << ", " << failsafe_target.y << ")\n";
        xyPID.timers_reset();
        current_a_odomPID.timers_reset();
      }
      if (xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT || a_exit == mA_EXIT || a_exit == VELOCITY_EXIT) {
        interfered_scope.mark();
      }
      break;
    }

    pros::delay(util::DELAY_TIME);
  }

  // The loop's own condition above (pp_index < injected_pp_index_snapshot[index]) is read fresh --
  // live, unsnapshotted pp_index -- every time control returns from that loop's own trailing
  // pros::delay(), including the pass right after a concurrent retarget lands during that exact
  // delay. If the new motion's own path-following task has already advanced pp_index up to or past
  // the OLD motion's threshold by the time this wait is rescheduled, the condition goes false and
  // the loop exits without its own retarget guard above ever running again for that pass -- falling
  // straight through to here instead of noticing the retarget. Re-running the exact same check right
  // after the loop, regardless of why it exited, catches that case too: harmless (already true) on a
  // path where the guard above already caught the retarget and broke out, a no-op on a genuinely
  // clean, un-retargeted finish, and the fix for the case this comment describes.
  if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
    if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of continuing on the wrong path.\n";
    interfered_scope.mark();
  }
}

void Drive::pid_wait_until_index(int index) {
  // Same concurrent-retarget guard as pid_wait_until_index_started() above, snapshotted before that call
  // (before even its own settle delay) so a retarget noticed during it, or one that lands in the gap
  // between it returning and this function reading pp_movements/injected_pp_index below, is caught either
  // way. Without this, a stale call would go on to read pp_movements/injected_pp_index for whatever motion
  // is current now -- not the one this call was started for -- and hand back a clean, silent success for
  // the wrong path.
  e_mode mode_snapshot = mode;
  pose retarget_target = odom_target_start;
  // See pid_wait()'s matching comment on InterferedScope -- opened here, before either phase below, so this
  // function's own two checks against the snapshot above (not just the phases' own internal ones) mark
  // `interfered` under the SAME motion those phases were themselves scoped to, and so a retarget this
  // function's own check catches (in the gap between phases, or after phase 2) supersedes whatever a phase's
  // own scope already asserted.
  InterferedScope interfered_scope(*this);

  pid_wait_until_index_started(index);

  if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
    if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of continuing on the wrong path.\n";
    interfered_scope.mark();
    return;
  }

  index += 1;
  // target is read from pp_movements in the SAME lock acquisition as injected_pp_index_snapshot below,
  // for the same reason pid_wait_until_index_started()'s failsafe_target is -- see its comment. This
  // one feeds pid_wait_until_point() directly (not just a print), so the race was live, not cosmetic:
  // a concurrent pid_odom_*_set() replacing pp_movements with a shorter path between two separate,
  // unlocked reads could index past its end.
  std::vector<int> injected_pp_index_snapshot;
  pose target{};
  bool have_target = false;
  {
    ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
    injected_pp_index_snapshot = injected_pp_index;
    if (index >= 0 && index < (int)injected_pp_index_snapshot.size()) {
      target = pp_movements[injected_pp_index_snapshot[index]].target;
      have_target = true;
    }
  }
  if (!have_target) return;
  pid_wait_until_point(target);

  // Re-checked against the SAME entry snapshot, after phase 2 too: pid_wait_until_point() now has its
  // own guard against a retarget landing in ITS first settle delay (see the comment on its guard), but
  // that guard's baseline is taken fresh at phase 2's own start -- it cannot see a retarget that lands
  // in the gap between phase 1 returning and phase 2 starting, or during phase 1's own loop after this
  // function's own check above already passed. Comparing against the snapshot taken before phase 1
  // above catches a retarget landing anywhere across the whole call, not just within phase 2's own loop.
  if (mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta) {
    if (print_toggle) std::cout << "  XY: retargeted by a concurrent motion mid-wait, ending early instead of finishing on the wrong path.\n";
    interfered_scope.mark();
  }
}

// Pid wait, but quickly :)
void Drive::pid_wait_quick() {
  if (mode == PURE_PURSUIT) {
    // Same concurrent-retarget guard as pid_wait()'s odom branch (see the comment there) -- unlike
    // pid_wait(), this had no guard on its own headingPID write at all. pid_wait_until_index() above
    // already ends the wait early with interfered=true on a retarget IT notices, but that alone
    // doesn't stop the write below from running on whatever odom_target_start now holds -- and it
    // can't by itself catch a retarget landing in pid_wait_until_index()'s own first settle delay,
    // before its internal guard has taken a baseline to compare against. Snapshotted here, at this
    // call's own entry, before any of that, so it catches a retarget landing anywhere across the
    // whole inner call, not just within its own loop.
    e_mode mode_snapshot = mode;
    pose retarget_target = odom_target_start;
    // See pid_wait()'s matching comment on InterferedScope -- opened here, at this call's own entry, so the
    // headingPID guard below marks `interfered` under the SAME motion this call itself started for,
    // superseding (with the same generation, in the ordinary case) whatever pid_wait_until_index()'s own
    // scope already did.
    InterferedScope interfered_scope(*this);
    int last_index;
    {
      ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
      last_index = (int)injected_pp_index.size() - 2;
    }
    pid_wait_until_index(last_index);
    {
      ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
      // Same as pid_wait(): store the equivalent angle nearest the IMU -- but only if this call's own
      // motion is still the current one. A stale pid_wait_quick() call must not clobber headingPID
      // with the hijacking motion's own in-flight heading, and must report interfered rather than a
      // clean, silent finish for a motion it was never waiting for.
      bool retargeted_since_snapshot = mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta;
      if (retargeted_since_snapshot) {
        interfered_scope.mark();
      } else if (odom_target_start.theta != ANGLE_NOT_SET) {
        headingPID.target_set(new_turn_target_compute(odom_target_start.theta, drive_angle_get(), shortest));
      }
    }
    return;
  } else if (mode == POINT_TO_POINT) {
    // Same concurrent-retarget guard as the PURE_PURSUIT branch above.
    e_mode mode_snapshot = mode;
    pose retarget_target = odom_target_start;
    // See pid_wait()'s matching comment on InterferedScope, and the PURE_PURSUIT branch above -- same reason.
    InterferedScope interfered_scope(*this);
    pid_wait_until_point(odom_target_start);
    {
      ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);
      bool retargeted_since_snapshot = mode != mode_snapshot || odom_target_start.x != retarget_target.x || odom_target_start.y != retarget_target.y || odom_target_start.theta != retarget_target.theta;
      if (retargeted_since_snapshot) {
        interfered_scope.mark();
      } else if (odom_target_start.theta != ANGLE_NOT_SET) {
        headingPID.target_set(new_turn_target_compute(odom_target_start.theta, drive_angle_get(), shortest));
      }
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
