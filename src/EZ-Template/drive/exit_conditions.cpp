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
      : xy_(step(xy), distance, xy.error), a_(step(angle), std::fabs(angle.error), angle.error), index_(index), window_(xy.exit.velocity_exit_time != 0 ? xy.exit.velocity_exit_time : xy.exit.mA_timeout), moved_(travelled > xy_.step || turned > a_.step) {
    int allowance = moved_ ? 0 : START_ALLOWANCE_MS;
    last_progress_ = pros::millis() + allowance;
    last_progress_pass_ = passes() + allowance / util::DELAY_TIME;
  }

  // distance: how far the robot is from the point it's driving to.  xy_error and a_error: the PIDs' signed errors.
  bool stuck(int index, double distance, double xy_error, double a_error, double travelled, double turned) {
    if (window_ == 0) return false;
    std::uint32_t now = pros::millis();
    std::uint32_t pass = passes();
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
    return waited > window_ && ((std::int32_t)(pass - last_progress_pass_) > window_ / util::DELAY_TIME || waited > STARVED_WINDOWS * window_);
  }

 private:
  static constexpr int START_ALLOWANCE_MS = 1000;  // PID::VELOCITY_ARM_FALLBACK, which is private
  static constexpr int STARVED_WINDOWS = 4;
  static std::uint32_t passes() { return ez::detail::stats.auto_task_passes.load(std::memory_order_relaxed); }
  static double step(PID& pid) {
    if (pid.exit.small_error > 0) return pid.exit.small_error;
    return pid.velocity_sensor_main_exit_get() * pid.exit.velocity_exit_time / util::DELAY_TIME;
  }
  struct Channel {
    double step, low;
    bool side, rebound = false, rebounded = false;
    Channel(double p_step, double size, double error) : step(p_step), low(size), side(error > 0) {}
    bool made(double size, double error) {
      if ((error > 0) != side && !rebounded) rebound = rebounded = true;
      side = error > 0;
      if (rebound) low = std::fmax(low, size);
      if (size >= low - step) return false;
      low = size;
      rebound = false;
      return true;
    }
  };
  Channel xy_, a_;
  int index_;
  int window_;
  bool moved_;
  std::uint32_t last_progress_, last_progress_pass_;
};

// A velocity exit doesn't end an odom wait: a robot pivoting at a corner, or just slow, reads as stopped to it while
// it's still getting somewhere.  StuckWatch decides stuck instead.  Small, big and current exits end it as always.
exit_output without_velocity(exit_output e) { return e == VELOCITY_EXIT ? RUNNING : e; }
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
    while (left_exit == RUNNING || right_exit == RUNNING) {
      secondary_velocity_sensor_update(leftPID);
      secondary_velocity_sensor_update(rightPID);
      left_exit = left_exit != RUNNING ? left_exit : leftPID.exit_condition(left_motors[0]);
      right_exit = right_exit != RUNNING ? right_exit : rightPID.exit_condition(right_motors[0]);
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle) std::cout << "  Left: " << exit_to_string(left_exit) << " Exit, error: " << leftPID.error << "   Right: " << exit_to_string(right_exit) << " Exit, error: " << rightPID.error << "\n";

    if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
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

    // Wait until pure pursuit is on the last point, then continue as normal.  xy's exit is checked every pass
    // and not kept: before the last point its target is only a look ahead away and keeps moving, so a small,
    // big or velocity exit here says nothing about the path, and one kept from a pause earlier on must not end
    // the wait later.  A current exit still ends it.
    if (mode == PURE_PURSUIT) {
      while (pp_index != (int)pp_movements.size() - 1) {
        secondary_velocity_sensor_update(xyPID);
        secondary_velocity_sensor_update(current_a_odomPID);
        xy_velocity_exit_hold_update();
        exit_output xy_pass = xyPID.exit_condition({left_motors[0], right_motors[0]});
        a_exit = a_exit != RUNNING ? a_exit : without_velocity(current_a_odomPID.exit_condition({left_motors[0], right_motors[0]}));

        if (xy_pass == mA_EXIT || watch.stuck(pp_index, target_distance(), xyPID.error, current_a_odomPID.error, travelled(), turned())) {
          stalled = true;
          if (print_toggle) std::cout << "  XY: " << (xy_pass == mA_EXIT ? exit_to_string(xy_pass) : "Stuck") << " Exited early at point " << pp_index << " of " << (int)pp_movements.size() - 1 << ", error: " << xyPID.error << ".   Angle error: " << current_a_odomPID.error << ".\n";
          break;
        }

        pros::delay(util::DELAY_TIME);
      }
    }

    // When we're at the last point in PP / we're just going to point
    while (!stalled && (xy_exit == RUNNING || a_exit == RUNNING)) {
      secondary_velocity_sensor_update(xyPID);
      secondary_velocity_sensor_update(current_a_odomPID);
      xy_velocity_exit_hold_update();
      xy_exit = xy_exit != RUNNING ? xy_exit : without_velocity(xyPID.exit_condition({left_motors[0], right_motors[0]}));
      a_exit = a_exit != RUNNING ? a_exit : without_velocity(current_a_odomPID.exit_condition({left_motors[0], right_motors[0]}));
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
      if (odom_target_start.theta != ANGLE_NOT_SET) headingPID.target_set(new_turn_target_compute(odom_target_start.theta, drive_angle_get(), shortest));
    }
  }

  // Turn Exit
  else if (mode == TURN || mode == TURN_TO_POINT) {
    exit_output turn_exit = RUNNING;
    while (turn_exit == RUNNING) {
      secondary_velocity_sensor_update(turnPID);
      turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition({left_motors[0], right_motors[0]});
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle) std::cout << "  Turn: " << exit_to_string(turn_exit) << " Exit, error: " << turnPID.error << "\n";

    if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
      interfered = true;
    }
  }

  // Swing Exit
  else if (mode == SWING) {
    exit_output swing_exit = RUNNING;
    pros::Motor& sensor = current_swing == ez::LEFT_SWING ? left_motors[0] : right_motors[0];
    while (swing_exit == RUNNING) {
      secondary_velocity_sensor_update(swingPID);
      swing_exit = swing_exit != RUNNING ? swing_exit : swingPID.exit_condition(sensor);
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle) std::cout << "  Swing: " << exit_to_string(swing_exit) << " Exit, error: " << swingPID.error << "\n";

    if (swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) {
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
  int l_sgn = util::sgn(l_error);
  int r_sgn = util::sgn(r_error);

  exit_output left_exit = RUNNING;
  exit_output right_exit = RUNNING;

  while (true) {
    l_error = l_tar - drive_sensor_left();
    r_error = r_tar - drive_sensor_right();

    // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
    if (util::sgn(l_error) == l_sgn || util::sgn(r_error) == r_sgn) {
      // An odom move ends on its xy exit, which only pid_wait() checks.  The left and right exits below are aimed
      // one look ahead from where the move started, so when the robot drives past that point they can never fire.
      // If the move ends before it reaches this target, return instead of waiting forever.
      bool on_last_point = mode == POINT_TO_POINT || (mode == PURE_PURSUIT && pp_index == (int)pp_movements.size() - 1);
      if (on_last_point) {
        secondary_velocity_sensor_update(xyPID);
        xy_velocity_exit_hold_update();
        exit_output xy_exit = xyPID.exit_condition({left_motors[0], right_motors[0]});
        if (xy_exit != RUNNING) {
          if (print_toggle) std::cout << "  XY: " << exit_to_string(xy_exit) << " Wait Until Exit Failsafe, the move ended before reaching " << target << "\n";
          if (xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT) interfered = true;
          return;
        }
      }

      if (left_exit == RUNNING || right_exit == RUNNING) {
        secondary_velocity_sensor_update(leftPID);
        secondary_velocity_sensor_update(rightPID);
        left_exit = left_exit != RUNNING ? left_exit : leftPID.exit_condition(left_motors[0]);
        right_exit = right_exit != RUNNING ? right_exit : rightPID.exit_condition(right_motors[0]);
        pros::delay(util::DELAY_TIME);
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
    // Once we've past target, return
    else if (util::sgn(l_error) != l_sgn || util::sgn(r_error) != r_sgn) {
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

  pros::Motor& sensor = current_swing == ez::LEFT_SWING ? left_motors[0] : right_motors[0];

  while (true) {
    g_error = target - drive_angle_get();

    // If turning...
    if (mode == TURN || mode == TURN_TO_POINT) {
      // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
      if (util::sgn(g_error) == g_sgn) {
        if (turn_exit == RUNNING) {
          secondary_velocity_sensor_update(turnPID);
          turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition({left_motors[0], right_motors[0]});
          pros::delay(util::DELAY_TIME);
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
          pros::delay(util::DELAY_TIME);
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

  int xy_sgn = util::sgn(is_past_target(target, odom_pose_get()));

  exit_output xy_exit = RUNNING;
  exit_output a_exit = RUNNING;
  StuckWatch watch(xyPID, current_a_odomPID, pp_index, util::distance_to_point(target, odom_pose_get()), util::distance_to_point(odom_start, odom_pose_get()), std::fabs(odom_theta_get() - odom_start.theta));

  while (true) {
    secondary_velocity_sensor_update(xyPID);
    secondary_velocity_sensor_update(current_a_odomPID);
    xy_velocity_exit_hold_update();
    xy_exit = xy_exit != RUNNING ? xy_exit : without_velocity(xyPID.exit_condition({left_motors[0], right_motors[0]}));
    a_exit = a_exit != RUNNING ? a_exit : without_velocity(current_a_odomPID.exit_condition({left_motors[0], right_motors[0]}));

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

  if (index < 0 || index > (int)injected_pp_index.size() - 2) {
    printf("  Wait Until PP Error!  Index %i is not within range!  %i is max!\n", index, (int)injected_pp_index.size() - 2);
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
  while (pp_index < injected_pp_index[index]) {
    secondary_velocity_sensor_update(xyPID);
    secondary_velocity_sensor_update(current_a_odomPID);
    xy_velocity_exit_hold_update();
    xy_exit = xy_exit != RUNNING ? xy_exit : without_velocity(xyPID.exit_condition({left_motors[0], right_motors[0]}));
    a_exit = a_exit != RUNNING ? a_exit : without_velocity(current_a_odomPID.exit_condition({left_motors[0], right_motors[0]}));

    // Same stuck check as pid_wait(), for a robot that is stuck but moving, which the exits above miss
    if (watch.stuck(pp_index, point_distance(), xyPID.error, current_a_odomPID.error, util::distance_to_point(odom_start, odom_pose_get()), std::fabs(odom_theta_get() - odom_start.theta))) {
      if (print_toggle) std::cout << "  Stuck before reaching point " << injected_pp_index[index] << ", at (" << odom_x_get() << ", " << odom_y_get() << ")\n";
      interfered = true;
      break;
    }

    if (xy_exit != RUNNING && a_exit != RUNNING) {
      if (print_toggle) {
        // index points into injected_pp_index, which holds where each waypoint sits in pp_movements
        std::cout << "  XY: " << exit_to_string(xy_exit) << " Wait Until Exit Failsafe, triggered at (" << odom_x_get() << ", " << odom_y_get() << ") instead of (" << pp_movements[injected_pp_index[index]].target.x << ", " << pp_movements[injected_pp_index[index]].target.y << ")\n";
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
  if (index < 0 || index >= (int)injected_pp_index.size()) return;
  pose target = pp_movements[injected_pp_index[index]].target;
  pid_wait_until_point(target);
}

// Pid wait, but quickly :)
void Drive::pid_wait_quick() {
  if (mode == PURE_PURSUIT) {
    pid_wait_until_index(injected_pp_index.size() - 2);
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
