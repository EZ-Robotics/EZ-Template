/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <cmath>

#include "EZ-Template/api.hpp"
#include "EZ-Units/units.hpp"

namespace ez {
// The part of pid_speed_max_set() every internal caller needs: just the clamp and the slew caps,
// no odom path rewrite. Callers already hold drive_mutex (raw_pid_odom_ptp_set(), and the drive/
// turn/swing setters' own re-apply of the motion's own speed at motion start) -- routing them
// through the public pid_speed_max_set() would make its odom-path rewrite (added for mid-motion
// speed changes) run on every point advance using that point's own already-correct speed, and,
// worse, run inside pid_drive_set()/pid_turn_set()/pid_swing_set() while mode still reads
// PURE_PURSUIT from the previous motion, rewriting a path that motion no longer owns.
void Drive::pid_speed_max_set_internal(int speed) {
  max_speed = std::fabs(util::clamp(speed, 127, -127));
  slew_left.speed_max_set(max_speed);
  slew_right.speed_max_set(max_speed);
  slew_turn.speed_max_set(max_speed);
  slew_swing.speed_max_set(max_speed);
}

// Updates max speed. On an odom pure-pursuit/boomerang motion currently running, also rewrites
// the stored speed on every remaining path point (from the one the robot is currently tracking to
// the end), so raw_pid_odom_ptp_set()'s own re-apply on the next point advance (or, for boomerang,
// on next tick's carrot recompute) can't silently restore the pre-call speed a few points later --
// see GitHub issue #536. Unconditional, not "lower only": a later point's own deliberately higher
// (or lower) stored speed is replaced too, matching how this call already behaves mid-pid_drive_set/
// pid_turn_set/pid_swing_set. Rewriting max_xy_speed to exactly what max_speed was just set to also
// means the next point advance's own "did a point ask for a higher speed than what's active" check
// (raw_pid_odom_ptp_set()'s slew_will_enable_later branch) never sees a mismatch here, so a runtime
// call can't be mistaken for the path's own point-to-point speed step and wrongly re-arm that ramp.
// A plain POINT_TO_POINT motion (pid_odom_ptp_set()) never re-applies a stored speed after motion
// start, so it needs no rewrite -- the internal setter's own slew-cap update already covers it.
void Drive::pid_speed_max_set(int speed) {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);

  pid_speed_max_set_internal(speed);

  if (mode == PURE_PURSUIT) {
    for (std::size_t i = pp_index; i < pp_movements.size(); i++) {
      pp_movements[i].max_xy_speed = max_speed;
    }
  }
}
int Drive::pid_speed_max_get() { return max_speed; }

// "turn bias" will bias either left or right, the user can decide
// the shortest path from 0.1 to 180 would be to go 179.9 degrees, but
// PID has some level of variance.  this allows the user to set a tolerance
// that will make the robot go left or right when it's within that tolerance
void Drive::pid_angle_behavior_bias_set(e_angle_behavior behavior) {
  if (behavior == ez::LEFT_TURN)
    turn_biased_left = true;
  else if (behavior == ez::RIGHT_TURN)
    turn_biased_left = false;
  else
    printf("Must input 'left' or 'right' for angle behavior bias!\n");
}
e_angle_behavior Drive::pid_angle_behavior_bias_get() { return turn_biased_left ? ez::LEFT_TURN : ez::RIGHT_TURN; }
void Drive::pid_angle_behavior_tolerance_set(double tolerance) { turn_tolerance = tolerance; }
void Drive::pid_angle_behavior_tolerance_set(ez::QAngle p_tolerance) { pid_angle_behavior_tolerance_set(p_tolerance.convert(ez::degree)); }
double Drive::pid_angle_behavior_tolerance_get() { return turn_tolerance; }

// Changes global default turn behavior to either:
//  - cw
//  - ccw
//  - shortest
//  - longest
//  - raw
void Drive::pid_angle_behavior_set(ez::e_angle_behavior behavior) {
  default_swing_type = behavior;
  default_turn_type = behavior;
  default_odom_type = behavior;
}

void Drive::pid_targets_reset() {
  ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);

  headingPID.target_set(0);
  leftPID.target_set(0);
  rightPID.target_set(0);
  xyPID.target_set(0);
  current_a_odomPID.target_set(0);
  forward_drivePID.target_set(0);
  backward_drivePID.target_set(0);
  turnPID.target_set(0);
  swingPID.target_set(0);
  forward_swingPID.target_set(0);
  backward_swingPID.target_set(0);
}

void Drive::drive_mode_set(e_mode p_mode, bool stop_drive) {
  mode = p_mode;
  if (mode == DISABLE && stop_drive) private_drive_set(0, 0);
}
e_mode Drive::drive_mode_get() { return mode; }

// Toggle drive motors but still allow PID to run
void Drive::pid_drive_toggle(bool toggle) { drive_toggle = toggle; }
bool Drive::pid_drive_toggle_get() { return drive_toggle; }

// Don't print stuff
void Drive::pid_print_toggle(bool toggle) { print_toggle = toggle; }
bool Drive::pid_print_toggle_get() { return print_toggle; }
}  // namespace ez
