/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"

using namespace ez;

void Drive::pid_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu) {
  leftPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  rightPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  leftPID.velocity_sensor_secondary_toggle_set(use_imu);
  rightPID.velocity_sensor_secondary_toggle_set(use_imu);
  internal_leftPID.exit = leftPID.exit;
  internal_rightPID.exit = rightPID.exit;
}

void Drive::pid_drive_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QLength p_small_error, okapi::QTime p_big_exit_time, okapi::QLength p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu) {
  // Convert okapi units to doubles
  double se = p_small_error.convert(okapi::inch);
  double be = p_big_error.convert(okapi::inch);
  int set = p_small_exit_time.convert(okapi::millisecond);
  int bet = p_big_exit_time.convert(okapi::millisecond);
  int vet = p_velocity_exit_time.convert(okapi::millisecond);
  int mAt = p_mA_timeout.convert(okapi::millisecond);

  pid_drive_exit_condition_set(set, se, bet, be, vet, mAt, use_imu);
}

void Drive::pid_turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu) {
  turnPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  turnPID.velocity_sensor_secondary_toggle_set(use_imu);
}

void Drive::pid_turn_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QAngle p_small_error, okapi::QTime p_big_exit_time, okapi::QAngle p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu) {
  // Convert okapi units to doubles
  double se = p_small_error.convert(okapi::degree);
  double be = p_big_error.convert(okapi::degree);
  int set = p_small_exit_time.convert(okapi::millisecond);
  int bet = p_big_exit_time.convert(okapi::millisecond);
  int vet = p_velocity_exit_time.convert(okapi::millisecond);
  int mAt = p_mA_timeout.convert(okapi::millisecond);

  pid_turn_exit_condition_set(set, se, bet, be, vet, mAt, use_imu);
}

void Drive::pid_swing_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu) {
  swingPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  swingPID.velocity_sensor_secondary_toggle_set(use_imu);
}

void Drive::pid_swing_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QAngle p_small_error, okapi::QTime p_big_exit_time, okapi::QAngle p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu) {
  // Convert okapi units to doubles
  double se = p_small_error.convert(okapi::degree);
  double be = p_big_error.convert(okapi::degree);
  int set = p_small_exit_time.convert(okapi::millisecond);
  int bet = p_big_exit_time.convert(okapi::millisecond);
  int vet = p_velocity_exit_time.convert(okapi::millisecond);
  int mAt = p_mA_timeout.convert(okapi::millisecond);

  pid_swing_exit_condition_set(set, se, bet, be, vet, mAt, use_imu);
}

void Drive::pid_odom_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu) {
  xyPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  xyPID.velocity_sensor_secondary_toggle_set(use_imu);
}

void Drive::pid_odom_drive_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QLength p_small_error, okapi::QTime p_big_exit_time, okapi::QLength p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu) {
  // Convert okapi units to doubles
  double se = p_small_error.convert(okapi::inch);
  double be = p_big_error.convert(okapi::inch);
  int set = p_small_exit_time.convert(okapi::millisecond);
  int bet = p_big_exit_time.convert(okapi::millisecond);
  int vet = p_velocity_exit_time.convert(okapi::millisecond);
  int mAt = p_mA_timeout.convert(okapi::millisecond);

  pid_odom_drive_exit_condition_set(set, se, bet, be, vet, mAt, use_imu);
}

void Drive::pid_odom_turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu) {
  current_a_odomPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  current_a_odomPID.velocity_sensor_secondary_toggle_set(use_imu);
}

void Drive::pid_odom_turn_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QAngle p_small_error, okapi::QTime p_big_exit_time, okapi::QAngle p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu) {
  // Convert okapi units to doubles
  double se = p_small_error.convert(okapi::degree);
  double be = p_big_error.convert(okapi::degree);
  int set = p_small_exit_time.convert(okapi::millisecond);
  int bet = p_big_exit_time.convert(okapi::millisecond);
  int vet = p_velocity_exit_time.convert(okapi::millisecond);
  int mAt = p_mA_timeout.convert(okapi::millisecond);

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
      leftPID.velocity_sensor_secondary_set(drive_imu_accel_get());
      rightPID.velocity_sensor_secondary_set(drive_imu_accel_get());
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

    // Wait until pure pursuit is on the last point, then continue as normal
    if (mode == PURE_PURSUIT) {
      while (pp_index != pp_movements.size() - 1) {
        xyPID.velocity_sensor_secondary_set(drive_imu_accel_get());
        current_a_odomPID.velocity_sensor_secondary_set(drive_imu_accel_get());
        xy_exit = xy_exit != RUNNING ? xy_exit : xyPID.exit_condition({left_motors[0], right_motors[0]});
        a_exit = a_exit != RUNNING ? a_exit : current_a_odomPID.exit_condition({left_motors[0], right_motors[0]});

        if ((xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT) && (a_exit == mA_EXIT || a_exit == VELOCITY_EXIT)) {
          if (print_toggle) std::cout << "  XY: " << exit_to_string(xy_exit) << " Exited early, error: " << xyPID.error << ".   Angle: " << exit_to_string(a_exit) << " Exited early, error: " << current_a_odomPID.error << ".\n";
          break;
        }

        pros::delay(util::DELAY_TIME);
      }
    }

    // When we're at the last point in PP / we're just going to point
    while (xy_exit == RUNNING || a_exit == RUNNING) {
      xyPID.velocity_sensor_secondary_set(drive_imu_accel_get());
      current_a_odomPID.velocity_sensor_secondary_set(drive_imu_accel_get());
      xy_exit = xy_exit != RUNNING ? xy_exit : xyPID.exit_condition({left_motors[0], right_motors[0]});
      a_exit = a_exit != RUNNING ? a_exit : current_a_odomPID.exit_condition({left_motors[0], right_motors[0]});
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle) std::cout << "  XY: " << exit_to_string(xy_exit) << " Exit, error: " << xyPID.error << ".   Angle: " << exit_to_string(a_exit) << " Exit, error: " << current_a_odomPID.error << ".\n";

    if (xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT || a_exit == mA_EXIT || a_exit == VELOCITY_EXIT) {
      interfered = true;
    }
  }

  // Turn Exit
  else if (mode == TURN || mode == TURN_TO_POINT) {
    exit_output turn_exit = RUNNING;
    while (turn_exit == RUNNING) {
      turnPID.velocity_sensor_secondary_set(drive_imu_accel_get());
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
      swingPID.velocity_sensor_secondary_set(drive_imu_accel_get());
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
      if (left_exit == RUNNING || right_exit == RUNNING) {
        leftPID.velocity_sensor_secondary_set(drive_imu_accel_get());
        rightPID.velocity_sensor_secondary_set(drive_imu_accel_get());
        left_exit = left_exit != RUNNING ? left_exit : leftPID.exit_condition(left_motors[0]);
        right_exit = right_exit != RUNNING ? right_exit : rightPID.exit_condition(right_motors[0]);
        pros::delay(util::DELAY_TIME);
      } else {
        if (print_toggle) {
          std::cout << "  Left: " << exit_to_string(left_exit) << " Wait Until Exit Failsafe, triggered at " << drive_sensor_left() - l_start << " instead of " << l_tar << "\n";
          std::cout << "  Right: " << exit_to_string(right_exit) << " Wait Until Exit Failsafe, triggered at " << drive_sensor_right() - r_start << " instead of " << r_tar << "\n";
        }
        if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
          interfered = true;
        }
        return;
      }
    }
    // Once we've past target, return
    else if (util::sgn(l_error) != l_sgn || util::sgn(r_error) != r_sgn) {
      if (print_toggle) printf("  Drive Wait Until Exit Success. Triggered at: L,R(%.2f, %.2f)  Target: L,R(%.2f, %.2f)\n", drive_sensor_left() - l_start, drive_sensor_right() - r_start, l_tar, r_tar);
      leftPID.timers_reset();
      rightPID.timers_reset();
      return;
    }

    pros::delay(util::DELAY_TIME);
  }
}

// Function to wait until a certain position is reached.  Wrapper for exit condition.
void Drive::wait_until_turn_swing(double target) {
  // Make sure mode is correct
  if (!(mode == TURN || mode == SWING || mode == TURN_TO_POINT)) {
    printf("Mode needs to be swing or turn!\n");
    return;
  }

  // Create new target that is the shortest from current
  target = new_turn_target_compute(target, drive_imu_get(), shortest);

  // Calculate error between current and target (target needs to be an in between position)
  double g_error = target - drive_imu_get();
  int g_sgn = util::sgn(g_error);

  exit_output turn_exit = RUNNING;
  exit_output swing_exit = RUNNING;

  pros::Motor& sensor = current_swing == ez::LEFT_SWING ? left_motors[0] : right_motors[0];

  while (true) {
    g_error = target - drive_imu_get();

    // If turning...
    if (mode == TURN || mode == TURN_TO_POINT) {
      // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
      if (util::sgn(g_error) == g_sgn) {
        if (turn_exit == RUNNING) {
          turnPID.velocity_sensor_secondary_set(drive_imu_accel_get());
          turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition({left_motors[0], right_motors[0]});
          pros::delay(util::DELAY_TIME);
        } else {
          if (print_toggle) std::cout << "  Turn: " << exit_to_string(turn_exit) << " Wait Until Exit Failsafe, triggered at " << drive_imu_get() << " instead of " << target << "\n";

          if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
            interfered = true;
          }
          return;
        }
      }
      // Once we've past target, return
      else if (util::sgn(g_error) != g_sgn) {
        if (print_toggle) printf("  Turn Wait Until Exit Success, triggered at %.2f.  Target: %.2f\n", drive_imu_get(), target);
        turnPID.timers_reset();
        return;
      }
    }

    // If swinging...
    else {
      // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
      if (util::sgn(g_error) == g_sgn) {
        if (swing_exit == RUNNING) {
          swingPID.velocity_sensor_secondary_set(drive_imu_accel_get());
          swing_exit = swing_exit != RUNNING ? swing_exit : swingPID.exit_condition(sensor);
          pros::delay(util::DELAY_TIME);
        } else {
          if (print_toggle) std::cout << "  Swing: " << exit_to_string(swing_exit) << " Wait Until Exit Failsafe, triggered at " << drive_imu_get() << " instead of " << target << "\n";

          if (swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) {
            interfered = true;
          }
          return;
        }
      }
      // Once we've past target, return
      else if (util::sgn(g_error) != g_sgn) {
        if (print_toggle) printf("  Swing Wait Until Exit Success, triggered at %.2f. Target: %.2f\n", drive_imu_get(), target);
        swingPID.timers_reset();
        return;
      }
    }

    pros::delay(util::DELAY_TIME);
  }
}

void Drive::pid_wait_until(okapi::QLength target) {
  // If robot is driving...
  if (mode == DRIVE || mode == POINT_TO_POINT || mode == PURE_PURSUIT) {
    wait_until_drive(target.convert(okapi::inch));
  } else {
    printf("QLength not supported for turn or swing!\n");
  }
}

void Drive::pid_wait_until(okapi::QAngle target) {
  // If robot is driving...
  if (mode == TURN || mode == SWING || mode == TURN_TO_POINT) {
    wait_until_turn_swing(target.convert(okapi::degree));
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

  while (true) {
    xyPID.velocity_sensor_secondary_set(drive_imu_accel_get());
    current_a_odomPID.velocity_sensor_secondary_set(drive_imu_accel_get());
    xy_exit = xy_exit != RUNNING ? xy_exit : xyPID.exit_condition({left_motors[0], right_motors[0]});
    a_exit = a_exit != RUNNING ? a_exit : current_a_odomPID.exit_condition({left_motors[0], right_motors[0]});

    if (xy_exit != RUNNING && a_exit != RUNNING) {
      if (print_toggle) {
        std::cout << "  XY: " << exit_to_string(xy_exit) << " Wait Until Exit Failsafe, triggered at (" << odom_x_get() << ", " << odom_y_get() << ") instead of (" << target.x << ", " << target.y << ")\n";
        xyPID.timers_reset();
        current_a_odomPID.timers_reset();
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

  if (index > injected_pp_index.size() - 2 || index < 0)
    printf("  Wait Until PP Error!  Index %i is not within range!  %i is max!\n", index, injected_pp_index.size() - 2);
  index += 1;

  exit_output xy_exit = RUNNING;
  exit_output a_exit = RUNNING;
  while (pp_index < injected_pp_index[index]) {
    xyPID.velocity_sensor_secondary_set(drive_imu_accel_get());
    current_a_odomPID.velocity_sensor_secondary_set(drive_imu_accel_get());
    xy_exit = xy_exit != RUNNING ? xy_exit : xyPID.exit_condition({left_motors[0], right_motors[0]});
    a_exit = a_exit != RUNNING ? a_exit : current_a_odomPID.exit_condition({left_motors[0], right_motors[0]});

    if (xy_exit != RUNNING && a_exit != RUNNING) {
      if (print_toggle) {
        std::cout << "  XY: " << exit_to_string(xy_exit) << " Wait Until Exit Failsafe, triggered at (" << odom_x_get() << ", " << odom_y_get() << ") instead of (" << pp_movements[index].target.x << ", " << pp_movements[index].target.y << ")\n";
        xyPID.timers_reset();
        current_a_odomPID.timers_reset();
      }
      break;
    }

    pros::delay(util::DELAY_TIME);
  }
}

void Drive::pid_wait_until_index(int index) {
  pid_wait_until_index_started(index);
  index += 1;
  pose target = pp_movements[injected_pp_index[index]].target;
  pid_wait_until_point(target);
}

// Pid wait, but quickly :)
void Drive::pid_wait_quick() {
  if (mode == PURE_PURSUIT) {
    pid_wait_until_index(injected_pp_index.size() - 2);
    return;
  } else if (mode == POINT_TO_POINT) {
    pid_wait_until_point(odom_target_start);
    return;
  } else if (!(mode == DRIVE || mode == TURN || mode == SWING || mode == TURN_TO_POINT)) {
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
void Drive::pid_drive_chain_constant_set(okapi::QLength input) { pid_drive_chain_constant_set(input.convert(okapi::inch)); }
void Drive::pid_drive_chain_forward_constant_set(okapi::QLength input) { pid_drive_chain_forward_constant_set(input.convert(okapi::inch)); }
void Drive::pid_drive_chain_backward_constant_set(okapi::QLength input) { pid_drive_chain_backward_constant_set(input.convert(okapi::inch)); }

// Set turn motion chain constants
void Drive::pid_turn_chain_constant_set(double input) { turn_motion_chain_scale = fabs(input); }
void Drive::pid_turn_chain_constant_set(okapi::QAngle input) { pid_turn_chain_constant_set(input.convert(okapi::degree)); }

// Set swing motion chain constants
void Drive::pid_swing_chain_constant_set(double input) {
  pid_swing_chain_forward_constant_set(input);
  pid_swing_chain_backward_constant_set(input);
}
void Drive::pid_swing_chain_forward_constant_set(double input) { swing_forward_motion_chain_scale = fabs(input); }
void Drive::pid_swing_chain_backward_constant_set(double input) { swing_backward_motion_chain_scale = fabs(input); }
void Drive::pid_swing_chain_constant_set(okapi::QAngle input) { pid_swing_chain_constant_set(input.convert(okapi::degree)); }
void Drive::pid_swing_chain_forward_constant_set(okapi::QAngle input) { pid_swing_chain_forward_constant_set(input.convert(okapi::degree)); }
void Drive::pid_swing_chain_backward_constant_set(okapi::QAngle input) { pid_swing_chain_backward_constant_set(input.convert(okapi::degree)); }

// Get motion chain constants
double Drive::pid_drive_chain_forward_constant_get() { return drive_forward_motion_chain_scale; }
double Drive::pid_drive_chain_backward_constant_get() { return drive_backward_motion_chain_scale; }
double Drive::pid_turn_chain_constant_get() { return turn_motion_chain_scale; }
double Drive::pid_swing_chain_forward_constant_get() { return swing_forward_motion_chain_scale; }
double Drive::pid_swing_chain_backward_constant_get() { return swing_backward_motion_chain_scale; }

// Pid wait that hold momentum into the next motion
void Drive::pid_wait_quick_chain() {
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
    printf("Not in a supported drive mode!\n");
    return;
  }

  // Exit at the real target
  pid_wait_quick();
}
