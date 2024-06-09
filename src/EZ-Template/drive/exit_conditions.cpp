/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/util.hpp"
#include "main.h"

using namespace ez;

void Drive::pid_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu) {
  leftPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  rightPID.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  leftPID.velocity_sensor_secondary_toggle_set(use_imu);
  rightPID.velocity_sensor_secondary_toggle_set(use_imu);
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

  // Turn Exit
  else if (mode == TURN) {
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
  if (!(mode == DRIVE)) {
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
  if (!(mode == TURN || mode == SWING)) {
    printf("Mode needs to be swing or turn!\n");
    return;
  }

  // Calculate error between current and target (target needs to be an in between position)
  double g_error = target - drive_imu_get();
  int g_sgn = util::sgn(g_error);

  exit_output turn_exit = RUNNING;
  exit_output swing_exit = RUNNING;

  pros::Motor& sensor = current_swing == ez::LEFT_SWING ? left_motors[0] : right_motors[0];

  while (true) {
    g_error = target - drive_imu_get();

    // If turning...
    if (mode == TURN) {
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
  if (mode == DRIVE) {
    wait_until_drive(target.convert(okapi::inch));
  } else {
    printf("QLength not supported for turn or swing!\n");
  }
}

void Drive::pid_wait_until(okapi::QAngle target) {
  // If robot is driving...
  if (mode == TURN || mode == SWING) {
    wait_until_turn_swing(target.convert(okapi::degree));
  } else {
    printf("QAngle not supported for drive!\n");
  }
}

void Drive::pid_wait_until(double target) {
  // If driving...
  if (mode == DRIVE) {
    wait_until_drive(target);
  }
  // If turning or swinging...
  else if (mode == TURN || mode == SWING) {
    wait_until_turn_swing(target);
  } else {
    printf("Not in a valid drive mode!\n");
  }
}

// Pid wait, but quickly :)
void Drive::pid_wait_quick() {
  if (!(mode == DRIVE || mode == TURN || mode == SWING)) {
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
  } else {
    printf("Not in a valid drive mode!\n");
    return;
  }

  // Exit at the real target
  pid_wait_quick();
}
