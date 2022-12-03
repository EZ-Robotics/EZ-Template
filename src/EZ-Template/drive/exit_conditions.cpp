/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/util.hpp"
#include "main.h"

using namespace ez;

void Drive::drive_exit_condition_set(int p_small_exit_time, okapi::QLength p_small_error, int p_big_exit_time, okapi::QLength p_big_error, int p_velocity_exit_time, int p_mA_timeout) {
  pid_left.exit_condition_set(p_small_exit_time, p_small_error.convert(okapi::inch), p_big_exit_time, p_big_error.convert(okapi::inch), p_velocity_exit_time, p_mA_timeout);
  pid_right.exit_condition_set(p_small_exit_time, p_small_error.convert(okapi::inch), p_big_exit_time, p_big_error.convert(okapi::inch), p_velocity_exit_time, p_mA_timeout);
}

void Drive::turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout) {
  pid_turn.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
}
void Drive::swing_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout) {
  pid_swing.exit_condition_set(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
}

// User wrapper for exit condition
void Drive::drive_wait_exit() {
  // Let the PID run at least 1 iteration
  pros::delay(util::DELAY_TIME);

  if (mode == DRIVE) {
    e_exit_output left_exit = RUNNING;
    e_exit_output right_exit = RUNNING;
    while (left_exit == RUNNING || right_exit == RUNNING) {
      left_exit = left_exit != RUNNING ? left_exit : pid_left.exit_condition(motors_left[0]);
      right_exit = right_exit != RUNNING ? right_exit : pid_right.exit_condition(motors_right[0]);
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle) std::cout << "  Left: " << exit_to_string(left_exit) << " Exit, error: " << pid_left.error << ".   Right: " << exit_to_string(right_exit) << " Exit, error: " << pid_right.error << ".\n";

    if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
      interfered = true;
    }
  }

  // Turn Exit
  else if (mode == TURN) {
    e_exit_output turn_exit = RUNNING;
    while (turn_exit == RUNNING) {
      turn_exit = turn_exit != RUNNING ? turn_exit : pid_turn.exit_condition({motors_left[0], motors_right[0]});
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle) std::cout << "  Turn: " << exit_to_string(turn_exit) << " Exit, error:" << pid_turn.error << ".\n";

    if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
      interfered = true;
    }
  }

  // Swing Exit
  else if (mode == SWING) {
    e_exit_output swing_exit = RUNNING;
    pros::Motor& sensor = swing_current == ez::LEFT_SWING ? motors_left[0] : motors_right[0];
    while (swing_exit == RUNNING) {
      swing_exit = swing_exit != RUNNING ? swing_exit : pid_swing.exit_condition(sensor);
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle) std::cout << "  Swing: " << exit_to_string(swing_exit) << " Exit, error:" << pid_swing.error << ".\n";

    if (swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) {
      interfered = true;
    }
  }
}

void Drive::drive_wait_distance(double target) {
  // If robot is driving...
  if (mode == DRIVE) {
    // Calculate error between current and target (target needs to be an in between position)
    int l_tar = start_left + target;
    int r_tar = start_right + target;
    int l_error = l_tar - sensor_left();
    int r_error = r_tar - sensor_right();
    int l_sgn = util::sign(l_error);
    int r_sgn = util::sign(r_error);

    e_exit_output left_exit = RUNNING;
    e_exit_output right_exit = RUNNING;

    while (true) {
      l_error = l_tar - sensor_left();
      r_error = r_tar - sensor_right();

      // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
      if (util::sign(l_error) == l_sgn || util::sign(r_error) == r_sgn) {
        if (left_exit == RUNNING || right_exit == RUNNING) {
          left_exit = left_exit != RUNNING ? left_exit : pid_left.exit_condition(motors_left[0]);
          right_exit = right_exit != RUNNING ? right_exit : pid_right.exit_condition(motors_right[0]);
          pros::delay(util::DELAY_TIME);
        } else {
          if (print_toggle) std::cout << "  Left: " << exit_to_string(left_exit) << " Wait Until Exit.   Right: " << exit_to_string(right_exit) << " Wait Until Exit.\n";

          if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
            interfered = true;
          }
          return;
        }
      }
      // Once we've past target, return
      else if (util::sign(l_error) != l_sgn || util::sign(r_error) != r_sgn) {
        if (print_toggle) std::cout << "  Drive Wait Until Exit.\n";
        return;
      }

      pros::delay(util::DELAY_TIME);
    }
  }
}

void Drive::drive_wait_until(okapi::QLength target) {
  // If robot is driving...
  if (mode == DRIVE) {
    drive_wait_until(target.convert(okapi::inch));
  } else {
    printf("QLength not supported for turn or swing!\n");
  }
}

// Function to wait until a certain position is reached.  Wrapper for exit condition.
void Drive::drive_wait_until(double target) {
  // If robot is driving...
  if (mode == DRIVE) {
    drive_wait_until(target);
  }

  // If robot is turning or swinging...
  else if (mode == TURN || mode == SWING) {
    // Calculate error between current and target (target needs to be an in between position)
    int g_error = target - imu_get();
    int g_sgn = util::sign(g_error);

    e_exit_output turn_exit = RUNNING;
    e_exit_output swing_exit = RUNNING;

    pros::Motor& sensor = swing_current == ez::LEFT_SWING ? motors_left[0] : motors_right[0];

    while (true) {
      g_error = target - imu_get();

      // If turning...
      if (mode == TURN) {
        // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
        if (util::sign(g_error) == g_sgn) {
          if (turn_exit == RUNNING) {
            turn_exit = turn_exit != RUNNING ? turn_exit : pid_turn.exit_condition({motors_left[0], motors_right[0]});
            pros::delay(util::DELAY_TIME);
          } else {
            if (print_toggle) std::cout << "  Turn: " << exit_to_string(turn_exit) << " Wait Until Exit.\n";

            if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
              interfered = true;
            }
            return;
          }
        }
        // Once we've past target, return
        else if (util::sign(g_error) != g_sgn) {
          if (print_toggle) std::cout << "  Turn Wait Until Exit.\n";
          return;
        }
      }

      // If swinging...
      else {
        // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
        if (util::sign(g_error) == g_sgn) {
          if (swing_exit == RUNNING) {
            swing_exit = swing_exit != RUNNING ? swing_exit : pid_swing.exit_condition(sensor);
            pros::delay(util::DELAY_TIME);
          } else {
            if (print_toggle) std::cout << "  Swing: " << exit_to_string(swing_exit) << " Wait Until Exit.\n";

            if (swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) {
              interfered = true;
            }
            return;
          }
        }
        // Once we've past target, return
        else if (util::sign(g_error) != g_sgn) {
          if (print_toggle) std::cout << "  Swing Wait Until Exit.\n";
          return;
        }
      }

      pros::delay(util::DELAY_TIME);
    }
  }
}
