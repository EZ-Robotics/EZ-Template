/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/util.hpp"
#include "main.h"

using namespace ez;

// Set exit condition timeouts
void Drive::set_exit_condition(int type, int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout) {
  if (type == drive_exit) {
    leftPID.set_exit_condition(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
    rightPID.set_exit_condition(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  }

  if (type == turn_exit) {
    turnPID.set_exit_condition(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  }

  if (type == swing_exit) {
    swingPID.set_exit_condition(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
  }
}

// User wrapper for exit condition
void Drive::wait_drive() {
  // Let the PID run at least 1 iteration
  pros::delay(util::DELAY_TIME);

  if (mode == DRIVE) {
    exit_output left_exit = RUNNING;
    exit_output right_exit = RUNNING;
    while (left_exit == RUNNING || right_exit == RUNNING) {
      left_exit = left_exit != RUNNING ? left_exit : leftPID.exit_condition(left_motors[0]);
      right_exit = right_exit != RUNNING ? right_exit : rightPID.exit_condition(right_motors[0]);
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle) std::cout << "  Left: " << exit_to_string(left_exit) << " Exit.   Right: " << exit_to_string(right_exit) << " Exit.\n";

    if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
      interfered = true;
    }
  }

  // Turn Exit
  else if (mode == TURN) {
    exit_output turn_exit = RUNNING;
    while (turn_exit == RUNNING) {
      turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition({left_motors[0], right_motors[0]});
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle) std::cout << "  Turn: " << exit_to_string(turn_exit) << " Exit.\n";

    if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
      interfered = true;
    }
  }

  // Swing Exit
  else if (mode == SWING) {
    exit_output swing_exit = RUNNING;
    pros::Motor& sensor = current_swing == ez::LEFT_SWING ? left_motors[0] : right_motors[0];
    while (swing_exit == RUNNING) {
      swing_exit = swing_exit != RUNNING ? swing_exit : swingPID.exit_condition(sensor);
      pros::delay(util::DELAY_TIME);
    }
    if (print_toggle) std::cout << "  Swing: " << exit_to_string(swing_exit) << " Exit.\n";

    if (swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) {
      interfered = true;
    }
  }
}

// Function to wait until a certain position is reached.  Wrapper for exit condition.
void Drive::wait_until(double target) {
  // If robot is driving...
  if (mode == DRIVE) {
    // Calculate error between current and target (target needs to be an in between position)
    int l_tar = l_start + (target * TICK_PER_INCH);
    int r_tar = r_start + (target * TICK_PER_INCH);
    int l_error = l_tar - left_sensor();
    int r_error = r_tar - right_sensor();
    int l_sgn = util::sgn(l_error);
    int r_sgn = util::sgn(r_error);

    exit_output left_exit = RUNNING;
    exit_output right_exit = RUNNING;

    while (true) {
      l_error = l_tar - left_sensor();
      r_error = r_tar - right_sensor();

      // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
      if (util::sgn(l_error) == l_sgn || util::sgn(r_error) == r_sgn) {
        if (left_exit == RUNNING || right_exit == RUNNING) {
          left_exit = left_exit != RUNNING ? left_exit : leftPID.exit_condition(left_motors[0]);
          right_exit = right_exit != RUNNING ? right_exit : rightPID.exit_condition(right_motors[0]);
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
      else if (util::sgn(l_error) != l_sgn || util::sgn(r_error) != r_sgn) {
        if (print_toggle) std::cout << "  Drive Wait Until Exit.\n";
        return;
      }

      pros::delay(util::DELAY_TIME);
    }
  }

  // If robot is turning or swinging...
  else if (mode == TURN || mode == SWING) {
    // Calculate error between current and target (target needs to be an in between position)
    int g_error = target - get_gyro();
    int g_sgn = util::sgn(g_error);

    exit_output turn_exit = RUNNING;
    exit_output swing_exit = RUNNING;

    pros::Motor& sensor = current_swing == ez::LEFT_SWING ? left_motors[0] : right_motors[0];

    while (true) {
      g_error = target - get_gyro();

      // If turning...
      if (mode == TURN) {
        // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
        if (util::sgn(g_error) == g_sgn) {
          if (turn_exit == RUNNING) {
            turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition({left_motors[0], right_motors[0]});
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
        else if (util::sgn(g_error) != g_sgn) {
          if (print_toggle) std::cout << "  Turn Wait Until Exit.\n";
          return;
        }
      }

      // If swinging...
      else {
        // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
        if (util::sgn(g_error) == g_sgn) {
          if (swing_exit == RUNNING) {
            swing_exit = swing_exit != RUNNING ? swing_exit : swingPID.exit_condition(sensor);
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
        else if (util::sgn(g_error) != g_sgn) {
          if (print_toggle) std::cout << "  Swing Wait Until Exit.\n";
          return;
        }
      }

      pros::delay(util::DELAY_TIME);
    }
  }
}
