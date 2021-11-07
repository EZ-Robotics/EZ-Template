/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

using namespace ez;


// Set exit condition timeouts
void Drive::set_exit_condition(exit_condition_ &type, int p_small_exit_time, int p_small_error, int p_big_exit_time, int p_big_error, int p_velocity_exit_time) {
  type.small_exit_time = p_small_exit_time;
  type.small_error = p_small_error;
  type.big_exit_time = p_big_exit_time;
  type.big_error = p_big_error;
  type.velocity_exit_time = p_velocity_exit_time;
}


// Exit condition
bool Drive::exit_condition(tuple<double, std::optional<double>> targets, exit_condition_ exitConditions)
{

  static int i = 0, j = 0, k = 0;
  int delay_time = util::DELAY_TIME;
  bool isDrive = std::get<1>(targets).has_value();

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (fabs(std::get<0>(targets)-(isDrive ? left_sensor() : get_gyro())) < exitConditions.small_error) {
    if(!isDrive || fabs(*std::get<1>(targets)-right_sensor()) < exitConditions.small_error) {
      j+=delay_time;
      //printf("\nJ: %i", j);

      if (j>exitConditions.small_exit_time) {
        i=0;k=0;j=0;
        return false;
      }
    }
  }
  else {
    j = 0;
  }

  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.
  if (fabs(std::get<0>(targets)-(isDrive ? left_sensor() : get_gyro()))<exitConditions.big_error) {
    if(!isDrive || fabs(*std::get<1>(targets)-right_sensor())<exitConditions.big_error) {
      i+=delay_time;
      //printf("\nI: %i", i);

      if (i>exitConditions.big_exit_time) {
        i=0;k=0;j=0;
        return false;
      }
    }
  }
  else {
    i = 0;
  }

  if (right_velocity()==0 && left_velocity()==0) {
    k+=delay_time;
    //printf("\nI: %i", i);

    if (k>exitConditions.velocity_exit_time) {
      i=0;k=0;j=0;
      return false;
    }
  }
  else {
    k = 0;
  }

  return true;
}

// User wrapper for exit condition
void Drive::wait_drive() {
  int delay_time = util::DELAY_TIME;
  pros::delay(delay_time);

  if (drive_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {

    while (exit_condition(tuple{leftPID.GetTarget(), rightPID.GetTarget()}, drive_exit)) {
      //printf("%i\n",exit_condition(tuple{leftPID.GetTarget(), rightPID.GetTarget()}, drive_exit));
      pros::delay(delay_time);
    }
  }
  else if (turn_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {
    while (exit_condition(tuple{turnPID.GetTarget(), std::nullopt}, turn_exit)) {
      pros::delay(delay_time);
    }
  }
  else if (swing_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {
    while (exit_condition(tuple{swingPID.GetTarget(), std::nullopt}, swing_exit)) {
      pros::delay(delay_time);
    }
  }
}

// Function to wait until a certain position is reached.  Wrapper for exit condition.
void Drive::wait_until(double target) {

  // If robot is driving...
  if (drive_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {
    // If robot is driving...
    // Calculate error between current and target (target needs to be an inbetween position)
    int l_tar   = l_start + (target*TICK_PER_INCH);
    int r_tar   = r_start + (target*TICK_PER_INCH);
    int l_error = l_tar   - left_sensor();
    int r_error = r_tar   - right_sensor();
    int l_sgn   = util::sgn(l_error);
    int r_sgn   = util::sgn(r_error);

    while (true) {
      l_error = l_tar - left_sensor();
      r_error = r_tar - right_sensor();

      // Break the loop once target is passed
      if (util::sgn(l_error)==l_sgn && util::sgn(r_error)==r_sgn) {
        // this makes sure that the following else if is rnu after the sgn is flipped
      }
      else if (util::sgn(l_error)!=l_sgn && util::sgn(r_error)!=r_sgn) {
        return;
      }
      else if (!exit_condition(tuple{l_tar, r_tar}, drive_exit)) {
        return;
      }

      pros::delay(util::DELAY_TIME);
    }
  }

  // If robot is turning...
  else if (turn_pid.get_state()!=pros::E_TASK_STATE_SUSPENDED || swing_pid.get_state()!=pros::E_TASK_STATE_SUSPENDED) {
    // Calculate error between current and target (target needs to be an inbetween position)
    int g_error = target - get_gyro();
    int g_sgn   = util::sgn(g_error);
    bool run    = true;

    while (true) {
      g_error = target - get_gyro();

      // Break the loop once target is passed
      if (util::sgn(g_error)==g_sgn) {
        // this makes sure that the following else if is rnu after the sgn is flipped
      }
      else if (util::sgn(g_error)!=g_sgn) {
        return;
      }
      else if (!exit_condition(tuple{target, std::nullopt}, turn_exit)) {
        return;
      }

      pros::delay(util::DELAY_TIME);
    }
  }
}
