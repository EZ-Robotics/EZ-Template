/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

using namespace ez;


// Set exit condition timeouts
void drive::set_exit_condition(exit_condition_ &type, int p_small_exit_time, int p_small_error, int p_big_exit_time, int p_big_error, int p_velocity_exit_time, int p_mA_thresh, int p_mA_timeout) {
  type.small_exit_time = p_small_exit_time;
  type.small_error = p_small_error;
  type.big_exit_time = p_big_exit_time;
  type.big_error = p_big_error;
  type.velocity_exit_time = p_velocity_exit_time;
  type.mA_thresh = p_mA_thresh;
  type.mA_timeout = p_mA_timeout;
}

// Exit condition
bool drive::exit_condition(std::tuple<double, std::optional<double>> targets, exit_condition_ input_struct, bool wait_until)
{
  static int i = 0, j = 0, k = 0, l = 0;
  bool is_drive = std::get<1>(targets).has_value();

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (fabs(std::get<0>(targets)-(is_drive ? left_sensor() : get_gyro())) < input_struct.small_error) {
    if(!is_drive || fabs(*std::get<1>(targets)-right_sensor()) < input_struct.small_error) {
      j+=util::DELAY_TIME;
      i = 0; // While this is running, don't run big thresh
      if (j>input_struct.small_exit_time) {
        if (!wait_until) printf(" Timed Out");
        else             printf(" Wait Until Timed Out");
        printf(" - Small Thresh\n");

        l=0;i=0;k=0;j=0;
        return false;
      }
    }
  }
  else {
    j = 0;
  }

  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.  This does not run while small_timeout is running
  if (fabs(std::get<0>(targets)-(is_drive ? left_sensor() : get_gyro()))<input_struct.big_error) {
    if(!is_drive || fabs(*std::get<1>(targets)-right_sensor())<input_struct.big_error) {
      i+=util::DELAY_TIME;
      if (i>input_struct.big_exit_time) {
        if (!wait_until) printf(" Timed Out");
        else             printf(" Wait Until Timed Out");
        printf(" - Big Thresh\n");

        l=0;i=0;k=0;j=0;
        return false;
      }
    }
  }
  else {
    i = 0;
  }

  // If the motors are pulling too many mA, the code will timeout and set interfered to true.
  if (right_mA()>=input_struct.mA_thresh || left_mA()>=input_struct.mA_thresh) {
    l+=util::DELAY_TIME;
    if (l>input_struct.mA_timeout) {
      if (!wait_until) printf(" Timed Out");
      else             printf(" Wait Until Timed Out");
      printf(" - mA\n");

      l=0;i=0;k=0;j=0;
      interfered = true;
      return false;
    }
  }
  else {
    l = 0;
  }

  if (right_velocity()==0 && left_velocity()==0) {
    k+=util::DELAY_TIME;
    if (k>input_struct.velocity_exit_time) {
      if (!wait_until) printf(" Timed Out");
      else             printf(" Wait Until Timed Out");
      printf(" - Velocity\n");

      l=0;i=0;k=0;j=0;
      interfered = true;
      return false;
    }
  }
  else {
    k = 0;
  }

  interfered = false;
  return true;
}

// User wrapper for exit condition
void drive::wait_drive() {
  pros::delay(util::DELAY_TIME);

  // if (drive_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {
  if (mode == DRIVE) {
    while (exit_condition(tuple{leftPID.get_target(), rightPID.get_target()}, drive_exit)) {
      pros::delay(util::DELAY_TIME);
    }
  }
  // else if (turn_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {
  else if (mode == TURN) {
    while (exit_condition(tuple{turnPID.get_target(), std::nullopt}, turn_exit)) {
      pros::delay(util::DELAY_TIME);
    }
  }
  // else if (swing_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {
  else if (mode == SWING) {
    while (exit_condition(tuple{swingPID.get_target(), std::nullopt}, swing_exit)) {
      pros::delay(util::DELAY_TIME);
    }
  }
}

// Function to wait until a certain position is reached.  Wrapper for exit condition.
void drive::wait_until(double target) {

  // If robot is driving...
  // if (drive_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {
  if (mode == DRIVE) {
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
      else if (!exit_condition(tuple{l_tar, r_tar}, drive_exit, true)) {
        return;
      }

      pros::delay(util::DELAY_TIME);
    }
  }

  // If robot is turning...
  // else if (turn_pid.get_state()!=pros::E_TASK_STATE_SUSPENDED || swing_pid.get_state()!=pros::E_TASK_STATE_SUSPENDED) {
  else if (mode == TURN || mode == SWING) {
    // Calculate error between current and target (target needs to be an inbetween position)
    int g_error = target - get_gyro();
    int g_sgn   = util::sgn(g_error);
    bool run    = true;

    // Change exit condition constants from turn to swing
    exit_condition_ current_exit;
    // if (turn_pid.get_state()!=pros::E_TASK_STATE_SUSPENDED)
    if (mode == TURN)
      current_exit = turn_exit;
    else
      current_exit = swing_exit;

    while (true) {
      g_error = target - get_gyro();

      // Break the loop once target is passed
      if (util::sgn(g_error)==g_sgn) {
        // this makes sure that the following else if is rnu after the sgn is flipped
      }
      else if (util::sgn(g_error)!=g_sgn) {
        return;
      }
      else if (!exit_condition(tuple{target, std::nullopt}, current_exit, true)) {
        return;
      }

      pros::delay(util::DELAY_TIME);
    }
  }
}
