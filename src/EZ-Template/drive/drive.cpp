/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include <list>


// !Util

Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double motor_cartridge, double ratio)
 : gyro (imu_port), master(pros::E_CONTROLLER_MASTER),
 drive_pid([this]{ this->drive_pid_task(); }),
 turn_pid([this]{ this->turn_pid_task(); }),
 swing_pid([this]{ this->swing_pid_task(); })
{
  // Set ports to a global vector
  for(auto i : left_motor_ports)
  {
    pros::Motor temp(abs(i), ez::util::isReversed(i));
    LeftMotors.push_back(temp);
  }
  for(auto i : right_motor_ports)
  {
    pros::Motor temp(abs(i), ez::util::isReversed(i));
    RightMotors.push_back(temp);
  }

  // Tick per inch calculation
  TICK_PER_REV  = (50.0*(3600.0/motor_cartridge)) * ratio; // with no cart, the encoder reads 50 counts per rotation
  CIRCUMFERENCE = wheel_diameter*M_PI;
  TICK_PER_INCH = (TICK_PER_REV/CIRCUMFERENCE);

  // PID Constants
  headingPID = {11, 0, 20, 0};
  forwardDrivePID = {0.45, 0, 5, 0};
  backwardDrivePID = {0.45, 0, 5, 0};
  turnPID = {5, 0.003, 35, 15};
  swingPID = {12, 0, 35, 0};
  leftPID = {0.45, 0, 5, 0};
  rightPID = {0.45, 0, 5, 0};

  // Slew constants
  set_slew_min_power(80, 80);
  set_slew_distance(7, 7);

  // Exit condition constants
  set_exit_condition(turn_exit,  100, 3,  500, 7,   500);
  set_exit_condition(swing_exit, 100, 3,  500, 7,   500);
  set_exit_condition(drive_exit, 80,  50, 300, 150, 500);

  // Modify joystick curve on controller (defaults to disabled)
  toggle_controller_curve_modifier(false);
}

void Drive::SetPIDConstants(PID pid, double kP, double kI, double kD, double startI)
{
  pid.SetConstants(kP, kI, kD, startI);
}

void
Drive::set_tank(int l, int r) {
  if (pros::millis() < 1500) return;

  for (auto i : LeftMotors) {
    i.move_voltage(l*(12000.0/127.0));
  }
  for (auto i : RightMotors) {
    i.move_voltage(r*(12000.0/127.0));
  }
}


// Motor telemetry
void
Drive::reset_drive_sensor() {
  LeftMotors.front().tare_position();
  RightMotors.front().tare_position();
}

int Drive::right_sensor()   { return RightMotors.front().get_position(); }
int Drive::right_velocity() { return RightMotors.front().get_actual_velocity(); }

int Drive::left_sensor()    { return LeftMotors.front().get_position(); }
int Drive::left_velocity()  { return LeftMotors.front().get_actual_velocity(); }


void  Drive::tare_gyro() { gyro.set_rotation(0); }
float Drive::get_gyro()  { return gyro.get_rotation(); }

bool
Drive::imu_calibrate() {
  gyro.reset();
  int time = pros::millis();
  int iter = 0;
  int delay = 10;
  while (gyro.get_status() & pros::c::E_IMU_STATUS_CALIBRATING) {
    iter += delay;

    if (iter > 2990) {
      printf("No IMU plugged in, (took %d ms to realize that)\n", iter);
      return false;
    }
    pros::delay(delay);
  }
  master.rumble(".");
  printf("IMU is done calibrating (took %d ms)\n", iter);
  return true;
}

// Brake modes
void
Drive::set_drive_brake(pros::motor_brake_mode_e_t input) {
  for (auto i : LeftMotors) {
    i.set_brake_mode(input);
  }
  for (auto i : RightMotors) {
    i.set_brake_mode(input);
  }
}

void
Drive::set_max_speed(int speed) {
  max_speed = speed;
}

void
Drive::set_drive_pid(double target, int speed, bool slew_on, bool toggle_heading) {
  turn_pid.suspend();
  swing_pid.suspend();

  // Global setup
  set_max_speed(abs(speed));
  l.enabled = slew_on;
  r.enabled = slew_on;
  l.max_speed = abs(speed);
  r.max_speed = abs(speed);
  HEADING_ON = toggle_heading;
  bool isBackwards = false;

  double l_target_encoder, r_target_encoder;

  // If drive or line, set targets to drive

    printf("Drive Started... Target Value: %f\n", target);
    l_start = left_sensor();
    r_start = right_sensor();
    l_target_encoder = l_start + (target*TICK_PER_INCH);
    r_target_encoder = r_start + (target*TICK_PER_INCH);
    if (l_target_encoder<l_start && r_target_encoder<r_start) {
      auto consts = backwardDrivePID.GetConstants();
      leftPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);
      rightPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);
      isBackwards = true;
    } else {
      auto consts = forwardDrivePID.GetConstants();
      leftPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);
      rightPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);
      //forwardDrivePID.SetTarget(target);
      isBackwards = false;
    }

    leftPID. SetTarget(l_target_encoder);
    rightPID.SetTarget(r_target_encoder);

    l.sign = ez::util::sgn(l_target_encoder-left_sensor());
    r.sign = ez::util::sgn(r_target_encoder-right_sensor());

    l.x_intercept = l_start + (SLEW_DISTANCE[isBackwards]*TICK_PER_INCH);
    r.x_intercept = r_start + (SLEW_DISTANCE[isBackwards]*TICK_PER_INCH);

    l.y_intercept = abs(speed) * l.sign;
    r.y_intercept = abs(speed) * r.sign;

    l.slope = (SLEW_MIN_POWER[isBackwards]-abs(speed)) / ((l_start+(SLEW_DISTANCE[isBackwards]*TICK_PER_INCH))-0);
    r.slope = (SLEW_MIN_POWER[isBackwards]-abs(speed)) / ((l_start+(SLEW_DISTANCE[isBackwards]*TICK_PER_INCH))-0);

    drive_pid.resume();
}

void Drive::set_turn_pid(double target, int speed)
{
  swing_pid.suspend();
  drive_pid.suspend();

  printf("Turn Started... Target Value: %f\n", target);
  turnPID.SetTarget(target);
  headingPID.SetTarget(target);
  set_max_speed(abs(speed));

  turn_pid.resume();

  //gyro_sign = sgn(target - get_gyro());
}

void Drive::set_swing_pid(e_swing type, double target, int speed)
{
  drive_pid.suspend();
  turn_pid.suspend();

  printf("Swing Started... Target Value: %f\n", target);
  current_swing = type;
  swingPID.SetTarget(target);
  headingPID.SetTarget(target);
  set_max_speed(abs(speed));

  swing_pid.resume();
  //swing_sign = sgn(target-get_gyro());
}

void Drive::set_exit_condition(exit_condition_ &type, int p_small_exit_time, int p_small_error, int p_big_exit_time, int p_big_error, int p_velocity_exit_time) {
  type.small_exit_time = p_small_exit_time;
  type.small_error = p_small_error;
  type.big_exit_time = p_big_exit_time;
  type.big_error = p_big_error;
  type.velocity_exit_time = p_velocity_exit_time;
}



bool Drive::drive_exit_condition(double l_target, double r_target) {
  static int i = 0, j = 0, k = 0;
  int delay_time = ez::util::DELAY_TIME;

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (fabs(l_target-left_sensor())<drive_exit.small_error && fabs(r_target-right_sensor())<drive_exit.small_error) {
    j+=delay_time;
    //printf("\nJ: %i", j/10);

    if (j>drive_exit.small_exit_time) {
      i=0;k=0;j=0;
      return false;
    }
  }
  else {
    j = 0;
  }

  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.
  if (fabs(l_target-left_sensor())<drive_exit.big_error && fabs(r_target-right_sensor())<drive_exit.big_error) {
    i+=delay_time;
    //printf("\nI: %i", i/10);

    if (i>drive_exit.big_exit_time) {
      i=0;k=0;j=0;
      return false;
    }
  }
  else {
    i = 0;
  }

  if (right_velocity()==0 && left_velocity()==0) {
    k+=delay_time;
    //printf("\nI: %i", i/10);

    if (k>drive_exit.velocity_exit_time) {
      i=0;k=0;j=0;
      return false;
    }
  }
  else {
    k = 0;
  }

  return true;
}

bool Drive::turn_exit_condition(double target) {
  static int i = 0, j = 0, k = 0;
  int delay_time = ez::util::DELAY_TIME;

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (fabs(target-get_gyro())<turn_exit.small_error) {
    j+=delay_time;
    //printf("\nJ: %i", j/10);

    if (j>turn_exit.small_exit_time) {
      i=0;k=0;j=0;
      return false;
    }
  }
  else {
    j = 0;
  }
  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.
  if (fabs(target-get_gyro())<turn_exit.big_error) {
    i+=delay_time;
    //printf("\nI: %i", i/10);

    if (i>turn_exit.big_exit_time) {
      i=0;k=0;j=0;
      return false;
    }
  }
  else {
    i = 0;
  }

  if (right_velocity()==0 && left_velocity()==0) {
    k+=delay_time;
    //printf("\nI: %i", i/10);

    if (k>turn_exit.velocity_exit_time) {
      i=0;k=0;j=0;
      return false;
    }
  }
  else {
    k = 0;
  }
  return true;
}

bool Drive::swing_exit_condition(double target) {
  static int i = 0, j = 0, k = 0;
  int delay_time = ez::util::DELAY_TIME;

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (fabs(target-get_gyro())<swing_exit.small_error) {
    j+=delay_time;
    //printf("\nJ: %i", j/10);

    if (j>swing_exit.small_exit_time) {
      i=0;k=0;j=0;
      return false;
    }
  }
  else {
    j = 0;
  }
  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.
  if (fabs(target-get_gyro())<swing_exit.big_error) {
    i+=delay_time;
    //printf("\nI: %i", i/10);

    if (i>swing_exit.big_exit_time) {
      i=0;k=0;j=0;
      return false;
    }
  }
  else {
    i = 0;
  }

  if (right_velocity()==0 && left_velocity()==0) {
    k+=delay_time;
    //printf("\nI: %i", i/10);

    if (k>swing_exit.velocity_exit_time) {
      i=0;k=0;j=0;
      return false;
    }
  }
  else {
    k = 0;
  }
  return true;
}

//Wait for drive
void Drive::wait_drive() {
  int delay_time = ez::util::DELAY_TIME;
  pros::delay(delay_time);

  if (drive_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {
    int i = 0;
    while (drive_exit_condition(leftPID.GetTarget(), rightPID.GetTarget())) {
      i+=delay_time;
      pros::delay(delay_time);
    }
  }
  else if (turn_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {
    while (turn_exit_condition(turnPID.GetTarget())) {
      pros::delay(delay_time);
    }
  }
  else if (swing_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {
    while (swing_exit_condition(swingPID.GetTarget())) {
      pros::delay(delay_time);
    }
  }
}
// Function to wait until a certain position is reached
void Drive::wait_until(double target) {
  const int delay_time = ez::util::DELAY_TIME;

  // If robot is driving...
  if (drive_pid.get_state() != pros::E_TASK_STATE_SUSPENDED) {
    // If robot is driving...
    // Calculate error between current and target (target needs to be an inbetween position)
    int l_tar   = l_start + (target*TICK_PER_INCH);
    int r_tar   = r_start + (target*TICK_PER_INCH);
    int l_error = l_tar   - left_sensor();
    int r_error = r_tar   - right_sensor();
    int l_sgn   = ez::util::sgn(l_error);
    int r_sgn   = ez::util::sgn(r_error);
    bool run    = true;

    while (run) {
      l_error = l_tar - left_sensor();
      r_error = r_tar - right_sensor();

      // Break the loop once target is passed
      if (ez::util::sgn(l_error)==l_sgn && ez::util::sgn(r_error)==r_sgn) {
        run = true; // this makes sure that the following else if is rnu after the sgn is flipped
      }
      else if (ez::util::sgn(l_error)!=l_sgn && ez::util::sgn(r_error)!=r_sgn) {
        run = false;
      }
      else if (!drive_exit_condition(l_tar, r_tar)) {
        run = false;
      }

      pros::delay(delay_time);
    }
  }

  // If robot is turning...
  else if (turn_pid.get_state()!=pros::E_TASK_STATE_SUSPENDED || swing_pid.get_state()!=pros::E_TASK_STATE_SUSPENDED) {
    // Calculate error between current and target (target needs to be an inbetween position)
    int g_error = target - get_gyro();
    int g_sgn   = ez::util::sgn(g_error);
    bool run    = true;

    while (run) {
      g_error = target - get_gyro();

      // Break the loop once target is passed
      if (ez::util::sgn(g_error)==g_sgn) {
        run = true; // this makes sure that the following else if is rnu after the sgn is flipped
      }
      else if (ez::util::sgn(g_error)!=g_sgn) {
        run = false;
      }
      else if (!turn_exit_condition(target)) {
        run = false;
      }

      pros::delay(delay_time);
    }
  }
}
