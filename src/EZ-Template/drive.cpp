/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include <list>

#include "EZ-Template/Helper.hpp"
// !Util

Drive::Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double motor_cartridge, double ratio)
 : gyro (imu_port), master(pros::E_CONTROLLER_MASTER),
 drive_pid([this]{ this->drive_pid_task(); }),
 turn_pid([this]{ this->turn_pid_task(); }),
 swing_pid([this]{ this->swing_pid_task(); })
{
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
  //pros::Imu gyro(imu_port);
  TICK_PER_REV  = (50*(3600/CART_RPM)) * RATIO; // with no cart, the encoder reads 50 counts per rotation
  CIRCUMFERENCE = WHEEL_DIA*M_PI;
  TICK_PER_INCH = (TICK_PER_REV/CIRCUMFERENCE);

  headingPID = {11, 0, 20, 0};
  forwardDrivePID = {.45, 0, 5, 0};
  backwardDrivePID = {.45, 0, 5, 0};
  turnPID = {5, 0.003, 35, 15};
  swingPID = {12, 0, 35, 0};
  leftPID = {.45, 0, 5, 0};
  rightPID = {.45, 0, 5, 0};

  set_exit_condition(turn_exit,  100, 3,  500, 7,   500);
  set_exit_condition(swing_exit, 100, 3,  500, 7,   500);
  set_exit_condition(drive_exit, 80,  50, 300, 150, 500);
}
void Drive::set_curve_default(int left, int right)
{
  left_curve_scale = left;
  right_curve_scale = right;
}
void Drive::SetPIDConstants(PID pid, double kP, double kI, double kD, double startI)
{
  pid.SetConstants(kP, kI, kD, startI);
}

void
Drive::set_tank(int l, int r) {

  for (auto i : LeftMotors) {
    i.move_voltage(l*(12000.0/127.0));
  }
  for (auto i : RightMotors) {
    i.move_voltage(r*(12000.0/127.0));
  }
}

void
Drive::init_curve_sd() {
  // If no SD card, return
  if (!ez::util::IS_SD_CARD) return;

  // Set Starting Curve to SD Card
  // Left Curve
  FILE* l_usd_file_read = fopen("/usd/left_curve.txt", "r");
  char l_buf[5];
  fread(l_buf, 1, 5, l_usd_file_read);
  left_curve_scale = std::stof(l_buf);
  fclose(l_usd_file_read);

  // Right Curve
  FILE* r_usd_file_read = fopen("/usd/right_curve.txt", "r");
  char r_buf[5];
  fread(r_buf, 1, 5, r_usd_file_read);
  right_curve_scale = std::stof(r_buf);
  fclose(r_usd_file_read);
}

// Save New left Curve
void
Drive::save_l_curve_sd() {
  // If no SD card, return
  if (!ez::util::IS_SD_CARD) return;

  FILE* usd_file_write = fopen("/usd/left_curve.txt", "w");
  std::string in_str = std::to_string(left_curve_scale);
  char const *in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}
void
Drive::save_r_curve_sd() {
  // If no SD card, return
  if (!ez::util::IS_SD_CARD) return;

  FILE* usd_file_write = fopen("/usd/right_curve.txt", "w");
  std::string in_str = std::to_string(right_curve_scale);
  char const *in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}

void Drive::l_increase() {
  left_curve_scale += 0.1;
}
 void Drive::l_decrease() {
  left_curve_scale -= 0.1;
  left_curve_scale =  left_curve_scale<0 ? 0 : left_curve_scale;
}
void Drive::r_increase() {
  right_curve_scale += 0.1;
}
void Drive::r_decrease() {
  right_curve_scale -= 0.1;
  right_curve_scale =  right_curve_scale<0 ? 0 : right_curve_scale;
}
void
Drive::modify_curve_with_controller() {

  button_press(&l_increase_, master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT), l_increase, save_l_curve_sd);
  button_press(&l_decrease_, master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT), l_decrease, save_l_curve_sd);
  if (!is_tank) {
    button_press(&r_increase_, master.get_digital(pros::E_CONTROLLER_DIGITAL_A), r_increase, save_r_curve_sd);
    button_press(&r_decrease_, master.get_digital(pros::E_CONTROLLER_DIGITAL_Y), r_decrease, save_r_curve_sd);
  }

  auto sf = std::to_string(right_curve_scale);
  auto st = std::to_string(left_curve_scale);
  if (!is_tank)
    master.set_text(2, 0, st+"   "+sf);
  else
    master.set_text(2, 0, st);
}
void
Drive::button_press(button_ *input_name, int button, std::function<void()> changeCurve, std::function<void()> save) {
  if (button && !input_name->lock) {
    changeCurve();
    input_name->lock = true;
    input_name->release_reset = true;
  }
  else if (button && input_name->lock) {
    input_name->hold_timer+=10;
    if (input_name->hold_timer > 500.0) {
      input_name->increase+=10;
      if (input_name->increase > 0.1) {
        changeCurve();
        input_name->increase = 0;
      }
    }
  }
  else if (!button) {
    input_name->lock = false;
    input_name->hold_timer = 0;

    if (input_name->release_reset) {
      input_name->release_timer+=10;
      if (input_name->release_timer > 500.0/2.0) {
        save();
        input_name->release_timer = 0;
        input_name->release_reset = false;
      }
    }
  }
}


void
Drive::chassis_tank()
{
  is_tank = true;

  // Toggle for controller curve
  if (!DISABLE_CONTROLLER)
    modify_curve_with_controller();

  // Put the joysticks through the curve function
  int l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
  int r_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_Y));

  // Threshold if joysticks don't come back to perfect 0
  if (abs(l_stick)>5 || abs(r_stick)>5) {
    set_tank(l_stick, r_stick);
    reset_drive_sensor();
  }
  // When joys are released, run active brake (P) on drive
  else {
    set_tank((0-left_sensor())*ACTIVE_BRAKE_KP, (0-right_sensor())*ACTIVE_BRAKE_KP);
  }
}
void
Drive::chassis_arcade_standard(e_type t) {
  is_tank = false;

  int l_stick, r_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (t == k_split) {
    // Put the joysticks through the curve function
    l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    r_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_X));
  }
  else if (t == k_single) {
    // Put the joysticks through the curve function
    l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    r_stick = right_curve_function(master.get_analog(ANALOG_LEFT_X));
  }

  // Threshold if joysticks don't come back to perfect 0
  if (abs(l_stick)>5 || abs(r_stick)>5) {
    set_tank(l_stick+r_stick, l_stick-r_stick);
    reset_drive_sensor();
  }
  // When joys are released, run active brake (P) on drive
  else {
    set_tank((0-left_sensor())*ACTIVE_BRAKE_KP, (0-right_sensor())*ACTIVE_BRAKE_KP);
  }
}


// Arcade control standard
void
Drive::chassis_arcade_flipped(e_type t) {
  is_tank = false;

  int l_stick, r_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (t == k_split) {
    // Put the joysticks through the curve function
    r_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_Y));
    l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_X));
  }
  else if (t == k_single) {
    // Put the joysticks through the curve function
    r_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_Y));
    l_stick = left_curve_function(master.get_analog(ANALOG_RIGHT_X));
  }

  // Threshold if joysticks don't come back to perfect 0
  if (abs(l_stick)>5 || abs(r_stick)>5) {
    set_tank(r_stick+l_stick, r_stick-l_stick);
    reset_drive_sensor();
  }
  // When joys are released, run active brake (P) on drive
  else {
    set_tank((0-left_sensor())*ACTIVE_BRAKE_KP, (0-right_sensor())*ACTIVE_BRAKE_KP);
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






//!Joystick

//bool is_tank;

///
// Input Curve
///



// ! Auton

///
// Adjust Constants
///
void
Drive::set_slew_min_power(int fwd, int rev) {
  SLEW_MIN_POWER[0] = fwd;
  SLEW_MIN_POWER[1] = rev;
}

void
Drive::set_slew_distance(int fwd, int rev) {
  SLEW_DISTANCE[0] = fwd;
  SLEW_DISTANCE[1] = rev;
}

void
Drive::set_drive_pid(double target, int speed, bool slew_on, bool toggle_heading) {
  // Global setup
  set_max_speed(speed);
  bool slew = slew_on;
  HEADING_ON = toggle_heading;
  bool isBackwards = false;

  double l_target_encoder, r_target_encoder;

  // If drive or line, set targets to drive

    printf("Drive Started... Target Value: %f\n", target);
    double l_start = left_sensor();
    double r_start = right_sensor();
    l_target_encoder = l_start + (target*TICK_PER_INCH);
    r_target_encoder = r_start + (target*TICK_PER_INCH);
    if (target<l_start && target<r_start) {
      isBackwards = true;
      auto consts = backwardDrivePID.GetConstants();
      leftPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);
      rightPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);

    } else {
      auto consts = forwardDrivePID.GetConstants();
      leftPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);
      rightPID.SetConstants(consts.kP, consts.kI, consts.kD, consts.StartI);
      forwardDrivePID.SetTarget(target);
      isBackwards = false;
    }

    leftPID.SetTarget(l_target_encoder);
    rightPID.SetTarget(r_target_encoder);

    l.sign = ez::util::sgn(l_target_encoder-left_sensor());
    r.sign = ez::util::sgn(r_target_encoder-right_sensor());

    l.x_intercept = l_start + (SLEW_DISTANCE[isBackwards]*TICK_PER_INCH);
    r.x_intercept = r_start + (SLEW_DISTANCE[isBackwards]*TICK_PER_INCH);

    l.y_intercept = max_speed * l.sign;
    r.y_intercept = max_speed * r.sign;

    l.slope = (SLEW_MIN_POWER[isBackwards]-max_speed) / ((l_start+(SLEW_DISTANCE[isBackwards]*TICK_PER_INCH))-0);
    r.slope = (SLEW_MIN_POWER[isBackwards]-max_speed) / ((l_start+(SLEW_DISTANCE[isBackwards]*TICK_PER_INCH))-0);

    turn_pid.suspend();
    swing_pid.suspend();
    drive_pid.resume();
}
void
Drive::set_turn_pid(double target, int speed, bool slew_on)
{
  printf("Turn Started... Target Value: %f\n", target);
  turnPID.SetTarget(target);
  swing_pid.suspend();
  drive_pid.suspend();
  turn_pid.resume();
  set_max_speed(speed);

  //gyro_sign = sgn(target - get_gyro());
}

void
Drive::set_swing_pid(double target, int speed, bool slew_on)
{
  printf("Swing Started... Target Value: %f\n", target);
  swingPID.SetTarget(target);
  set_max_speed(speed);
  //swing_sign = sgn(target-get_gyro());
}
///
// Reset to default constants
///



// "Enumerator" for drive type

bool heading_on = false;
float l_target_encoder, r_target_encoder;
float l_start, r_start;
int max_speed = 0;
float gyro_target = 0;

// Slew variables
int l_x_intercept, r_x_intercept;
int l_y_intercept, r_y_intercept;
int l_sign, r_sign, gyro_sign;
float l_slew_error, r_slew_error;
float l_slope, r_slope;
bool slew = false;

// Swing variables
int swing_sign = 0;
bool stop = false;

// Drive PID with active straight code
// - it makes sure the angle of the robot is what it should be all the way through the movements,
// - turning if needed to keep it going straight
void
Drive::drive_pid_task() {
  while (true) {

    pros::delay(10);
  }
}

void
Drive::turn_pid_task() {
  while (true) {

    pros::delay(10);
  }
}

void
Drive::swing_pid_task() {
  while (true) {

    pros::delay(10);
  }
}


void Drive::set_exit_condition(exit_condition_ type, int p_small_exit_time, int p_small_error, int p_big_exit_time, int p_big_error, int p_velocity_exit_time) {
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

  // if (drive task running) {
    while (drive_exit_condition(leftPID.GetTarget(), rightPID.GetTarget())) {
      pros::delay(delay_time);
    }
  // }
  // else if (turn task running) {
    while (turn_exit_condition(turnPID.GetTarget())) {
      pros::delay(delay_time);
    }
  // }
  // else if (swing task running) {
    while (swing_exit_condition(swingPID.GetTarget())) {
      pros::delay(delay_time);
    }
  // }
}
// Function to wait until a certain position is reached
void Drive::wait_until(double target) {
  const int delay_time = ez::util::DELAY_TIME;

  // If robot is driving...
  if (true/*drive task running*/) { // CHANGE THIS TO DETECT IF TASK IS RUNNING
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
        printf("Drive Wait Until Completed- Error Sgn Flipped\n");
        run = false;
      }
      else if (!drive_exit_condition(l_tar, r_tar)) {
        run = false;
      }

      pros::delay(delay_time);
    }
  }

  // If robot is turning...
  else if (true /*turning or swinging is running*/) { // CHANGE THIS TO CHECK FOR TASK RUNNING!
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
