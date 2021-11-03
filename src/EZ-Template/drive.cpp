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
 : gyro (imu_port), master(pros::E_CONTROLLER_MASTER), drive_pid(drive_pid_task, nullptr, "Drive Task"),
 turn_pid(turn_pid_task, nullptr, "Turn Task"), swing_pid(swing_pid_task, nullptr, "Swing Task")
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

// Slew
// Variables that are arrays mean the first variable is for forward and the second is for backward
int SLEW_MIN_POWER[2] = {FW_SLEW_MIN_POWER, BW_SLEW_MIN_POWER}; // Starting speed for the slew
int SLEW_DISTANCE [2] = {FW_SLEW_DISTANCE, BW_SLEW_DISTANCE};  // Distance the robot slews at before reaching max speed

const bool DEBUG = false;

const int FORWARD  = 0;
const int BACKWARD = 1;

// Forward Drive Constants
float fw_drive_kp = FW_DRIVE_KP;
float fw_drive_kd = FW_DRIVE_KD;

// Backward Drive Constants
float bw_drive_kp = BW_DRIVE_KP;
float bw_drive_kd = BW_DRIVE_KD;

// Minimum speed for driving and error to stop within
// if spede goes below min_speed, robot travels at min_speed until it gets within min_error, where motors go 0
int min_speed = MIN_SPEED;
int min_error = MIN_ERROR;

// Heading Constants (uses imu to keep robot driving straight)
float heading_kp = HEADING_KP;
float heading_kd = HEADING_KD;

// Turn Constants
float gyro_kp = GYRO_KP;
float gyro_ki = GYRO_KI;
float gyro_kd = GYRO_KD;
int   start_i = START_I; // Start I when error is this
int clipped_turn_i_speed = CLIPPED_TURN_I_SPEED; // When I engages, this becomes max power

// Swing Constants
float swing_kp = SWING_KP;
float swing_kd = SWING_KD;

float drive_constant[2][2] = {
  {fw_drive_kp, fw_drive_kd}, // Foward KP, KD
  {bw_drive_kp, bw_drive_kd}  // Backward KP, KD
};
int direction;

///
// Adjust Constants
///
void
Drive::set_slew_min_power(int fw, int bw) {
  SLEW_MIN_POWER[FORWARD]  = fw;
  SLEW_MIN_POWER[BACKWARD] = bw;
}

void
Drive::set_slew_distance(int fw, int bw) {
  SLEW_DISTANCE[FORWARD]  = fw;
  SLEW_DISTANCE[BACKWARD] = bw;
}

void
Drive::set_fw_drive_constants(float kp, float kd) {
  drive_constant[FORWARD][0] = kp;
  drive_constant[FORWARD][1] = kd;
}

void
Drive::set_bw_drive_constants(float kp, float kd) {
  drive_constant[BACKWARD][0] = kp;
  drive_constant[BACKWARD][1] = kd;
}

void
Drive::set_heading_constants(float kp, float kd) {
  heading_kp = kp;
  heading_kd = kd;
}

void
Drive::set_turn_constants(float kp, float ki, float kd) {
  gyro_kp = kp;
  gyro_ki = ki;
  gyro_kd = kd;
}

void
Drive::set_turn_i_constants(float starting, int clipping) {
  start_i = starting;
  clipped_turn_i_speed = clipping;
}

void
Drive::set_swing_constants(float kp, float kd) {
  swing_kp = kp;
  swing_kd = kd;
}
void
Drive::set_drive_pid(double target, int speed, bool slew_on, bool toggle_heading, ) {
  // Global setup
  set_max_speed(speed);
  active_drive_type = type;
  slew = slew_on;
  stop = slew;
  heading_on = toggle_heading;
  bool isBackwards = false;
  // If drive or line, set targets to drive

    printf("Drive Started... Target Value: %f\n", target);
    l_start = left_sensor();
    r_start = right_sensor();
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

    l_sign = sgn(l_target_encoder-left_sensor());
    r_sign = sgn(r_target_encoder-right_sensor());

    l_x_intercept = l_start + (SLEW_DISTANCE[isBackwards]*TICK_PER_INCH);
    r_x_intercept = r_start + (SLEW_DISTANCE[isBackwards]*TICK_PER_INCH);

    l_y_intercept = max_speed * l_sign;
    r_y_intercept = max_speed * r_sign;

    l_slope = (SLEW_MIN_POWER[isBackwards]-max_speed) / ((l_start+(SLEW_DISTANCE[isBackwards]*TICK_PER_INCH))-0);
    r_slope = (SLEW_MIN_POWER[isBackwards]-max_speed) / ((l_start+(SLEW_DISTANCE[isBackwards]*TICK_PER_INCH))-0);
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
Drive::drive_pid_task(void*) {
  float left_error, right_error, gyro_error;
  float last_time, last_l_error, last_r_error, last_gyro_error;
  float l_der, r_der, gyro_der;
  float gyro_integral;
  float right_output, left_output, gyro_output;
  int l_output, r_output;
  bool slow_turn = false;
  while (true) {

    // Math for P
    left_error    = l_target_encoder - left_sensor();
    right_error   = r_target_encoder - right_sensor();
    gyro_error    = gyro_target      - get_gyro();

    // Math for D
    l_der    = left_error  - last_l_error;
    r_der    = right_error - last_r_error;
    gyro_der = gyro_error  - last_gyro_error;

    // Math for I
    if (fabs(gyro_error)<start_i/* && sgn(gyro_error)==gyro_sign*/) {
      slow_turn = true;
      gyro_integral = gyro_integral + gyro_error;
    } else {
      slow_turn = false;
      gyro_integral = 0;
    }

    // Combing P I D
    left_output    = (left_error *drive_constant[direction][0])  + (l_der*drive_constant[direction][1]);
    right_output   = (right_error*drive_constant[direction][0])  + (r_der*drive_constant[direction][1]);
    // Different kP, kI and kD are used for turning, heading and swings
    if (active_drive_type==drive)
      gyro_output = (gyro_error*heading_kp) + (gyro_der*heading_kd);
    else if (active_drive_type==turn)
      gyro_output = (gyro_error*gyro_kp) + (gyro_integral*gyro_ki) + (gyro_der*gyro_kd);
    else if (active_drive_type==l_swing || active_drive_type==r_swing)
      gyro_output = (gyro_error*swing_kp) + (gyro_der*swing_kd);

    // If enabled, slew the drive at the begining so the robot doesn't wheelie
    if (slew) {
      // Error for distance it needs to trigger this code
      l_slew_error = l_x_intercept - left_sensor();
      r_slew_error = r_x_intercept - right_sensor();
      if (active_drive_type==drive) {
        // y=mx+b using everything calculated, where x is error
        if (sgn(l_slew_error) == l_sign) {
          left_output  = (l_slope * l_slew_error) + l_y_intercept;
        }
        if (sgn(r_slew_error) == r_sign) {
          right_output = (r_slope * r_slew_error) + r_y_intercept;
        }
        if (sgn(r_slew_error)!=r_sign && sgn(l_slew_error)!=l_sign) {
          slew = false;
        }
      }
    }

    // Clip the speeds to be slower
    left_output  = clip_num(left_output,  max_speed, -max_speed);
    right_output = clip_num(right_output, max_speed, -max_speed);
    if (active_drive_type==turn) {
      if (!slow_turn)
        gyro_output = clip_num(gyro_output, max_speed, -max_speed);
      else
        gyro_output = clip_num(gyro_output, clipped_turn_i_speed, -clipped_turn_i_speed);
    }
    else if (active_drive_type==l_swing || active_drive_type==r_swing) {
      gyro_output = clip_num(gyro_output, max_speed, -max_speed);
    }

    // Set drive based on drive type
    if (active_drive_type == drive) {
      if (heading_on) {
        l_output = left_output;
        r_output = right_output;
      } else {
        l_output = left_output  + gyro_output;
        r_output = right_output - gyro_output;
      }

      // Setting drive to min_speed
      if (!slew) {
        if (abs(l_output)<min_speed) {
          if (fabs(left_error)>min_error)
            l_output = min_speed * sgn(left_error);
          else
            l_output = 0;
        }
        if (abs(r_output)<min_speed) {
          if (fabs(right_error)>min_error)
            r_output = min_speed * sgn(right_error);
          else
            r_output = 0;
        }
      }
    }

    // Turn
    else if (active_drive_type == turn) {
      l_output =  gyro_output;
      r_output = -gyro_output;
    }

    // L Swing
    else if (active_drive_type == l_swing) {
      l_output = gyro_output;
      r_output = 0;
    }

    // R Swing
    else if (active_drive_type == r_swing) {
      r_output = -gyro_output;
      l_output = 0;
    }

    // Don't run motors in the first 1500 the program is on
    // (while IMU is calibrating)
    if (pros::millis()<1500) {
      set_tank(0, 0);
    } else {
      set_tank(l_output, r_output);
    }

    if (DEBUG && pros::millis()>1500) {
      if (active_drive_type == drive)  printf("le: %f   re: %f   l_der %f\n", left_error, right_error, l_der*drive_constant[direction][1]);
      if (active_drive_type==turn || active_drive_type==l_swing || active_drive_type==r_swing)   printf("output: %f   error: %f   p: %f   i: %f   d: %f\n", gyro_output, gyro_error, gyro_error*gyro_kp, gyro_integral*gyro_ki, gyro_der*gyro_kd);
    }

    last_time       = pros::millis();
    last_l_error    = left_error;
    last_r_error    = right_error;
    last_gyro_error = gyro_error;

    pros::delay(10);
  }
}




bool
Drive::drive_exit_condition(float l_target, float r_target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, int delay_time = 10, bool wait_until = false) {
  static int i = 0, j = 0, k = 0, g = 0;

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (fabs(l_target-left_sensor())<start_small_counter_within && fabs(r_target-right_sensor())<start_small_counter_within) {
    j+=delay_time;
    //printf("\nJ: %i", j/10);

    if (j>small_timeout) {
      if (!wait_until) printf("Drive Timed Out");
      else             printf("Drive Wait Until Timed Out");
      printf(" - Small Thresh\n");
      return false;
    }
  }
  else {
    j = 0;
  }

  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.
  if (fabs(l_target-left_sensor())<start_big_counter_within && fabs(r_target-right_sensor())<start_big_counter_within) {
    i+=delay_time;
    //printf("\nI: %i", i/10);

    if (i>big_timeout) {
      if (!wait_until) printf("Drive Timed Out");
      else             printf("Drive Wait Until Timed Out");
      printf(" - Big Thresh\n");
      return false;
    }
  }
  else {
    i = 0;
  }

  if (right_velocity()==0 && left_velocity()==0) {
    k+=delay_time;
    //printf("\nI: %i", i/10);

    if (k>velocity_timeout) {
      if (!wait_until) printf("Drive Timed Out");
      else             printf("Drive Wait Until Timed Out");
      printf(" - Velocity 0\n");
      return false;
    }
  }
  else {
    k = 0;
  }

  return true;
}

bool
Drive::turn_exit_condition(double target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, int delay_time = 10, bool wait_until = false) {
  static int i, j, k;

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (fabs(target-get_gyro())<start_small_counter_within) {
    j+=delay_time;
    //printf("\nJ: %i", j/10);

    if (j>small_timeout) {
      if (!wait_until) printf("Turn Timed Out");
      else             printf("Turn Wait Until Timed Out");
      printf(" - Small Thresh\n");
      return false;
    }
  }
  else {
    j = 0;
  }
  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.
  if (fabs(target-get_gyro())<start_big_counter_within) {
    i+=delay_time;
    //printf("\nI: %i", i/10);

    if (i>big_timeout) {
      if (!wait_until) printf("Turn Timed Out");
      else             printf("Turn Wait Until Timed Out");
      printf(" - Big Thresh\n");
      return false;
    }
  }
  else {
    i = 0;
  }

  if (right_velocity()==0 && left_velocity()==0) {
    k+=delay_time;
    //printf("\nI: %i", i/10);

    if (k>velocity_timeout) {
      if (!wait_until) printf("Turn Timed Out");
      else             printf("Turn Wait Until Timed Out");
      printf(" - Velocity 0\n");
      return false;
    }
  }
  else {
    k = 0;
  }
  return true;
}

//Wait for drive
void
Drive::wait_drive(float l_target, float r_target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, int delay_time = 10, bool wait_until = false) {
  pros::delay(delay_time);

  // Parameters for exit condition function:
  // #3 - time the robot has to be within #2 of target
  // #4 - threshold for timer to start
  // #5 - time for if position is never reached
  // #6 - position for robot to be within to never reach target
  // #7 - velocity timeout

    while (drive_exit_condition(l_target_encoder, r_target_encoder, small_timeout, start_small_counter_within, big_timeout, start_big_counter_within, velocity_timeout, delay_time, wait_until)) {
      pros::delay(delay_time);
    }
  }
}
void
Drive::wait_turn(double target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, int delay_time = 10, bool wait_until = false)
{
  while (turn_exit_condition(target, small_timeout, start_small_counter_within, big_timeout, start_big_counter_within, velocity_timeout, delay_time, wait_until)) {
    pros::delay(delay_time);
  }

}
void
Drive::wait_swing(double target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, int delay_time = 10, bool wait_until = false)
{
  while (turn_exit_condition(target, small_timeout, start_small_counter_within, big_timeout, start_big_counter_within, velocity_timeout, delay_time, wait_until)) {
    pros::delay(delay_time);
  }
}
// Function to wait until a certain position is reached
void
Drive::wait_until_drive(double input, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, int delay_time = 10) {
  // If robot is driving...
    // Calculate error between current and target (target needs to be an inbetween position)
    int l_tar   = l_start + (input*TICK_PER_INCH);
    int r_tar   = r_start + (input*TICK_PER_INCH);
    int l_error = l_tar   - left_sensor();
    int r_error = r_tar   - right_sensor();
    int l_sgn   = sgn(l_error);
    int r_sgn   = sgn(r_error);
    bool run    = true;

    while (run) {
      l_error = l_tar - left_sensor();
      r_error = r_tar - right_sensor();

      // Break the loop once target is passed
      if (sgn(l_error)==l_sgn && sgn(r_error)==r_sgn) {
        run = true; // this makes sure that the following else if is rnu after the sgn is flipped
      }
      else if (sgn(l_error)!=l_sgn && sgn(r_error)!=r_sgn) {
        printf("Drive Wait Until Completed- Error Sgn Flipped\n");
        run = false;
      }
      else if (!drive_exit_condition(l_tar, r_tar, small_timeout, start_small_counter_within, big_timeout, start_big_counter_within, velocity_timeout, delay_time, true)) {
        run = false;
      }

      pros::delay(delay_time);
    }
}
void
Drive::wait_until_turn(double input, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, int delay_time = 10)
{
  int g_error = input - get_gyro();
  int g_sgn   = sgn(g_error);
  bool run    = true;

  while (run) {
    g_error = input - get_gyro();

    // Break the loop once target is passed
    if (sgn(g_error)==g_sgn) {
      run = true; // this makes sure that the following else if is rnu after the sgn is flipped
    }
    else if (sgn(g_error)!=g_sgn) {
      run = false;
    }
    else if (!turn_exit_condition(input, small_timeout, start_small_counter_within, big_timeout, start_big_counter_within, velocity_timeout, delay_time, true)) {
      run = false;
    }

    pros::delay(delay_time);
  }
}
void
Drive::wait_until_swing(double input, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, int delay_time = 10)
{
  wait_until_turn(input, small_timeout, start_small_counter_within, big_timeout, start_big_counter_within, velocity_timeout, delay_time);
}
