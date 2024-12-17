/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <iostream>
#include <tuple>

#include "EZ-Template/PID.hpp"
#include "EZ-Template/slew.hpp"
#include "EZ-Template/tracking_wheel.hpp"
#include "EZ-Template/util.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QTime.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"

using namespace ez;

namespace ez {
class Drive {
 public:
  /**
   * Joysticks will return 0 when they are within this number.
   *
   * Set with opcontrol_joystick_threshold_set()
   */
  int JOYSTICK_THRESHOLD;

  /**
   * Global current brake mode.
   */
  pros::motor_brake_mode_e_t CURRENT_BRAKE = pros::E_MOTOR_BRAKE_COAST;

  /**
   * Global current mA.
   */
  int CURRENT_MA = 2500;

  /**
   * Current swing type.
   */
  e_swing current_swing;

  /**
   * Vector of pros motors for the left chassis.
   */
  std::vector<pros::Motor> left_motors;

  /**
   * Vector of pros motors for the right chassis.
   */
  std::vector<pros::Motor> right_motors;

  /**
   * Vector of pros motors that are disconnected from the drive.
   */
  std::vector<int> pto_active;

  /**
   * Inertial sensor.
   */
  pros::Imu imu;

  /**
   * Deprecated left tracking wheel.
   */
  pros::adi::Encoder left_tracker;

  /**
   * Deprecated right tracking wheel.
   */
  pros::adi::Encoder right_tracker;

  /**
   * Deprecated left rotation tracker.
   */
  pros::Rotation left_rotation;

  /**
   * Deprecated right rotation tracker.
   */
  pros::Rotation right_rotation;

  /**
   * Left vertical tracking wheel.
   */
  tracking_wheel* odom_tracker_left;

  /**
   * Right vertical tracking wheel.
   */
  tracking_wheel* odom_tracker_right;

  /**
   * Front horizontal tracking wheel.
   */
  tracking_wheel* odom_tracker_front;

  /**
   * Back horizontal tracking wheel.
   */
  tracking_wheel* odom_tracker_back;

  /**
   * PID objects.
   */
  PID headingPID;
  PID turnPID;
  PID leftPID;
  PID rightPID;
  PID forward_drivePID;
  PID backward_drivePID;
  PID fwd_rev_drivePID;
  PID swingPID;
  PID forward_swingPID;
  PID backward_swingPID;
  PID fwd_rev_swingPID;
  PID xyPID;
  PID current_a_odomPID;
  PID boomerangPID;
  PID odom_angularPID;
  PID internal_leftPID;
  PID internal_rightPID;
  PID left_activebrakePID;
  PID right_activebrakePID;

  /**
   * Slew objects.
   */
  ez::slew slew_left;
  ez::slew slew_right;
  ez::slew slew_forward;
  ez::slew slew_backward;
  ez::slew slew_turn;
  ez::slew slew_swing_forward;
  ez::slew slew_swing_backward;
  ez::slew slew_swing;

  /**
   * Sets constants for slew for swing movements.
   *
   * Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed, an okapi distance unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_swing_constants_set(okapi::QLength distance, int min_speed);

  /**
   * Sets constants for slew for forward swing movements.
   *
   * Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed, an okapi distance unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_swing_constants_forward_set(okapi::QLength distance, int min_speed);

  /**
   * Sets constants for slew for backward swing movements.
   *
   * Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed, an okapi distance unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_swing_constants_backward_set(okapi::QLength distance, int min_speed);

  /**
   * Sets constants for slew for swing movements.
   *
   * Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed, an okapi angle unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_swing_constants_set(okapi::QAngle distance, int min_speed);

  /**
   * Sets constants for slew for swing forward movements.
   *
   * Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed, an okapi angle unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_swing_constants_forward_set(okapi::QAngle distance, int min_speed);

  /**
   * Sets constants for slew for swing backward movements.
   *
   * Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed, an okapi angle unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_swing_constants_backward_set(okapi::QAngle distance, int min_speed);

  /**
   * Sets constants for slew for turns.
   *
   * Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed, an okapi angle unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_turn_constants_set(okapi::QAngle distance, int min_speed);

  /**
   * Sets constants for slew for driving forward.
   *
   * Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed, an okapi distance unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_drive_constants_forward_set(okapi::QLength distance, int min_speed);

  /**
   * Sets constants for slew for driving backward.
   *
   * Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed, an okapi distance unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_drive_constants_backward_set(okapi::QLength distance, int min_speed);

  /**
   * Sets constants for slew for driving.
   *
   * Slew ramps up the speed of the robot until the set distance is traveled.
   *
   * \param distance
   *        the distance the robot travels before reaching max speed, an okapi distance unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_drive_constants_set(okapi::QLength distance, int min_speed);

  /**
   * Sets the default slew for drive forwards and backwards motions, can be overwritten in movement functions.
   *
   * \param slew_on
   *        true enables, false disables
   */
  void slew_drive_set(bool slew_on);

  /**
   * Sets the default slew for drive forward motions, can be overwritten in movement functions.
   *
   * \param slew_on
   *        true enables, false disables
   */
  void slew_drive_forward_set(bool slew_on);

  /**
   * Returns true if slew is enabled for all drive forward movements, false otherwise.
   */
  bool slew_drive_forward_get();

  /**
   * Sets the default slew for drive backward motions, can be overwritten in movement functions.
   *
   * \param slew_on
   *        true enables, false disables
   */
  void slew_drive_backward_set(bool slew_on);

  /**
   * Returns true if slew is enabled for all drive backward movements, false otherwise.
   */
  bool slew_drive_backward_get();

  /**
   * Sets the default slew for swing forward and backward motions, can be overwritten in movement functions.
   *
   * \param slew_on
   *        true enables, false disables
   */
  void slew_swing_set(bool slew_on);

  /**
   * Sets the default slew for swing forward motions, can be overwritten in movement functions.
   *
   * \param slew_on
   *        true enables, false disables
   */
  void slew_swing_forward_set(bool slew_on);

  /**
   * Returns true if slew is enabled for all swing forward motions, false otherwise.
   */
  bool slew_swing_forward_get();

  /**
   * Sets the default slew for swing backward motions, can be overwritten in movement functions.
   *
   * \param slew_on
   *        true enables, false disables
   */
  void slew_swing_backward_set(bool slew_on);

  /**
   * Returns true if slew is enabled for all swing backward motions, false otherwise.
   */
  bool slew_swing_backward_get();

  /**
   * Sets the default slew for turn motions, can be overwritten in movement functions.
   *
   * \param slew_on
   *        true enables, false disables
   */
  void slew_turn_set(bool slew_on);

  /**
   * Returns true if slew is enabled for all turn motions, false otherwise.
   */
  bool slew_turn_get();

  /**
   * Allows slew to reenable when the new input speed is larger than the current speed during pure pursuits.
   *
   * \param slew_on
   *        true enables, false disables
   */
  void slew_odom_reenable(bool reenable);

  /**
   * Returns if slew will reenable when the new input speed is larger than the current speed during pure pursuits.
   */
  bool slew_odom_reenabled();

  /**
   * Current mode of the drive.
   */
  e_mode mode;

  /**
   * Sets current mode of drive.
   *
   * \param p_mode
   *        the new drive mode
   * \param stop_drive
   *        if the drive will stop when p_mode is DISABLED
   */
  void drive_mode_set(e_mode p_mode, bool stop_drive = true);

  /**
   * Returns current mode of drive.
   */
  e_mode drive_mode_get();

  /**
   * Calibrates imu and initializes sd card to curve.
   */
  void initialize();

  /**
   * Tasks for autonomous.
   */
  pros::Task ez_auto;

  /**
   * Creates a Drive Controller using internal encoders.
   *
   * \param left_motor_ports
   *        input {1, -2...}. make ports negative if reversed
   * \param right_motor_ports
   *        input {-3, 4...}. make ports negative if reversed
   * \param imu_port
   *        port the IMU is plugged into
   * \param wheel_diameter
   *        diameter of your drive wheels
   * \param ticks
   *        motor cartridge RPM
   * \param ratio
   *        external gear ratio, wheel gear / motor gear
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio = 1.0);

  /**
   * Creates a Drive Controller using encoders plugged into the brain.
   *
   * \param left_motor_ports
   *        input {1, -2...}. make ports negative if reversed
   * \param right_motor_ports
   *        input {-3, 4...}. make ports negative if reversed
   * \param imu_port
   *        port the IMU is plugged into
   * \param wheel_diameter
   *        diameter of your sensored wheel
   * \param ticks
   *        ticks per revolution of your encoder
   * \param ratio
   *        external gear ratio, wheel gear / sensor gear
   * \param left_tracker_ports
   *        input {1, 2}. make ports negative if reversed
   * \param right_tracker_ports
   *        input {3, 4}. make ports negative if reversed
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports) __attribute__((deprecated("Use the integrated encoder constructor with odom_tracker_left_set() and odom_tracker_right_set() instead!")));

  /**
   * Creates a Drive Controller using encoders plugged into a 3 wire expander.
   *
   * \param left_motor_ports
   *        input {1, -2...}. make ports negative if reversed
   * \param right_motor_ports
   *        input {-3, 4...}. make ports negative if reversed
   * \param imu_port
   *        port the IMU is plugged into
   * \param wheel_diameter
   *        diameter of your sensored wheel
   * \param ticks
   *        ticks per revolution of your encoder
   * \param ratio
   *        external gear ratio, wheel gear / sensor gear
   * \param left_tracker_ports
   *        input {1, 2}. make ports negative if reversed
   * \param right_tracker_ports
   *        input {3, 4}. make ports negative if reversed
   * \param expander_smart_port
   *        port the expander is plugged into
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports, int expander_smart_port) __attribute__((deprecated("Use the integrated encoder constructor with odom_tracker_left_set() and odom_tracker_right_set() instead!")));

  /**
   * Creates a Drive Controller using rotation sensors.
   *
   * \param left_motor_ports
   *        input {1, -2...}. make ports negative if reversed
   * \param right_motor_ports
   *        input {-3, 4...}. make ports negative if reversed
   * \param imu_port
   *        port the IMU is plugged into
   * \param wheel_diameter
   *        diameter of your sensored wheel
   * \param ratio
   *        external gear ratio, wheel gear / sensor gear
   * \param left_tracker_port
   *        make ports negative if reversed
   * \param right_tracker_port
   *        make ports negative if reversed
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ratio, int left_rotation_port, int right_rotation_port) __attribute__((deprecated("Use the integrated encoder constructor with odom_tracker_left_set() and odom_tracker_right_set() instead!")));

  /**
   * Sets drive defaults.
   */
  void drive_defaults_set();

  /////
  //
  // General Odometry
  //
  /////

  /**
   * Tasks for tracking.
   */
  void ez_tracking_task();

  /**
   * Enables / disables tracking.
   *
   * \param input
   *        true enables tracking, false disables tracking
   */
  void odom_enable(bool input);

  /**
   * Returns whether the bot is tracking with odometry.
   *
   * True means tracking is enabled, false means tracking is disabled.
   */
  bool odom_enabled();

  /**
   * Sets the width of the drive.
   *
   * This is used for tracking.
   *
   * \param input
   *        a unit in inches, from center of the wheel to center of the wheel
   */
  void drive_width_set(double input);

  /**
   * Sets the width of the drive.
   *
   * This is used for tracking.
   *
   * \param input
   *        an okapi unit, from center of the wheel to center of the wheel
   */
  void drive_width_set(okapi::QLength p_input);

  /**
   * Returns the width of the drive
   */
  double drive_width_get();

  /**
   * Sets the current X coordinate of the robot.
   *
   * \param x
   *        new x coordinate in inches
   */
  void odom_x_set(double x);

  /**
   * Sets the current X coordinate of the robot.
   *
   * \param p_x
   *        new x coordinate as an okapi unit
   */
  void odom_x_set(okapi::QLength p_x);

  /**
   * Returns the current X coordinate of the robot in inches.
   */
  double odom_x_get();

  /**
   * Sets the current Y coordinate of the robot.
   *
   * \param y
   *        new y coordinate in inches
   */
  void odom_y_set(double y);

  /**
   * Sets the current Y coordinate of the robot.
   *
   * \param p_y
   *        new y coordinate as an okapi unit
   */
  void odom_y_set(okapi::QLength p_y);

  /**
   * Returns the current Y coordinate of the robot in inches.
   */
  double odom_y_get();

  /**
   * Sets the current angle of the robot.
   *
   * \param a
   *        new angle in degrees
   */
  void odom_theta_set(double a);

  /**
   * Sets the current Theta of the robot.
   *
   * \param p_a
   *        new angle as an okapi unit
   */
  void odom_theta_set(okapi::QAngle p_a);

  /**
   * Returns the current Theta of the robot in degrees.
   */
  double odom_theta_get();

  /**
   * Sets the current pose of the robot.
   *
   * \param itarget
   *        {x, y, t} units in inches and degrees
   */
  void odom_pose_set(pose itarget);

  /**
   * Sets the current pose of the robot.
   *
   * \param itarget
   *        {x, y, t} as an okapi unit
   */
  void odom_pose_set(united_pose itarget);

  /**
   * Sets the current X and Y coordinate for the robot.
   *
   * \param x
   *        new x value, in inches
   * \param y
   *        new y value, in inches
   */
  void odom_xy_set(double x, double y);

  /**
   * Sets the current X and Y coordinate for the robot.
   *
   * \param p_x
   *        new x value, okapi unit
   * \param p_y
   *        new y value, okapi unit
   */
  void odom_xy_set(okapi::QLength p_x, okapi::QLength p_y);

  /**
   * Sets the current X, Y, and Theta values for the robot.
   *
   * \param x
   *        new x value, in inches
   * \param y
   *        new y value, in inches
   * \param t
   *        new theta value, in degrees
   */
  void odom_xyt_set(double x, double y, double t);

  /**
   * Sets the current X, Y, and Theta values for the robot.
   *
   * \param p_x
   *        new x value, okapi unit
   * \param p_y
   *        new y value, okapi unit
   * \param p_t
   *        new theta value, okapi unit
   */
  void odom_xyt_set(okapi::QLength p_x, okapi::QLength p_y, okapi::QAngle p_t);

  /**
   * Returns the current pose of the robot.
   */
  pose odom_pose_get();

  /**
   * Resets xyt to 0.
   */
  void odom_reset();

  /**
   * Flips the X axis.
   *
   * \param flip
   *        true means left is positive x, false means right is positive x
   */
  void odom_x_flip(bool flip = true);

  /**
   * Checks if X axis is flipped.
   *
   * True means left is positive X, false means right is positive X.
   */
  bool odom_x_direction_get();

  /**
   * Flips the Y axis.
   *
   * \param flip
   *        true means down is positive y, false means up is positive y
   */
  void odom_y_flip(bool flip = true);

  /**
   * Checks if Y axis is flipped.
   *
   * True means down is positive Y, false means up is positive Y.
   */
  bool odom_y_direction_get();

  /**
   * Flips the rotation axis.
   *
   * \param flip
   *        true means counterclockwise is positive, false means clockwise is positive
   */
  void odom_theta_flip(bool flip = true);

  /**
   * Checks if the rotation axis is flipped.
   *
   * True means counterclockwise is positive, false means clockwise is positive.
   */
  bool odom_theta_direction_get();

  /**
   * Sets a new dlead.
   *
   * Dlead is a proportional value of how much to make the robot curve during boomerang motions.
   *
   * \param input
   *        a value between 0 and 1
   */
  void odom_boomerang_dlead_set(double input);

  /**
   * Returns the current dlead.
   */
  double odom_boomerang_dlead_get();

  /**
   * Sets how far away the carrot point can be from the target point.
   *
   * \param distance
   *        distance in inches
   */
  void odom_boomerang_distance_set(double distance);

  /**
   * Sets how far away the carrot point can be from the target point.
   *
   * \param distance
   *        distance as an okapi unit
   */
  void odom_boomerang_distance_set(okapi::QLength p_distance);

  /**
   * Returns how far away the carrot point can be from target.
   */
  double odom_boomerang_distance_get();

  /**
   * A proportion of how prioritized turning is during odometry motions.
   *
   * Turning is prioritized so the robot "applies brakes" while turning.  Lower number means more braking.
   *
   * \param bias
   *        a number between 0 and 1
   */
  void odom_turn_bias_set(double bias);

  /**
   * Returns the proportion of how prioritized turning is during odometry motions.
   */
  double odom_turn_bias_get();

  /**
   * Sets the spacing between points when points get injected into the path.
   *
   * \param spacing
   *        a small number in inches
   */
  void odom_path_spacing_set(double spacing);

  /**
   * Sets the spacing between points when points get injected into the path.
   *
   * \param spacing
   *        a small number in okapi units
   */
  void odom_path_spacing_set(okapi::QLength p_spacing);

  /**
   * Returns the spacing between points when points get injected into the path.
   */
  double odom_path_spacing_get();

  /**
   * Sets the constants for smoothing out a path.
   *
   * Path smoothing based on https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
   *
   * \param weight_smooth
   *        how much weight to update the data
   * \param weight_data
   *        how much weight to smooth the coordinates
   * \param tolerance
   *        how much change per iteration is necessary to keep iterating
   */
  void odom_path_smooth_constants_set(double weight_smooth, double weight_data, double tolerance);

  /**
   * Returns the constants for smoothing out a path.
   *
   * In order of:
   *  - weight_smooth
   *  - weight_data
   *  - tolerance
   */
  std::vector<double> odom_path_smooth_constants_get();

  /**
   * Prints the current path the robot is following.
   */
  void odom_path_print();

  /**
   * Sets how far away the robot looks in the path during pure pursuits.
   *
   * \param distance
   *        how long the "carrot on a stick" is, in inches
   */
  void odom_look_ahead_set(double distance);

  /**
   * Sets how far away the robot looks in the path during pure pursuits.
   *
   * \param distance
   *        how long the "carrot on a stick" is, in okapi units
   */
  void odom_look_ahead_set(okapi::QLength p_distance);

  /**
   * Returns how far away the robot looks in the path during pure pursuits.
   */
  double odom_look_ahead_get();

  /**
   * Sets the parallel left tracking wheel for odometry.
   *
   * \param input
   *        an ez::tracking_wheel
   */
  void odom_tracker_left_set(tracking_wheel* input);

  /**
   * Sets the parallel right tracking wheel for odometry.
   *
   * \param input
   *        an ez::tracking_wheel
   */
  void odom_tracker_right_set(tracking_wheel* input);

  /**
   * Sets the perpendicular front tracking wheel for odometry.
   *
   * \param input
   *        an ez::tracking_wheel
   */
  void odom_tracker_front_set(tracking_wheel* input);

  /**
   * Sets the perpendicular back tracking wheel for odometry.
   *
   * \param input
   *        an ez::tracking_wheel
   */
  void odom_tracker_back_set(tracking_wheel* input);

  /**
   * Sets the default behavior for turns in odom, swinging, and turning.
   *
   * \param behavior
   *        ez::shortest, ez::longest, ez::left, ez::right, ez::raw
   */
  void pid_angle_behavior_set(e_angle_behavior behavior);

  /**
   * Sets the default behavior for turns in turning movements.
   *
   * \param behavior
   *        ez::shortest, ez::longest, ez::left, ez::right, ez::raw
   */
  void pid_turn_behavior_set(e_angle_behavior behavior);

  /**
   * Sets the default behavior for turns in swinging movements.
   *
   * \param behavior
   *        ez::shortest, ez::longest, ez::left, ez::right, ez::raw
   */
  void pid_swing_behavior_set(e_angle_behavior behavior);

  /**
   * Sets the default behavior for turns in odom turning movements.
   *
   * \param behavior
   *        ez::shortest, ez::longest, ez::left, ez::right, ez::raw
   */
  void pid_odom_behavior_set(e_angle_behavior behavior);

  /**
   * Returns the turn behavior for turns.
   */
  e_angle_behavior pid_turn_behavior_get();

  /**
   * Returns the turn behavior for swings.
   */
  e_angle_behavior pid_swing_behavior_get();

  /**
   * Returns the turn behavior for odom turns.
   */
  e_angle_behavior pid_odom_behavior_get();

  /**
   * Gives some wiggle room in shortest vs longest, so a 180.1 and 179.9 degree turns have consistent behavior.
   *
   * \param p_tolerance
   *        angle wiggle room, an okapi unit
   */
  void pid_angle_behavior_tolerance_set(okapi::QAngle p_tolerance);

  /**
   * Gives some wiggle room in shortest vs longest, so a 180.1 and 179.9 degree turns have consistent behavior.
   *
   * \param p_tolerance
   *        angle wiggle room, in degrees
   */
  void pid_angle_behavior_tolerance_set(double tolerance);

  /**
   * Returns the wiggle room in shortest vs longest, so a 180.1 and 179.9 degree turns have consistent behavior.
   */
  double pid_angle_behavior_tolerance_get();

  /**
   * When a turn is within its tolerance, you can have it bias left or right.
   *
   * \param behavior
   *        ez::left or ez::right
   */
  void pid_angle_behavior_bias_set(e_angle_behavior behavior);

  /**
   * Returns the behavior when a turn is within its tolerance, you can have it bias left or right.
   */
  e_angle_behavior pid_angle_behavior_bias_get();

  /////
  //
  // User Control
  //
  /////

  /**
   * Sets the chassis to controller joysticks using tank control.
   * Run in usercontrol.
   *
   * This passes the controller through the curve functions, but is disabled by default.
   * Use opcontrol_curve_buttons_toggle() to enable it.
   */
  void opcontrol_tank();

  /**
   * Sets the chassis to controller joysticks using standard arcade control, where left stick is fwd/rev.
   * Run in usercontrol.
   *
   * This passes the controller through the curve functions, but is disabled by default.
   * Use opcontrol_curve_buttons_toggle() to enable it.
   *
   * \param stick_type
   *        ez::SINGLE or ez::SPLIT control
   */
  void opcontrol_arcade_standard(e_type stick_type);

  /**
   * Sets the chassis to controller joysticks using flipped arcade control, where right stick is fwd/rev.
   * Run in usercontrol.
   *
   * This passes the controller through the curve functions, but is disabled by default.
   * Use opcontrol_curve_buttons_toggle() to enable it.
   *
   * \param stick_type
   *        ez::SINGLE or ez::SPLIT control
   */
  void opcontrol_arcade_flipped(e_type stick_type);

  /**
   * Initializes left and right curves with the SD card, recommended to run in initialize().
   */
  void opcontrol_curve_sd_initialize();

  /**
   * Sets the default joystick curves.
   *
   * \param left
   *        left default curve
   * \param right
   *        right default curve
   */
  void opcontrol_curve_default_set(double left, double right = 0);

  /**
   * Gets the default joystick curves, in {left, right}
   */
  std::vector<double> opcontrol_curve_default_get();

  /**
   * Runs a PID loop on the drive when the joysticks are released.
   *
   * \param kp
   *        proportional term
   * \param ki
   *        integral term
   * \param kd
   *        derivative term
   * \param start_i
   *        error threshold to start integral
   */
  void opcontrol_drive_activebrake_set(double kp, double ki = 0.0, double kd = 0.0, double start_i = 0.0);

  /**
   * Returns kP for active brake.
   */
  double opcontrol_drive_activebrake_get();

  /**
   * Returns all PID constants for active brake.
   */
  PID::Constants opcontrol_drive_activebrake_constants_get();

  /**
   * Enables/disables modifying the joystick input curves with the controller.
   *
   * \param input
   *        true enables, false disables
   */
  void opcontrol_curve_buttons_toggle(bool toggle);

  /**
   * Gets the current state of the toggle. Enables/disables modifying the joystick input curves with the controller.
   *
   * True enabled, false disabled.
   */
  bool opcontrol_curve_buttons_toggle_get();

  /**
   * Sets buttons for modifying the left joystick curve.
   *
   * \param decrease
   *        a pros button enumerator
   * \param increase
   *        a pros button enumerator
   */
  void opcontrol_curve_buttons_left_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

  /**
   * Returns a vector of pros controller buttons user for the left joystick curve, in {decrease, increase}
   */
  std::vector<pros::controller_digital_e_t> opcontrol_curve_buttons_left_get();

  /**
   * Sets buttons for modifying the right joystick curve.
   *
   * \param decrease
   *        a pros button enumerator
   * \param increase
   *        a pros button enumerator
   */
  void opcontrol_curve_buttons_right_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

  /**
   * Returns a vector of pros controller buttons user for the right joystick curve, in {decrease, increase}
   */
  std::vector<pros::controller_digital_e_t> opcontrol_curve_buttons_right_get();

  /**
   * Outputs a curve from 5225A In the Zone.
   *
   * This gives more control over the robot at lower speeds.
   *
   * \param x
   *        joystick input
   */
  double opcontrol_curve_left(double x);

  /**
   * Outputs a curve from 5225A In the Zone.
   *
   * This gives more control over the robot at lower speeds.
   *
   * \param x
   *        joystick input
   */
  double opcontrol_curve_right(double x);

  /**
   * Sets a new threshold for the joystick.
   *
   * The joysticks wil not return a value if they are within this.
   *
   * \param threshold
   *        new threshold
   */
  void opcontrol_joystick_threshold_set(int threshold);

  /**
   * Gets the threshold for the joystick.
   */
  int opcontrol_joystick_threshold_get();

  /**
   * Resets drive sensors at the start of opcontrol.
   */
  void opcontrol_drive_sensors_reset();

  /**
   * Sets minimum value distance constants.
   *
   * \param l_stick
   *        input for left joystick
   * \param r_stick
   *        input for right joystick
   */
  void opcontrol_joystick_threshold_iterate(int l_stick, int r_stick);

  /////
  //
  // PTO
  //
  /////

  /**
   * Checks if the motor is currently in pto_list.
   *
   * Returns true if it's already in pto_list.
   *
   * \param check_if_pto
   *        motor to check
   */
  bool pto_check(pros::Motor check_if_pto);

  /**
   * Adds motors to the pto list, removing them from the drive.
   *
   * You cannot add the first index because it's used for autonomous.
   *
   * \param pto_list
   *        list of motors to remove from the drive
   */
  void pto_add(std::vector<pros::Motor> pto_list);

  /**
   * Removes motors from the pto list, adding them to the drive.
   *
   * \param pto_list
   *        list of motors to add to the drive
   */
  void pto_remove(std::vector<pros::Motor> pto_list);

  /**
   * Adds/removes motors from drive.
   * 
   * You cannot add the first index because it's used for autonomous.
   *
   * \param pto_list
   *        list of motors to add/remove from the drive
   * \param toggle
   *        list of motors to add/remove from the drive
   */
  void pto_toggle(std::vector<pros::Motor> pto_list, bool toggle);

  /////
  //
  // PROS Wrappers
  //
  /////

  /**
   * Sets the chassis to voltage.
   *
   * Disables PID when called.
   *
   * \param left
   *        voltage for left side, -127 to 127
   * \param right
   *        voltage for right side, -127 to 127
   */
  void drive_set(int left, int right);

  /**
   * Gets the chassis to voltage, -127 to 127.  Returns {left, right}.
   */
  std::vector<int> drive_get();

  /**
   * Changes the way the drive behaves when it is not under active user control.
   *
   * \param brake_type
   *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
   */
  void drive_brake_set(pros::motor_brake_mode_e_t brake_type);

  /**
   * Returns the brake mode of the drive in pros_brake_mode_e_t_.
   */
  pros::motor_brake_mode_e_t drive_brake_get();

  /**
   * Sets the limit for the current on the drive.
   *
   * \param mA
   *        input in milliamps
   */
  void drive_current_limit_set(int mA);

  /**
   * Gets the limit for the current on the drive.
   */
  int drive_current_limit_get();

  /**
   * Toggles set drive in autonomous.
   *
   * \param toggle
   *        true enables, false disables
   */
  void pid_drive_toggle(bool toggle);

  /**
   * Gets the current state of the toggle.
   *
   * This toggles set drive in autonomous.
   *
   * True enabled, false disabled.
   */
  bool pid_drive_toggle_get();

  /**
   * Toggles printing in autonomous.
   *
   * \param toggle
   *        true enables, false disables
   */
  void pid_print_toggle(bool toggle);

  /**
   * Gets the current state of the toggle.
   *
   * This toggles printing in autonomous.
   *
   * True enabled, false disabled.
   */
  bool pid_print_toggle_get();

  /////
  //
  // Telemetry
  //
  /////

  /**
   * The position of the right sensor in inches.
   *
   * If you have two parallel tracking wheels, this will return tracking wheel position.  Otherwise this returns motor position.
   */
  double drive_sensor_right();

  /**
   * The position of the right sensor.
   *
   * If you have two parallel tracking wheels, this will return tracking wheel position.  Otherwise this returns motor position.
   */
  int drive_sensor_right_raw();

  /**
   * The velocity of the right motor.
   */
  int drive_velocity_right();

  /**
   * The watts of the right motor.
   */
  double drive_mA_right();

  /**
   * Return true when the motor is over current.
   */
  bool drive_current_right_over();

  /**
   * The position of the left sensor in inches.
   *
   * If you have two parallel tracking wheels, this will return tracking wheel position.  Otherwise this returns motor position.
   */
  double drive_sensor_left();

  /**
   * The position of the left sensor.
   *
   * If you have two parallel tracking wheels, this will return tracking wheel position.  Otherwise this returns motor position.
   */
  int drive_sensor_left_raw();

  /**
   * The velocity of the left motor.
   */
  int drive_velocity_left();

  /**
   * The watts of the left motor.
   */
  double drive_mA_left();

  /**
   * Return true when the motor is over current.
   */
  bool drive_current_left_over();

  /**
   * Reset all the chassis motors and tracking wheels, recommended to run at the start of your autonomous routine.
   */
  void drive_sensor_reset();

  /**
   * Resets the current imu value.  Defaults to 0, recommended to run at the start of your autonomous routine.
   *
   * \param new_heading
   *        new heading value
   */
  void drive_imu_reset(double new_heading = 0);

  /**
   * Returns the current imu rotation value in degrees.
   */
  double drive_imu_get();

  /**
   * Returns the current imu accel x + accel y value.
   */
  double drive_imu_accel_get();

  /**
   * Sets a new imu scaling factor.
   *
   * This value is multiplied by the imu to change its output.
   *
   * \param scaler
   *        factor to scale the imu by
   */
  void drive_imu_scaler_set(double scaler);

  /**
   * Returns the current imu scaling factor.
   */
  double drive_imu_scaler_get();

  /**
   * Calibrates the IMU, recommended to run in initialize().
   *
   * \param run_loading_animation
   *        true runs the animation, false doesn't
   */
  bool drive_imu_calibrate(bool run_loading_animation = true);

  /**
   * Checks if the imu calibrated successfully or if it took longer than expected.
   *
   * Returns true if calibrated successfully, and false if unsuccessful.
   */
  bool drive_imu_calibrated();

  /**
   * Loading display while the IMU calibrates.
   */
  void drive_imu_display_loading(int iter);

  /**
   * Practice mode for driver practice that shuts off the drive if you go max speed.
   *
   * \param toggle
   *        true enables, false disables
   */
  void opcontrol_joystick_practicemode_toggle(bool toggle);

  /**
   * Gets current state of the toggle.
   *
   * True is enabled, false is disabled.
   */
  bool opcontrol_joystick_practicemode_toggle_get();

  /**
   * Reversal for drivetrain in opcontrol that flips the left and right side and the direction of the drive.
   *
   * \param toggle
   *        true if you want your drivetrain reversed and false if you do not
   */
  void opcontrol_drive_reverse_set(bool toggle);

  /**
   * Gets current state of the toggle.
   *
   * Reversal for drivetrain in opcontrol that flips the left and right side and the direction of the drive.
   */
  bool opcontrol_drive_reverse_get();

  /////
  //
  // Autonomous Functions
  //
  /////

  /**
   * Sets the robot to move forward using PID without okapi units, only using slew if globally enabled.
   *
   * This function is actually odom.
   *
   * \param target
   *        target value as a double, unit is inches
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_odom_set(double target, int speed);

  /**
   * Sets the robot to move forward using PID without okapi units, using slew if enabled for this motion.
   *
   * This function is actually odom
   *
   * \param target
   *        target value as a double, unit is inches
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_set(double target, int speed, bool slew_on);

  /**
   * Sets the robot to move forward using PID with okapi units, only using slew if globally enabled.
   *
   * This function is actually odom
   *
   * \param target
   *        target value in inches
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_odom_set(okapi::QLength p_target, int speed);

  /**
   * Sets the robot to move forward using PID with okapi units, using slew if enabled for this motion.
   *
   * This function is actually odom
   *
   * \param target
   *        target value in inches
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   * \param toggle_heading
   *        toggle for heading correction.  true enables, false disables
   */
  void pid_odom_set(okapi::QLength p_target, int speed, bool slew_on);

  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   */
  void pid_odom_set(odom imovement);

  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_set(odom imovement, bool slew_on);

  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   */
  void pid_odom_ptp_set(odom imovement);

  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_ptp_set(odom imovement, bool slew_on);

  /**
   * Takes in an odom movement to go to a single point using boomerang.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   */
  void pid_odom_boomerang_set(odom imovement);

  /**
   * Takes in an odom movement to go to a single point using boomerang.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_boomerang_set(odom imovement, bool slew_on);

  /**
   * Takes in an odom movement to go to a single point using boomerang.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   */
  void pid_odom_boomerang_set(united_odom p_imovement);

  /**
   * Takes in an odom movement to go to a single point using boomerang.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_boomerang_set(united_odom p_imovement, bool slew_on);

  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   */
  void pid_odom_ptp_set(united_odom p_imovement);

  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_ptp_set(united_odom p_imovement, bool slew_on);

  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   */
  void pid_odom_set(united_odom p_imovement);

  /**
   * Takes in an odom movement to go to a single point.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_set(united_odom p_imovement, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   */
  void pid_odom_set(std::vector<odom> imovements);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_set(std::vector<odom> imovements, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   */
  void pid_odom_pp_set(std::vector<odom> imovements);

  /**
   * Takes in odom movements to go through multiple points.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_pp_set(std::vector<odom> imovements, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   */
  void pid_odom_injected_pp_set(std::vector<odom> imovements);

  /**
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_injected_pp_set(std::vector<odom> imovements, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   */
  void pid_odom_smooth_pp_set(std::vector<odom> imovements);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_smooth_pp_set(std::vector<odom> imovements, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   */
  void pid_odom_smooth_pp_set(std::vector<united_odom> p_imovements);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_smooth_pp_set(std::vector<united_odom> p_imovements, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   */
  void pid_odom_injected_pp_set(std::vector<united_odom> p_imovements);

  /**
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_injected_pp_set(std::vector<united_odom> p_imovements, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   */
  void pid_odom_pp_set(std::vector<united_odom> p_imovements);

  /**
   * Takes in odom movements to go through multiple points.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_pp_set(std::vector<united_odom> p_imovements, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   */
  void pid_odom_set(std::vector<united_odom> p_imovements);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements.  values are united here with okapi units
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_set(std::vector<united_odom> p_imovements, bool slew_on);

  /**
   * Sets the robot to move forward using PID with okapi units, only using slew if globally enabled.
   *
   * \param p_target
   *        target okapi unit
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_drive_set(okapi::QLength p_target, int speed);

  /**
   * Sets the robot to move forward using PID with okapi units, using slew if enabled for this motion.
   *
   * \param p_target
   *        target okapi unit
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   * \param toggle_heading
   *        toggle for heading correction.  true enables, false disables
   */
  void pid_drive_set(okapi::QLength p_target, int speed, bool slew_on, bool toggle_heading = true);

  /**
   * Sets the robot to move forward using PID without okapi units, only using slew if globally enabled.
   *
   * \param target
   *        target value in inches
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_drive_set(double target, int speed);

  /**
   * Sets the robot to move forward using PID without okapi units, using slew if enabled for this motion.
   *
   * \param target
   *        target value in inches
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   * \param toggle_heading
   *        toggle for heading correction.  true enables, false disables
   */
  void pid_drive_set(double target, int speed, bool slew_on, bool toggle_heading = true);

  /**
   * Sets the robot to turn face a point using PID and odometry.
   *
   * \param target
   *        {x, y}  a target point to face
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_turn_set(pose itarget, drive_directions dir, int speed);

  /**
   * Sets the robot to turn face a point using PID and odometry.
   *
   * \param target
   *        {x, y}  a target point to face
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(pose itarget, drive_directions dir, int speed, bool slew_on);

  /**
   * Sets the robot to turn face a point using PID and odometry.
   *
   * \param target
   *        {x, y}  a target point to face
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   */
  void pid_turn_set(pose itarget, drive_directions dir, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn face a point using PID and odometry.
   *
   * \param target
   *        {x, y}  a target point to face
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(pose itarget, drive_directions dir, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn face a point using PID and odometry.
   *
   * \param target
   *        {x, y}  a target point to face.  this uses okapi units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed);

  /**
   * Sets the robot to turn face a point using PID and odometry.
   *
   * \param target
   *        {x, y}  a target point to face.  this uses okapi units
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, bool slew_on);

  /**
   * Sets the robot to turn face a point using PID and odometry.
   *
   * \param target
   *        {x, y}  a target point to face.  this uses okapi units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   */
  void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn face a point using PID and odometry.
   *
   * \param target
   *        {x, y}  a target point to face.  this uses okapi units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn relative to initial heading using PID.
   *
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(double target, int speed);

  /**
   * Sets the robot to turn relative to initial heading using PID.
   *
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   */
  void pid_turn_set(double target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn relative to initial heading using PID, using slew if enabled for this motion.
   *
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(double target, int speed, bool slew_on);

  /**
   * Sets the robot to turn relative to initial heading using PID, using slew if enabled for this motion.
   *
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(double target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn relative to initial heading using PID with okapi units.
   *
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_turn_set(okapi::QAngle p_target, int speed);

  /**
   * Sets the robot to turn relative to initial heading using PID with okapi units.
   *
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   */
  void pid_turn_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn relative to initial heading using PID with okapi units, using slew if enabled for this motion.
   *
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(okapi::QAngle p_target, int speed, bool slew_on);

  /**
   * Sets the robot to turn relative to initial heading using PID with okapi units, using slew if enabled for this motion.
   *
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn relative to current heading using PID with okapi units, only using slew if globally enabled.
   *
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_turn_relative_set(okapi::QAngle p_target, int speed);

  /**
   * Sets the robot to turn relative to current heading using PID with okapi units, only using slew if globally enabled.
   *
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   */
  void pid_turn_relative_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn relative to current heading using PID with okapi units, using slew if enabled for this motion.
   *
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_relative_set(okapi::QAngle p_target, int speed, bool slew_on);

  /**
   * Sets the robot to turn relative to current heading using PID with okapi units, using slew if enabled for this motion.
   *
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_relative_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn relative to current heading using PID without okapi units, only using slew if globally enabled.
   *
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_turn_relative_set(double target, int speed);

  /**
   * Sets the robot to turn relative to current heading using PID without okapi units, only using slew if globally enabled.
   *
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   */
  void pid_turn_relative_set(double target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn relative to current heading using PID without okapi units, using slew if enabled for this motion.
   *
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_relative_set(double target, int speed, bool slew_on);

  /**
   * Sets the robot to turn relative to current heading using PID without okapi units, using slew if enabled for this motion.
   *
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be left, right, shortest, longest, raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_relative_set(double target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_set(e_swing type, double target, int speed);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_set(e_swing type, double target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_set(e_swing type, double target, int speed, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_set(e_swing type, double target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_set(e_swing type, double target, int speed, int opposite_speed);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_set(e_swing type, double target, int speed, int opposite_speed, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to initial heading using PID with okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID with okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID with okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID with okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID with okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID with okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID with okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID with okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID with okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value in okapi angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID without okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_relative_set(e_swing type, double target, int speed);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID without okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID without okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID without okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID without okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID without okapi units, only using slew if globally enabled.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID without okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to current heading using PID without okapi units, using slew if enabled for this motion.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param p_target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing. this is used for arcs, and is defaulted to 0
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Resets all PID targets to 0.
   */
  void pid_targets_reset();

  /**
   * Sets heading of imu and target of PID, okapi angle.
   */
  void drive_angle_set(okapi::QAngle p_angle);

  /**
   * Sets heading of imu and target of PID, takes double as an angle.
   */
  void drive_angle_set(double angle);

  /**
   * Lock the code in a while loop until the robot has settled.
   */
  void pid_wait();

  /**
   * Lock the code in a while loop until this position has passed for turning or swinging with okapi units.
   *
   * \param target
   *        for turning, using okapi units
   */
  void pid_wait_until(okapi::QAngle target);

  /**
   * Lock the code in a while loop until this position has passed for driving with okapi units.
   *
   * \param target
   *        for driving, using okapi units
   */
  void pid_wait_until(okapi::QLength target);

  /**
   * Lock the code in a while loop until this position has passed for driving without okapi units.
   *
   * \param target
   *        for driving or turning, using a double.  degrees for turns/swings, inches for driving
   */
  void pid_wait_until(double target);

  /**
   * Lock the code in a while loop until the robot has settled.
   *
   * Wrapper for pid_wait_until(target), target is your previously input target.
   */
  void pid_wait_quick();

  /**
   * Lock the code in a while loop until the robot has settled.
   *
   * This also adds distance to target, and then exits with pid_wait_quick.
   *
   * This will exit the motion while carrying momentum into the next motion.
   */
  void pid_wait_quick_chain();

  /**
   * Lock the code in a while loop until this point has been passed.
   *
   * \param index
   *        index of your input points, 0 is the first point in the index
   */
  void pid_wait_until_index(int index);

  /**
   * Lock the code in a while loop until this point becomes the target.
   *
   * \param index
   *        index of your input points, 0 is the first point in the index
   */
  void pid_wait_until_index_started(int index);

  /**
   * Lock the code in a while loop until this point has been passed.
   *
   * \param target
   *        {x, y} pose for the robot to pass through before the while loop is released
   */
  void pid_wait_until_point(pose target);

  /**
   * Lock the code in a while loop until this point has been passed, with okapi units.
   *
   * \param target
   *        {x, y} pose with units for the robot to pass through before the while loop is released
   */
  void pid_wait_until_point(united_pose target);

  /**
   * Lock the code in a while loop until this point has been passed.
   *
   * Wrapper for pid_wait_until_point.
   *
   * \param target
   *        {x, y}  a pose for the robot to pass through before the while loop is released
   */
  void pid_wait_until(pose target);

  /**
   * Lock the code in a while loop until this point has been passed, with okapi units.
   *
   * Wrapper for pid_wait_until_point.
   *
   * \param target
   *        {x, y}  a pose with units for the robot to pass through before the while loop is released
   */
  void pid_wait_until(united_pose target);

  /**
   * Autonomous interference detection.
   *
   * Returns true when interfered, and false when nothing happened.
   */
  bool interfered = false;

  /**
   * Set the ratio of the robot.
   *
   * \param ratio
   *        ratio of the gears
   */
  void drive_ratio_set(double ratio);

  /**
   * Set the cartridge/wheel rpm of the robot.
   *
   * \param rpm
   *        rpm of the cartridge or wheel
   */
  void drive_rpm_set(double rpm);

  /**
   * Returns the ratio of the drive.
   */
  double drive_ratio_get();

  /**
   * Returns the current cartridge / wheel rpm.
   */
  double drive_rpm_get();

  /**
   * Changes max speed during a drive motion.
   *
   * \param speed
   *        new clipped speed, between 0 and 127     
   */
  void pid_speed_max_set(int speed);

  /**
   * Returns max speed of drive during autonomous.
   */
  int pid_speed_max_get();

  /**
   * Set the turn pid constants object.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral
   */
  void pid_turn_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns PID constants with PID::Constants.
   */
  PID::Constants pid_turn_constants_get();

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This sets turning constants.
   *
   * \param input
   *        okapi angle unit
   */
  void pid_turn_chain_constant_set(okapi::QAngle input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This sets turning constants.
   *
   * \param input
   *        angle in degrees
   */
  void pid_turn_chain_constant_set(double input);

  /**
   * Returns the amount that the PID will overshoot target by to maintain momentum into the next motion for turning.
   */
  double pid_turn_chain_constant_get();

  /**
   * Set the swing pid constants object.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral
   */
  void pid_swing_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns PID constants with PID::Constants.
   *
   * Returns -1 if fwd and rev constants aren't the same!
   */
  PID::Constants pid_swing_constants_get();

  /**
   * Set the forward swing pid constants object.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral
   */
  void pid_swing_constants_forward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns PID constants with PID::Constants.
   */
  PID::Constants pid_swing_constants_forward_get();

  /**
   * Set the backward swing pid constants object.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral
   */
  void pid_swing_constants_backward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns PID constants with PID::Constants.
   */
  PID::Constants pid_swing_constants_backward_get();

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This sets forward and backwards swing constants.
   *
   * \param input
   *        okapi angle unit
   */
  void pid_swing_chain_constant_set(okapi::QAngle input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets forward swing constants.
   *
   * \param input
   *        okapi angle unit
   */
  void pid_swing_chain_forward_constant_set(okapi::QAngle input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets backward swing constants.
   *
   * \param input
   *        okapi angle unit
   */
  void pid_swing_chain_backward_constant_set(okapi::QAngle input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This sets forward and backwards swing constants.
   *
   * \param input
   *        angle in degrees
   */
  void pid_swing_chain_constant_set(double input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets forward constants.
   *
   * \param input
   *        angle in degrees
   */
  void pid_swing_chain_forward_constant_set(double input);

  /**
   * Returns the amount that the PID will overshoot target by to maintain momentum into the next motion for swinging forward.
   */
  double pid_swing_chain_forward_constant_get();

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets backwards swing constants.
   *
   * \param input
   *        angle in degrees
   */
  void pid_swing_chain_backward_constant_set(double input);

  /**
   * Returns the amount that the PID will overshoot target by to maintain momentum into the next motion for swinging backward.
   */
  double pid_swing_chain_backward_constant_get();

  /**
   * Set the heading pid constants object.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral
   */
  void pid_heading_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns PID constants with PID::Constants.
   */
  PID::Constants pid_heading_constants_get();

  /**
   * Set the drive pid constants object.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral
   */
  void pid_drive_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns PID constants with PID::Constants.
   *
   * Returns -1 if fwd and rev constants aren't the same!
   */
  PID::Constants pid_drive_constants_get();

  /**
   * Set the forward pid constants object.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral
   */
  void pid_drive_constants_forward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns PID constants with PID::Constants.
   */
  PID::Constants pid_drive_constants_forward_get();

  /**
   * Set the backwards pid constants object.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral
   */
  void pid_drive_constants_backward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns PID constants with PID::Constants.
   */
  PID::Constants pid_drive_constants_backward_get();

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This sets forward and backwards driving constants.
   *
   * \param input
   *        okapi length unit
   */
  void pid_drive_chain_constant_set(okapi::QLength input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets forward driving constants.
   *
   * \param input
   *        okapi length unit
   */
  void pid_drive_chain_forward_constant_set(okapi::QLength input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets backward driving constants.
   *
   * \param input
   *        okapi length unit
   */
  void pid_drive_chain_backward_constant_set(okapi::QLength input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This sets forward and backwards driving constants.
   *
   * \param input
   *        distance in inches
   */
  void pid_drive_chain_constant_set(double input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets forward driving constants.
   *
   * \param input
   *        distance in inches
   */
  void pid_drive_chain_forward_constant_set(double input);

  /**
   * Returns the amount that the PID will overshoot target by to maintain momentum into the next motion for driving forward.
   */
  double pid_drive_chain_forward_constant_get();

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets backward driving constants.
   *
   * \param input
   *        distance in inches
   */
  void pid_drive_chain_backward_constant_set(double input);

  /**
   * Returns the amount that the PID will overshoot target by to maintain momentum into the next motion for driving backward.
   */
  double pid_drive_chain_backward_constant_get();

  /**
   * Sets minimum power for swings when kI and startI are enabled.
   *
   * \param min
   *        new clipped speed
   */
  void pid_swing_min_set(int min);

  /**
   * The minimum power for turns when kI and startI are enabled.
   *
   * \param min
   *        new clipped speed
   */
  void pid_turn_min_set(int min);

  /**
   * Returns minimum power for swings when kI and startI are enabled.
   */
  int pid_swing_min_get();

  /**
   * Returns minimum power for turns when kI and startI are enabled.
   */
  int pid_turn_min_get();

  /**
   * Set the odom angular pid constants object.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral
   */
  void pid_odom_angular_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Set the odom boomerang pid constants object.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral
   */
  void pid_odom_boomerang_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Set's constants for odom driving exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within smalL_error, in ms
   * \param p_small_error
   *        small timer will start when error is within this, in inches
   * \param p_big_exit_time
   *        time to exit when within big_error, in ms
   * \param p_big_error
   *        big timer will start when error is within this, in inches
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0, in ms
   * \param p_mA_timeout
   *        mA timer will start when the motors are pulling too much current, in ms
   * \param use_imu
   *        true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't
   */
  void pid_odom_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu = true);

  /**
   * Set's constants for odom turning exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within smalL_error, in ms
   * \param p_small_error
   *        small timer will start when error is within this, in degrees
   * \param p_big_exit_time
   *        time to exit when within big_error, in ms
   * \param p_big_error
   *        big timer will start when error is within this, in degrees
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0, in ms
   * \param p_mA_timeout
   *        mA timer will start when the motors are pulling too much current, in ms
   * \param use_imu
   *        true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't
   */
  void pid_odom_turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu = true);

  /**
   * Set's constants for odom turning exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within smalL_error, okapi unit
   * \param p_small_error
   *        small timer will start when error is within this, okapi unit
   * \param p_big_exit_time
   *        time to exit when within big_error, okapi unit
   * \param p_big_error
   *        big timer will start when error is within this, okapi unit
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0, okapi unit
   * \param p_mA_timeout
   *        mA timer will start when the motors are pulling too much current, okapi unit
   * \param use_imu
   *        true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't
   */
  void pid_odom_turn_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QAngle p_small_error, okapi::QTime p_big_exit_time, okapi::QAngle p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu = true);

  /**
   * Set's constants for odom driving exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within smalL_error, okapi unit
   * \param p_small_error
   *        small timer will start when error is within this, okapi unit
   * \param p_big_exit_time
   *        time to exit when within big_error, okapi unit
   * \param p_big_error
   *        big timer will start when error is within this, okapi unit
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0, okapi unit
   * \param p_mA_timeout
   *        mA timer will start when the motors are pulling too much current, okapi unit
   * \param use_imu
   *        true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't
   */
  void pid_odom_drive_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QLength p_small_error, okapi::QTime p_big_exit_time, okapi::QLength p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu = true);

  /**
   * Set's constants for drive exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within smalL_error, okapi unit
   * \param p_small_error
   *        small timer will start when error is within this, okapi unit
   * \param p_big_exit_time
   *        time to exit when within big_error, okapi unit
   * \param p_big_error
   *        big timer will start when error is within this, okapi unit
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0, okapi unit
   * \param p_mA_timeout
   *        mA timer will start when the motors are pulling too much current, okapi unit
   * \param use_imu
   *        true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't
   */
  void pid_drive_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QLength p_small_error, okapi::QTime p_big_exit_time, okapi::QLength p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu = true);

  /**
   * Set's constants for turn exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within smalL_error, okapi unit
   * \param p_small_error
   *        small timer will start when error is within this, okapi unit
   * \param p_big_exit_time
   *        time to exit when within big_error, okapi unit
   * \param p_big_error
   *        big timer will start when error is within this, okapi unit
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0, okapi unit
   * \param p_mA_timeout
   *        mA timer will start when the motors are pulling too much current, okapi unit
   * \param use_imu
   *        true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't
   */
  void pid_turn_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QAngle p_small_error, okapi::QTime p_big_exit_time, okapi::QAngle p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu = true);

  /**
   * Set's constants for swing exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within smalL_error, okapi unit
   * \param p_small_error
   *        small timer will start when error is within this, okapi unit
   * \param p_big_exit_time
   *        time to exit when within big_error, okapi unit
   * \param p_big_error
   *        big timer will start when error is within this, okapi unit
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0, okapi unit
   * \param p_mA_timeout
   *        mA timer will start when the motors are pulling too much current, okapi unit
   * \param use_imu
   *        true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't
   */
  void pid_swing_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QAngle p_small_error, okapi::QTime p_big_exit_time, okapi::QAngle p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu = true);

  /**
   * Set's constants for drive exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within smalL_error, in ms
   * \param p_small_error
   *        small timer will start when error is within this, in inches
   * \param p_big_exit_time
   *        time to exit when within big_error, in ms
   * \param p_big_error
   *        big timer will start when error is within this, in inches
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0, in ms
   * \param p_mA_timeout
   *        mA timer will start when the motors are pulling too much current, in ms
   * \param use_imu
   *        true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't
   */
  void pid_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu = true);

  /**
   * Set's constants for turn exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within smalL_error, in ms
   * \param p_small_error
   *        small timer will start when error is within this, in degrees
   * \param p_big_exit_time
   *        time to exit when within big_error, in ms
   * \param p_big_error
   *        big timer will start when error is within this, in degrees
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0, in ms
   * \param p_mA_timeout
   *        mA timer will start when the motors are pulling too much current, in ms
   * \param use_imu
   *        true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't
   */
  void pid_turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu = true);

  /**
   * Set's constants for swing exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within smalL_error, in ms
   * \param p_small_error
   *        small timer will start when error is within this, in degrees
   * \param p_big_exit_time
   *        time to exit when within big_error, in ms
   * \param p_big_error
   *        big timer will start when error is within this, in degrees
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0, in ms
   * \param p_mA_timeout
   *        mA timer will start when the motors are pulling too much current, in ms
   * \param use_imu
   *        true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't
   */
  void pid_swing_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu = true);

  /**
   * Returns current tick_per_inch.
   */
  double drive_tick_per_inch();

  /**
   * Iterates modifying controller curves with the controller.
   */
  void opcontrol_curve_buttons_iterate();

  /**
   * Enables PID Tuner.
   */
  void pid_tuner_enable();

  /**
   * Disables PID Tuner.
   */
  void pid_tuner_disable();

  /**
   * Toggles PID tuner between enabled and disabled.
   */
  void pid_tuner_toggle();

  /**
   * Checks if PID Tuner is enabled.
   * 
   * True is enabled, false is disabled.
   */
  bool pid_tuner_enabled();

  /**
   * Iterates through controller inputs to modify PID constants.
   */
  void pid_tuner_iterate();

  /**
   * Toggle for printing the display of the PID Tuner to the brain.
   *
   * \param input
   *        true prints to brain, false doesn't
   */
  void pid_tuner_print_brain_set(bool input);

  /**
   * Toggle for printing the display of the PID Tuner to the terminal.
   *
   * \param input
   *        true prints to terminal, false doesn't
   */
  void pid_tuner_print_terminal_set(bool input);

  /**
   * Returns true if printing to terminal is enabled.
   */
  bool pid_tuner_print_terminal_enabled();

  /**
   * Returns true if printing to brain is enabled.
   */
  bool pid_tuner_print_brain_enabled();

  /**
   * Sets the value that PID Tuner increments P.
   *
   * \param p
   *        p will increase by this
   */
  void pid_tuner_increment_p_set(double p);

  /**
   * Sets the value that PID Tuner increments I.
   *
   * \param p
   *        i will increase by this
   */
  void pid_tuner_increment_i_set(double i);

  /**
   * Sets the value that PID Tuner increments D.
   *
   * \param p
   *        d will increase by this
   */
  void pid_tuner_increment_d_set(double d);

  /**
   * Sets the value that PID Tuner increments Start I.
   *
   * \param p
   *        start i will increase by this
   */
  void pid_tuner_increment_start_i_set(double start_i);

  /**
   * Returns the value that PID Tuner increments P.
   */
  double pid_tuner_increment_p_get();

  /**
   * Returns the value that PID Tuner increments I.
   */
  double pid_tuner_increment_i_get();

  /**
   * Returns the value that PID Tuner increments D.
   */
  double pid_tuner_increment_d_get();

  /**
   * Returns the value that PID Tuner increments Start I.
   */
  double pid_tuner_increment_start_i_get();

  /**
   * Enables the full PID tuner with unique fwd/rev constants.
   *
   * \param enable
   *        true will enable the full PID tuner, false will use the simplified PID tuner
   */
  void pid_tuner_full_enable(bool enable);

  /**
   * Returns if the full PID tuner with unique fwd/rev constants is enabled.
   *
   * True means the full PID tuner is enabled, false means the simplified PID tuner is enabled.
   */
  bool pid_tuner_full_enabled();

  struct const_and_name {
    std::string name = "";
    PID::Constants* consts;
  };

  /**
   * Vector used for a simplified PID Tuner
   */
  std::vector<const_and_name> pid_tuner_pids = {
      {"Drive PID Constants", &fwd_rev_drivePID.constants},
      {"Odom Angular PID Constants", &odom_angularPID.constants},
      {"Boomerang Angular PID Constants", &boomerangPID.constants},
      {"Heading PID Constants", &headingPID.constants},
      {"Turn PID Constants", &turnPID.constants},
      {"Swing PID Constants", &fwd_rev_swingPID.constants}};

  /**
   * Vector used for the full PID Tuner
   */
  std::vector<const_and_name> pid_tuner_full_pids = {
      {"Drive Forward PID Constants", &forward_drivePID.constants},
      {"Drive Backward PID Constants", &backward_drivePID.constants},
      {"Odom Angular PID Constants", &odom_angularPID.constants},
      {"Boomerang Angular PID Constants", &boomerangPID.constants},
      {"Heading PID Constants", &headingPID.constants},
      {"Turn PID Constants", &turnPID.constants},
      {"Swing Forward PID Constants", &forward_swingPID.constants},
      {"Swing Backward PID Constants", &backward_swingPID.constants}};

  /**
   * Sets the max speed for user control.
   *
   * \param int
   *        the speed limit
   */
  void opcontrol_speed_max_set(int speed);

  /**
   * Returns the max speed for user control.
   */
  int opcontrol_speed_max_get();

  /**
   * Toggles vector scaling for arcade control.  True enables, false disables.
   *
   * \param bool
   *        true enables, false disables
   */
  void opcontrol_arcade_scaling(bool enable);

  /**
   * Returns if vector scaling for arcade control is enabled.  True enables, false disables.
   */
  bool opcontrol_arcade_scaling_enabled();

 private:
  void opcontrol_drive_activebrake_targets_set();
  double odom_smooth_weight_smooth = 0.0;
  double odom_smooth_weight_data = 0.0;
  double odom_smooth_tolerance = 0.0;
  bool odom_use_left = true;
  double odom_ime_track_width_left = 0.0;
  double odom_ime_track_width_right = 0.0;
  bool imu_calibrate_took_too_long = false;
  bool is_full_pid_tuner_enabled = false;
  std::vector<const_and_name>* used_pid_tuner_pids;
  double opcontrol_speed_max = 127.0;
  bool arcade_vector_scaling = false;
  // odom privates
  std::vector<odom> pp_movements;
  std::vector<int> injected_pp_index;
  int pp_index = 0;
  std::vector<odom> smooth_path(std::vector<odom> ipath, double weight_smooth, double weight_data, double tolerance);
  double is_past_target(pose target, pose current);
  void raw_pid_odom_pp_set(std::vector<odom> imovements, bool slew_on);
  bool ptf1_running = false;
  std::vector<pose> find_point_to_face(pose current, pose target, drive_directions dir, bool set_global);
  void raw_pid_odom_ptp_set(odom imovement, bool slew_on);
  std::vector<odom> inject_points(std::vector<odom> imovements);
  std::vector<pose> point_to_face = {{0, 0, 0}, {0, 0, 0}};
  double turn_is_toleranced(double target, double current, double input, double longest, double shortest);
  double turn_short(double target, double current, bool print = false);
  double turn_long(double target, double current, bool print = false);
  double new_turn_target_compute(double target, double current, ez::e_angle_behavior behavior);
  double turn_left(double target, double current, bool print = false);
  double turn_right(double target, double current, bool print = false);
  bool imu_calibration_complete = false;
  bool is_swing_slew_enabled(e_swing type, double target, double current);
  bool slew_reenables_when_max_speed_changes = true;
  int slew_min_when_it_enabled = 0;
  bool slew_will_enable_later = false;
  bool current_slew_on = false;
  bool is_odom_turn_bias_enabled = true;
  bool odom_turn_bias_enabled();
  void odom_turn_bias_enable(bool set);
  double angle_rad = 0.0;
  double global_track_width = 0.0;
  bool odometry_enabled = true;
  pose odom_target = {0.0, 0.0, 0.0};
  pose odom_current = {0.0, 0.0, 0.0};
  pose odom_second_to_last = {0.0, 0.0, 0.0};
  pose odom_start = {0.0, 0.0, 0.0};
  pose odom_target_start = {0.0, 0.0, 0.0};
  pose turn_to_point_target = {0.0, 0.0, 0.0};
  bool y_flipped = false;
  bool x_flipped = false;
  bool theta_flipped = false;
  double flip_angle_target(double target);
  double odom_imu_start = 0.0;
  int past_target = 0;
  double SPACING = 0.5;
  double LOOK_AHEAD = 7.0;
  double dlead = 0.5;
  double max_boomerang_distance = 12.0;
  double odom_turn_bias_amount = 1.375;
  drive_directions current_drive_direction = fwd;
  double h_last = 0.0, t_last = 0.0, l_last = 0.0, r_last = 0.0;
  pose l_pose{0.0, 0.0, 0.0};
  pose r_pose{0.0, 0.0, 0.0};
  pose central_pose{0.0, 0.0, 0.0};
  double xy_current_fake = 0.0;
  double xy_last_fake = 0.0;
  double xy_delta_fake = 0.0;
  double new_current_fake = 0.0;
  bool was_odom_just_set = false;
  std::pair<float, float> decide_vert_sensor(ez::tracking_wheel* tracker, bool is_tracker_enabled, float ime = 0.0, float ime_track = 0.0);
  pose solve_xy_vert(float p_track_width, float current_t, float delta_vert, float delta_t);
  pose solve_xy_horiz(float p_track_width, float current_t, float delta_horiz, float delta_t);
  bool was_last_pp_mode_boomerang = false;
  bool global_forward_drive_slew_enabled = false;
  bool global_backward_drive_slew_enabled = false;
  bool global_forward_swing_slew_enabled = false;
  bool global_backward_swing_slew_enabled = false;
  double turn_tolerance = 3.0;
  bool global_turn_slew_enabled = false;
  e_angle_behavior current_angle_behavior = raw;
  e_angle_behavior default_swing_type = raw;
  e_angle_behavior default_turn_type = raw;
  e_angle_behavior default_odom_type = shortest;
  bool turn_biased_left = false;
  std::vector<odom> set_odoms_direction(std::vector<odom> inputs);
  odom set_odom_direction(odom input);
  pose flip_pose(pose input);
  bool odom_tracker_left_enabled = false;
  bool odom_tracker_right_enabled = false;
  bool odom_tracker_front_enabled = false;
  bool odom_tracker_back_enabled = false;

  double chain_target_start = 0.0;
  double chain_sensor_start = 0.0;
  double drive_forward_motion_chain_scale = 0.0;
  double drive_backward_motion_chain_scale = 0.0;
  double swing_forward_motion_chain_scale = 0.0;
  double swing_backward_motion_chain_scale = 0.0;
  double turn_motion_chain_scale = 0.0;
  double used_motion_chain_scale = 0.0;
  bool motion_chain_backward = false;

  double IMU_SCALER = 1.0;

  bool drive_toggle = true;
  bool print_toggle = true;
  int swing_min = 0;
  int turn_min = 0;
  bool practice_mode_is_on = false;
  int swing_opposite_speed = 0;
  bool slew_swing_fwd_using_angle = false;
  bool slew_swing_rev_using_angle = false;
  bool slew_swing_using_angle = false;
  bool pid_tuner_terminal_b = false;
  bool pid_tuner_lcd_b = true;
  void pid_tuner_print();
  void pid_tuner_value_modify(float p, float i, float d, float start);
  void pid_tuner_value_increase();
  void pid_tuner_value_decrease();
  void pid_tuner_print_brain();
  void pid_tuner_print_terminal();
  void pid_tuner_brain_init();
  int column = 0;
  int row = 0;
  std::string arrow = " <--\n";
  bool last_controller_curve_state = false;
  bool last_auton_selector_state = false;
  bool pid_tuner_on = false;
  std::string complete_pid_tuner_output = "";
  float p_increment = 0.1, i_increment = 0.001, d_increment = 0.25, start_i_increment = 1.0;

  /**
   * Private wait until for drive
   */
  void wait_until_drive(double target);
  void wait_until_turn_swing(double target);

  /**
   * Sets the chassis to voltage.
   *
   * \param left
   *        voltage for left side, -127 to 127
   * \param right
   *        voltage for right side, -127 to 127
   */
  void private_drive_set(int left, int right);

  /**
   * Returns joystick value clipped to JOYSTICK_THRESH
   */
  int clipped_joystick(int joystick);

  /**
   * Heading bool.
   */
  bool heading_on = true;

  /**
   * Tick per inch calculation.
   */
  double TICK_PER_REV;
  double TICK_PER_INCH;
  double CIRCUMFERENCE;

  double CARTRIDGE;
  double RATIO;
  double WHEEL_DIAMETER;

  /**
   * Max speed for autonomous.
   */
  int max_speed;

  /**
   * Tasks
   */
  void drive_pid_task();
  void swing_pid_task();
  void turn_pid_task();
  void ez_auto_task();
  void ptp_task();
  void boomerang_task();
  void pp_task();

  /**
   * Starting value for left/right
   */
  double l_start = 0;
  double r_start = 0;

  /**
   * Enable/disable modifying controller curve with controller.
   */
  bool disable_controller = true;  // True enables, false disables.

  /**
   * Is tank drive running?
   */
  bool is_tank;

#define DRIVE_INTEGRATED 1
#define DRIVE_ADI_ENCODER 2
#define DRIVE_ROTATION 3
#define ODOM_TRACKER 4

  /**
   * Is tracking?
   */
  int is_tracker = DRIVE_INTEGRATED;

  /**
   * Save input to sd card
   */
  void save_l_curve_sd();
  void save_r_curve_sd();

  /**
   * Struct for buttons for increasing/decreasing curve with controller
   */
  struct button_ {
    bool lock = false;
    bool release_reset = false;
    int release_timer = 0;
    int hold_timer = 0;
    int increase_timer;
    pros::controller_digital_e_t button;
  };

  button_ l_increase_;
  button_ l_decrease_;
  button_ r_increase_;
  button_ r_decrease_;

  /**
   * Function for button presses.
   */
  void button_press(button_* input_name, int button, std::function<void()> changeCurve, std::function<void()> save);

  /**
   * The left and right curve scalers.
   */
  double left_curve_scale;
  double right_curve_scale;

  /**
   * Increase and decrease left and right curve scale.
   */
  void l_decrease();
  void l_increase();
  void r_decrease();
  void r_increase();

  /**
   * Boolean to flip which side is the front of the robot for driver control.
   */
  bool is_reversed = false;
};
};  // namespace ez
