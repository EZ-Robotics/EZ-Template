/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <stack>
#include <tuple>

#include "EZ-Template/PID.hpp"
#include "EZ-Template/lock.hpp"
#include "EZ-Template/slew.hpp"
#include "EZ-Template/tracking_wheel.hpp"
#include "EZ-Template/util.hpp"
#include "EZ-Units/units.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

// using namespace ez;

namespace ez {
class Drive {
public:
  /**
   * Joysticks will return 0 when they are within this number.
   *
   * Defaults to 3: a stick at rest often reads 1 or 2, which would otherwise keep active brake off.
   *
   * Set with opcontrol_joystick_threshold_set()
   */
  int JOYSTICK_THRESHOLD = 3;

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
  e_swing current_swing = LEFT_SWING;

  /**
   * Vector of pros motors for the left chassis.
   */
  std::vector<pros::Motor> left_motors;

  /**
   * Vector of pros motors for the right chassis.
   */
  std::vector<pros::Motor> right_motors;

  /**
   * Vector of smart ports of the motors that are disconnected from the drive.
   */
  std::vector<int> pto_active;

  /**
   * Current focused Inertial sensor.
   */
  pros::Imu* imu = nullptr;

  /**
   * All good imus, for redundancy.
   */
  std::deque<pros::Imu*> good_imus;

  /**
   * Left vertical tracking wheel.
   */
  tracking_wheel* odom_tracker_left = nullptr;

  /**
   * Right vertical tracking wheel.
   */
  tracking_wheel* odom_tracker_right = nullptr;

  /**
   * Front horizontal tracking wheel.
   */
  tracking_wheel* odom_tracker_front = nullptr;

  /**
   * Back horizontal tracking wheel.
   */
  tracking_wheel* odom_tracker_back = nullptr;

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
   * Slew ramps up the speed of the robot from min_speed to full speed (127) over the set angle.  A movement with a lower max speed is capped at that speed,
   * so it reaches that speed before the angle is turned.
   *
   * \param distance
   *        the angle the robot swings through to ramp up to full speed (127), in degrees or any angle unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.slew_swing_constants_set(7_deg, 50);
   *
   *   chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void slew_swing_constants_set(ez::QAngle distance, int min_speed);

  /**
   * Sets constants for slew for swing movements.
   *
   * Slew ramps up the speed of the robot from min_speed to full speed (127) over the set angle.  A movement with a lower max speed is capped at that speed,
   * so it reaches that speed before the angle is turned.
   *
   * \param distance
   *        the angle the robot swings through to ramp up to full speed (127), in degrees
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.slew_swing_constants_set(7.0, 50);
   *
   *   chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void slew_swing_constants_set(double distance, int min_speed);

  /**
   * Sets constants for slew for swing forward movements.
   *
   * Slew ramps up the speed of the robot from min_speed to full speed (127) over the set angle.  A movement with a lower max speed is capped at that speed,
   * so it reaches that speed before the angle is turned.
   *
   * \param distance
   *        the angle the robot swings through to ramp up to full speed (127), in degrees or any angle unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_swing_constants_forward_set(ez::QAngle distance, int min_speed);

  /**
   * Sets constants for slew for swing forward movements.
   *
   * Slew ramps up the speed of the robot from min_speed to full speed (127) over the set angle.  A movement with a lower max speed is capped at that speed,
   * so it reaches that speed before the angle is turned.
   *
   * \param distance
   *        the angle the robot swings through to ramp up to full speed (127), in degrees
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_swing_constants_forward_set(double distance, int min_speed);

  /**
   * Sets constants for slew for swing backward movements.
   *
   * Slew ramps up the speed of the robot from min_speed to full speed (127) over the set angle.  A movement with a lower max speed is capped at that speed,
   * so it reaches that speed before the angle is turned.
   *
   * \param distance
   *        the angle the robot swings through to ramp up to full speed (127), in degrees or any angle unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_swing_constants_backward_set(ez::QAngle distance, int min_speed);

  /**
   * Sets constants for slew for swing backward movements.
   *
   * Slew ramps up the speed of the robot from min_speed to full speed (127) over the set angle.  A movement with a lower max speed is capped at that speed,
   * so it reaches that speed before the angle is turned.
   *
   * \param distance
   *        the angle the robot swings through to ramp up to full speed (127), in degrees
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_swing_constants_backward_set(double distance, int min_speed);

  /**
   * Sets constants for slew for turns.
   *
   * Slew ramps up the speed of the robot from min_speed to full speed (127) over the set angle.  A movement with a lower max speed is capped at that speed,
   * so it reaches that speed before the angle is turned.
   *
   * \param distance
   *        the angle the robot turns through to ramp up to full speed (127), in degrees or any angle unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.slew_turn_constants_set(5_deg, 50);
   *
   *   chassis.pid_turn_set(-90_deg, 110, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void slew_turn_constants_set(ez::QAngle distance, int min_speed);

  /**
   * Sets constants for slew for turns.
   *
   * Slew ramps up the speed of the robot from min_speed to full speed (127) over the set angle.  A movement with a lower max speed is capped at that speed,
   * so it reaches that speed before the angle is turned.
   *
   * \param distance
   *        the angle the robot turns through to ramp up to full speed (127), in degrees
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.slew_turn_constants_set(5.0, 50);
   *
   *   chassis.pid_turn_set(-90_deg, 110, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void slew_turn_constants_set(double distance, int min_speed);

  /**
   * Sets constants for slew for driving forward.
   *
   * Slew ramps up the speed of the robot from `min_speed` to full speed (127) over the set distance.  A movement with a lower max speed is capped at
   * that speed, so it reaches that speed before the distance is traveled.
   *
   * \param distance
   *        the distance the robot travels to ramp up to full speed (127), a distance unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_drive_constants_forward_set(ez::QLength distance, int min_speed);

  /**
   * Sets constants for slew for driving forward.
   *
   * Slew ramps up the speed of the robot from min_speed to full speed (127) over the set distance.  A movement with a lower max speed is capped at that speed,
   * so it reaches that speed before the distance is traveled.
   *
   * \param distance
   *        the distance the robot travels to ramp up to full speed (127), in inches
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_drive_constants_forward_set(double distance, int min_speed);

  /**
   * Sets constants for slew for driving backward.
   *
   * Slew ramps up the speed of the robot from `min_speed` to full speed (127) over the set distance.  A movement with a lower max speed is capped at
   * that speed, so it reaches that speed before the distance is traveled.
   *
   * \param distance
   *        the distance the robot travels to ramp up to full speed (127), a distance unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_drive_constants_backward_set(ez::QLength distance, int min_speed);

  /**
   * Sets constants for slew for driving backward.
   *
   * Slew ramps up the speed of the robot from min_speed to full speed (127) over the set distance.  A movement with a lower max speed is capped at that speed,
   * so it reaches that speed before the distance is traveled.
   *
   * \param distance
   *        the distance the robot travels to ramp up to full speed (127), in inches
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   */
  void slew_drive_constants_backward_set(double distance, int min_speed);

  /**
   * Sets constants for slew for driving.
   *
   * Slew ramps up the speed of the robot from `min_speed` to full speed (127) over the set distance.  A movement with a lower max speed is capped at
   * that speed, so it reaches that speed before the distance is traveled.
   *
   * \param distance
   *        the distance the robot travels to ramp up to full speed (127), a distance unit
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.slew_drive_constants_set(5_in, 50);
   *
   *   chassis.pid_drive_set(12_in, 110, true);
   *   chassis.pid_wait();
   *
   *   chassis.pid_drive_set(-12_in, 110, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void slew_drive_constants_set(ez::QLength distance, int min_speed);

  /**
   * Sets constants for slew for driving.
   *
   * Slew ramps up the speed of the robot from min_speed to full speed (127) over the set distance.  A movement with a lower max speed is capped at that speed,
   * so it reaches that speed before the distance is traveled.
   *
   * \param distance
   *        the distance the robot travels to ramp up to full speed (127), in inches
   * \param min_speed
   *        the starting speed for the movement, 0 - 127
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.slew_drive_constants_set(5.0, 50);
   *
   *   chassis.pid_drive_set(12_in, 110, true);
   *   chassis.pid_wait();
   *
   *   chassis.pid_drive_set(-12_in, 110, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void slew_drive_constants_set(double distance, int min_speed);

  /**
   * Sets the default slew for drive forwards and backwards motions, can be overwritten in movement functions.
   *
   * \param slew_on
   *        true enables, false disables
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Set the default slew state to true
   *   chassis.slew_drive_set(true);
   *
   *   // This will not use slew because we explicitly told the robot to not slew
   *   chassis.pid_drive_set(24_in, 110, false);
   *   chassis.pid_wait();
   *
   *   // This will slew because it's the default state
   *   chassis.pid_drive_set(-24_in, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
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
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Set the default slew state to true
   *   chassis.slew_drive_forward_set(true);
   *   if (chassis.slew_drive_forward_get()) printf("Slew Forward is Enabled!\n");
   *
   *   // This will not use slew because we explicitly told the robot to not slew
   *   chassis.pid_drive_set(24_in, 110, false);
   *   chassis.pid_wait();
   *
   *   // This will slew because it's the default state going forward
   *   chassis.pid_drive_set(24_in, 110);
   *   chassis.pid_wait();
   *
   *   // This will not slew because no default is set for going backward
   *   chassis.pid_drive_set(-48_in, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
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
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Set the default slew state to true
   *   chassis.slew_swing_set(true);
   *
   *   // This will not use slew because we explicitly told the robot to not slew
   *   chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, false);
   *   chassis.pid_wait();
   *
   *   // This will slew because it's the default state
   *   chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
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
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Set the default slew state to true
   *   chassis.slew_swing_forward_set(true);
   *   if (chassis.slew_swing_forward_get()) printf("Slew Forward is Enabled!\n");
   *
   *   // This will not use slew because we explicitly told the robot to not slew
   *   chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, false);
   *   chassis.pid_wait();
   *
   *   // This will slew because it's the default forward state
   *   chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 110);
   *   chassis.pid_wait();
   *
   *   // This will not slew because we haven't set the default backward state
   *   chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
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
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Set the default slew state to true
   *   chassis.slew_turn_set(true);
   *
   *   // This will not use slew because we explicitly told the robot to not slew
   *   chassis.pid_turn_set(90_deg, 90, false);
   *   chassis.pid_wait();
   *
   *   // This will slew because it's the default state
   *   chassis.pid_turn_set(0_deg, 90);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void slew_turn_set(bool slew_on);

  /**
   * Returns true if slew is enabled for all turn motions, false otherwise.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Set the default slew state to true
   *   chassis.slew_turn_set(true);
   *   if (chassis.slew_turn_get()) printf("Turn Slew is Enabled!\n");
   *
   *   // This will not use slew because we explicitly told the robot to not slew
   *   chassis.pid_turn_set(90_deg, 90, false);
   *   chassis.pid_wait();
   *
   *   // This will slew because it's the default state
   *   chassis.pid_turn_set(0_deg, 90);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  bool slew_turn_get();

  /**
   * Allows slew to reenable when the new input speed is larger than the current speed during pure pursuits.  Defaults to true.
   *
   * \param reenable
   *        true enables, false disables
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Turn off slew reenabling, it is on by default
   *   chassis.slew_odom_reenable(false);
   *
   *   // This will slew from the start to 110,
   *   // and once the speed dips down to 50 it won't slew again up to 90
   *   chassis.pid_odom_set(
   *       {
   *           {{6_in, 10_in}, ez::fwd, 110},
   *           {{0_in, 20_in}, ez::fwd, 50},
   *           {{0_in, 30_in}, ez::fwd, 90},
   *       },
   *       true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void slew_odom_reenable(bool reenable);

  /**
   * Returns if slew will reenable when the new input speed is larger than the current speed during pure pursuits.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Allow slew to reenable when speed dips too low
   *   chassis.slew_odom_reenable(true);
   *   if (chassis.slew_odom_reenabled()) printf("Slew will Reenable in Odom!\n");
   *
   *   // This will slew from the start to 110,
   *   // and once the speed dips down to 50 it'll slew again up to 90
   *   chassis.pid_odom_set(
   *       {
   *           {{6_in, 10_in}, ez::fwd, 110},
   *           {{0_in, 20_in}, ez::fwd, 50},
   *           {{0_in, 30_in}, ez::fwd, 90},
   *       },
   *       true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  bool slew_odom_reenabled();

  /**
   * Current mode of the drive.
   */
  e_mode mode = DISABLE;

  /**
   * Sets the current mode of the drive.
   *
   * When your brain is connected to a competition switch or field control, EZ-Template sets the drive mode to ez::DISABLE while the robot is disabled,
   * and once when autonomous ends.  This stops a motion that field control cut off from resuming on its own when driver control starts.  It does not
   * happen when entering autonomous, and it never happens without a competition switch, so testing at your desk or running autons from the brain menu is
   * not affected.
   *
   * \param p_mode
   *        the current task running for the drive.  one of ez::DISABLE, ez::SWING, ez::TURN, ez::TURN_TO_POINT, ez::DRIVE, ez::POINT_TO_POINT, ez::PURE_PURSUIT
   * \param stop_drive
   *        if the drive motors stop when p_mode is ez::DISABLE.  Defaults to true
   *
   * \code{.cpp}
   * const int DRIVE_SPEED = 110;
   *
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_drive_set(12_in, DRIVE_SPEED);
   *   chassis.pid_wait();
   *
   *   // Disable drive
   *   // drive_set() also sets ez::DISABLE, this line just makes the intent clear
   *   chassis.drive_mode_set(ez::DISABLE);
   *
   *   chassis.drive_set(-127, -127);  // Run drive motors myself
   *   pros::delay(2000);
   *   chassis.drive_set(0, 0);
   * }
   * \endcode
   */
  void drive_mode_set(e_mode p_mode, bool stop_drive = true);

  /**
   * Returns the current drive mode that the task is running.
   *
   * Returns ez::DISABLE, ez::SWING, ez::TURN, ez::TURN_TO_POINT, ez::DRIVE, ez::POINT_TO_POINT, or ez::PURE_PURSUIT.
   *
   * \code{.cpp}
   * const int DRIVE_SPEED = 110;
   *
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_drive_set(12_in, DRIVE_SPEED);
   *   chassis.pid_wait();
   *
   *   if (chassis.interfered) chassis.drive_mode_set(ez::DISABLE);
   *
   *   if (chassis.drive_mode_get() == ez::DISABLE) {
   *     chassis.drive_set(-127, -127);  // Run drive motors myself
   *     pros::delay(2000);
   *     chassis.drive_set(0, 0);
   *   }
   * }
   * \endcode
   */
  e_mode drive_mode_get();

  /**
   * Runs opcontrol_curve_sd_initialize(), drive_imu_calibrate() and drive_sensor_reset().
   *
   * \param run_loading_animation
   *        false skips the loading animation on the brain screen while the IMU calibrates
   *
   * \code{.cpp}
   * void initialize() { chassis.initialize(); }
   * \endcode
   */
  void initialize(bool run_loading_animation = true);

  /**
   * Tasks for autonomous.
   */
  pros::Task ez_auto;

  /**
   * Creates a Drive Controller for driver control only. No IMU is configured, so this will not work for autonomous routines:
   * PID driving, turning, swinging, and odometry all need an IMU.
   *
   * Intended for brand new users and short-term setups (classrooms, camps) where getting
   * a drivetrain moving matters more than tuned autonomous routines. Switch to the
   * constructor below once you're ready to add an IMU and autonomous movements.
   *
   * \param left_motor_ports
   *        input {1, -2...}. make ports negative if reversed
   * \param right_motor_ports
   *        input {-3, 4...}. make ports negative if reversed
   *
   * \code{.cpp}
   * ez::Drive chassis(
   *     // These are your drive motors, the first motor is used for sensing!
   *     {-5, -6, -7, -8},   // Left Chassis Ports (negative port will reverse it!)
   *     {11, 15, 16, 17});  // Right Chassis Ports
   * \endcode
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports);

  /**
   * Creates a Drive Controller using internal encoders.
   *
   * If your drivetrain has external gearing (a transmission, or a wheel gear that
   * differs from the motor gear), set `ticks` to your wheel's effective RPM
   * (cartridge RPM * (motor gear / wheel gear)) so tracking still reads distances
   * correctly. If your gearing changes at runtime (a shifting transmission), call
   * drive_rpm_set() with the new gear's wheel RPM whenever you shift. You can do that at any
   * time, even in the middle of a motion: odom and the motion carry on from where the robot is.
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
   *        the wheel's RPM: cartridge RPM * (motor gear / wheel gear). A 600 RPM cartridge with a 36 tooth motor
   *        gear driving a 48 tooth wheel gear is 600 * (36 / 48) = 450. With no external gearing it is just the
   *        cartridge RPM (100, 200 or 600).  Get it wrong and every distance the robot reports is off by the same factor.
   *
   * \code{.cpp}
   * ez::Drive chassis(
   *     // These are your drive motors, the first motor is used for sensing!
   *     {-5, -6, -7, -8},  // Left Chassis Ports (negative port will reverse it!)
   *     {11, 15, 16, 17},  // Right Chassis Ports (negative port will reverse it!)
   *     21,                // IMU Port
   *
   *     // Wheel Diameter (Remember, 4" wheels without screw holes are actually
   *     // 4.125!)
   *     4.125,
   *
   *     // Wheel RPM = cartridge RPM * (motor gear / wheel gear)
   *     // eg. if your drive is 84:36 where the 36t is powered,
   *     //     cartridge 600 * (36/84) = 257
   *     // eg. if your drive is 36:60 where the 60t is powered,
   *     //     cartridge 600 * (60/36) = 1000
   *     420.0);
   * \endcode
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks);

  /**
   * Creates a Drive Controller using internal encoders with redundant IMUs.
   *
   * If your drivetrain has external gearing (a transmission, or a wheel gear that
   * differs from the motor gear), set `ticks` to your wheel's effective RPM
   * (cartridge RPM * (motor gear / wheel gear)) so tracking still reads distances
   * correctly. If your gearing changes at runtime (a shifting transmission), call
   * drive_rpm_set() with the new gear's wheel RPM whenever you shift. You can do that at any
   * time, even in the middle of a motion: odom and the motion carry on from where the robot is.
   *
   * \param left_motor_ports
   *        input {1, -2...}. make ports negative if reversed
   * \param right_motor_ports
   *        input {-3, 4...}. make ports negative if reversed
   * \param imu_ports
   *        input {5, 6...}. multiple IMU ports
   * \param wheel_diameter
   *        diameter of your drive wheels
   * \param ticks
   *        the wheel's RPM: cartridge RPM * (motor gear / wheel gear). A 600 RPM cartridge with a 36 tooth motor
   *        gear driving a 48 tooth wheel gear is 600 * (36 / 48) = 450. With no external gearing it is just the
   *        cartridge RPM (100, 200 or 600).  Get it wrong and every distance the robot reports is off by the same factor.
   *
   * \code{.cpp}
   * ez::Drive chassis(
   *     // These are your drive motors, the first motor is used for sensing!
   *     {-5, -6, -7, -8},  // Left Chassis Ports (negative port will reverse it!)
   *     {11, 15, 16, 17},  // Right Chassis Ports (negative port will reverse it!)
   *
   *     {21, 12},  // IMU Ports, in order of preference
   *
   *     // Wheel Diameter (Remember, 4" wheels without screw holes are actually
   *     // 4.125!)
   *     4.125,
   *
   *     // Wheel RPM = cartridge RPM * (motor gear / wheel gear)
   *     420.0);
   * \endcode
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, std::vector<int> imu_ports, double wheel_diameter, double ticks);

  // What the compiler says about either six-argument constructor. One string for both so they cannot drift apart.
#define EZ_DRIVE_RATIO_ARG_DEPRECATED \
  "Drive's 6th argument (ratio) was removed in 4.0. Pass the wheel RPM instead: cartridge_rpm / ratio. 600 and 1.667 becomes 360: (..., 3.25, 600, 1.667) -> (..., 3.25, 360)"
  /**
   * Compatibility only: the constructor 3.x and 4.0 beta.1 to beta.3 had, with a `ratio` as the sixth argument.
   * It exists so those projects still compile, and it will be removed in a later major version.
   *
   * To migrate, fold the ratio into the wheel RPM and drop the last argument: `ticks = cartridge_rpm / ratio`. For example,
   * 600 and 1.667 becomes 360: `Drive(left, right, 21, 3.25, 600, 1.667)` becomes `Drive(left, right, 21, 3.25, 360)`.
   * Deleting the last argument without changing `ticks` makes every distance `ratio` times too long.
   *
   * It gives exactly the result of the migrated call: `ticks / ratio` goes to the five-argument constructor. Like that
   * constructor, it does not check its numbers, so a `ratio` of 0 reads as infinite ticks (every distance reads 0) and a
   * negative `ratio` reads distances backwards.
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
   *        motor cartridge RPM (it becomes `ticks / ratio`, the wheel RPM)
   * \param ratio
   *        wheel gear / motor gear, as in 3.x
   */
  [[deprecated(EZ_DRIVE_RATIO_ARG_DEPRECATED)]]
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio);

  /**
   * Compatibility only: the redundant IMU constructor 3.x and 4.0 beta.1 to beta.3 had, with a `ratio` as the sixth
   * argument. It exists so those projects still compile, and it will be removed in a later major version.
   *
   * To migrate, fold the ratio into the wheel RPM and drop the last argument: `ticks = cartridge_rpm / ratio`. For example,
   * 600 and 1.667 becomes 360: `Drive(left, right, {21, 20}, 3.25, 600, 1.667)` becomes `Drive(left, right, {21, 20}, 3.25, 360)`.
   * It gives exactly the result of the migrated call, and like the five-argument constructor it does not check its numbers
   * (a `ratio` of 0 reads as infinite ticks, a negative one reads distances backwards).
   *
   * \param left_motor_ports
   *        input {1, -2...}. make ports negative if reversed
   * \param right_motor_ports
   *        input {-3, 4...}. make ports negative if reversed
   * \param imu_ports
   *        input {5, 6...}. multiple IMU ports
   * \param wheel_diameter
   *        diameter of your drive wheels
   * \param ticks
   *        motor cartridge RPM (it becomes `ticks / ratio`, the wheel RPM)
   * \param ratio
   *        wheel gear / motor gear, as in 3.x
   */
  [[deprecated(EZ_DRIVE_RATIO_ARG_DEPRECATED)]]
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, std::vector<int> imu_ports, double wheel_diameter, double ticks, double ratio);
#undef EZ_DRIVE_RATIO_ARG_DEPRECATED

  // Deconstructor
  ~Drive();

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
   * Enables / disables tracking.  Tracking is enabled by default.
   *
   * \param input
   *        true enables tracking, false disables tracking
   *
   * \code{.cpp}
   * void initialize() {
   *   // Print our branding over your terminal :D
   *   ez::ez_template_print();
   *
   *   pros::delay(500);  // Stop the user from doing anything while legacy ports configure
   *
   *   // Disable tracking
   *   chassis.odom_enable(false);
   * }
   * \endcode
   */
  void odom_enable(bool input);

  /**
   * Returns whether the bot is tracking with odometry.
   *
   * True means tracking is enabled, false means tracking is disabled.
   *
   * \code{.cpp}
   * void initialize() {
   *   // Print our branding over your terminal :D
   *   ez::ez_template_print();
   *
   *   pros::delay(500);  // Stop the user from doing anything while legacy ports configure
   *
   *   // Enable tracking
   *   chassis.odom_enable(true);
   *
   *   if (chassis.odom_enabled()) printf("Odom is enabled!\n");
   * }
   * \endcode
   */
  bool odom_enabled();

  /**
   * Sets the width of the drive.
   *
   * Odometry uses this for a side that has no parallel tracking wheel (it is tracking with that side's drive
   * motors).  A side with an odom_tracker_left / odom_tracker_right uses that tracking wheel's distance to
   * center instead.
   *
   * \param input
   *        a unit in inches, from center of the wheel to center of the wheel
   */
  void drive_width_set(double input);

  /**
   * Sets the width of the drive.
   *
   * Odometry uses this for a side that has no parallel tracking wheel (it is tracking with that side's drive
   * motors).  A side with an odom_tracker_left / odom_tracker_right uses that tracking wheel's distance to
   * center instead.
   *
   * \param p_input
   *        a unit, from center of the wheel to center of the wheel
   */
  void drive_width_set(ez::QLength p_input);

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
   *        new x coordinate as a unit
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_odom_set({{24_in, 0_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   chassis.odom_x_set(0_in);  // Set current x to 0
   *
   *   // This will go back to the starting location
   *   chassis.pid_odom_set({{-24_in, 0_in}, ez::rev, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void odom_x_set(ez::QLength p_x);

  /**
   * Returns the current X coordinate of the robot in inches.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_odom_set({{24_in, 24_in, 45_deg}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   printf("X: %.2f  Y: %.2f  T: %.2f\n", chassis.odom_x_get(), chassis.odom_y_get(),
   *          chassis.odom_theta_get());
   * }
   * \endcode
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
   *        new y coordinate as a unit
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_odom_set({{0_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   chassis.odom_y_set(0_in);  // Set current y to 0
   *
   *   // This will go back to the starting location
   *   chassis.pid_odom_set({{0_in, -24_in}, ez::rev, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void odom_y_set(ez::QLength p_y);

  /**
   * Returns the current Y coordinate of the robot in inches.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_odom_set({{24_in, 24_in, 45_deg}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   printf("X: %.2f  Y: %.2f  T: %.2f\n", chassis.odom_x_get(), chassis.odom_y_get(),
   *          chassis.odom_theta_get());
   * }
   * \endcode
   */
  double odom_y_get();

  /**
   * Sets the current angle of the robot.
   *
   * This sets the IMU's heading and moves the heading PID's target to it, so it is for setting the heading at a known
   * moment (the start of a match, a reset against a wall).  Do not call it in a loop: every call overwrites the IMU's
   * heading and the heading target of whatever motion is running.  To correct position in a loop, use odom_xy_set().
   *
   * \param a
   *        new angle in degrees
   */
  void odom_theta_set(double a);

  /**
   * Sets the current Theta of the robot.
   *
   * \param p_a
   *        new angle as a unit
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_turn_set(90_deg, 110);
   *   chassis.pid_wait();
   *
   *   chassis.odom_theta_set(0_deg);  // Set current theta to 0
   *
   *   // This will turn back to the starting heading
   *   chassis.pid_turn_set(-90_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void odom_theta_set(ez::QAngle p_a);

  /**
   * Returns the current Theta of the robot in degrees.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_odom_set({{24_in, 24_in, 45_deg}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   printf("X: %.2f  Y: %.2f  T: %.2f\n", chassis.odom_x_get(), chassis.odom_y_get(),
   *          chassis.odom_theta_get());
   * }
   * \endcode
   */
  double odom_theta_get();

  /**
   * Sets the current pose of the robot.
   *
   * If t is left out, only x and y are set and the heading is left as it is.
   * Setting t is a heading set, see odom_theta_set(): do not do it in a loop.
   *
   * \param itarget
   *        {x, y, t} units in inches and degrees
   */
  void odom_pose_set(pose itarget);

  /**
   * Sets the current pose of the robot.
   *
   * If t is left out, only x and y are set and the heading is left as it is.
   *
   * \param itarget
   *        {x, y, t} as a unit
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_odom_set({{24_in, 24_in, 45_deg}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   // Set the position to 0, 0 and keep the heading the robot is really at, so only x and y shift
   *   ez::united_pose new_pose = {0_in, 0_in, 45_deg};
   *   chassis.odom_pose_set(new_pose);
   *
   *   // This will go back to the starting location
   *   chassis.pid_odom_set({{-24_in, -24_in}, ez::rev, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void odom_pose_set(united_pose itarget);

  /**
   * Sets the current X and Y coordinate for the robot.
   *
   * This leaves the heading alone, so it is the one to call repeatedly, for example to correct position from a GPS every
   * pass.  A custom tracking function can also call it to write its pose.
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
   *        new x value, unit
   * \param p_y
   *        new y value, unit
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   chassis.odom_xy_set(0_in, 0_in);  // Set current x and y to 0
   *
   *   // This will go back to the starting location
   *   chassis.pid_odom_set({{-24_in, -24_in}, ez::rev, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void odom_xy_set(ez::QLength p_x, ez::QLength p_y);

  /**
   * Sets the current X, Y, and Theta values for the robot.
   *
   * This sets the IMU's heading and moves the heading PID's target to it, so it is for setting the heading at a known
   * moment (the start of a match, a reset against a wall).  Do not call it in a loop: every call overwrites the IMU's
   * heading and the heading target of whatever motion is running.  To correct position in a loop, use odom_xy_set().
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
   *        new x value, unit
   * \param p_y
   *        new y value, unit
   * \param p_t
   *        new theta value, unit
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_odom_set({{24_in, 24_in, 45_deg}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   // Set the position to 0, 0 and keep the heading the robot is really at, so only x and y shift
   *   chassis.odom_xyt_set(0_in, 0_in, 45_deg);
   *
   *   // This will go back to the starting location
   *   chassis.pid_odom_set({{-24_in, -24_in}, ez::rev, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void odom_xyt_set(ez::QLength p_x, ez::QLength p_y, ez::QAngle p_t);

  /**
   * Returns the current pose of the robot.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_odom_set({{24_in, 24_in, 45_deg}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   ez::pose c_pose = chassis.odom_pose_get();
   *   printf("X: %.2f  Y: %.2f  T: %.2f\n", c_pose.x, c_pose.y, c_pose.theta);
   * }
   * \endcode
   */
  pose odom_pose_get();

  /**
   * Resets xyt to 0.
   */
  void odom_reset();

  /**
   * Sets a new dlead.
   *
   * Dlead is a proportional value of how much to make the robot curve during boomerang motions.
   *
   * \param input
   *        a value between 0 and 1
   *
   * \code{.cpp}
   * void initialize() { chassis.odom_boomerang_dlead_set(0.5); }
   * \endcode
   */
  void odom_boomerang_dlead_set(double input);

  /**
   * Returns the current dlead.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.odom_boomerang_dlead_set(0.5);
   *   printf("dlead: %.2f\n", chassis.odom_boomerang_dlead_get());
   * }
   * \endcode
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
   * \param p_distance
   *        distance as a unit
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.odom_boomerang_dlead_set(0.5);
   *   chassis.odom_boomerang_distance_set(12_in);
   * }
   * \endcode
   */
  void odom_boomerang_distance_set(ez::QLength p_distance);

  /**
   * Returns the maximum distance the carrot point can be away from the target point, in inches.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.odom_boomerang_dlead_set(0.5);
   *   chassis.odom_boomerang_distance_set(12_in);
   *   printf("Max Distance: %.2f\n", chassis.odom_boomerang_distance_get());
   * }
   * \endcode
   */
  double odom_boomerang_distance_get();

  /**
   * A proportion of how prioritized turning is during odometry motions.
   *
   * Turning is prioritized so the robot "applies brakes" while turning.  Lower number means more braking.
   * Values below 1 make the robot stop driving once turning has gone past a certain angle error, and the
   * robot will never drive backwards to reach the point.  The internal scale factor is clamped to [0, 1].
   * Non-positive values are rejected, a message is printed to the terminal, and the previous value is kept, since this is used as a divisor.
   *
   * \param bias
   *        a positive number, default is 0.9
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.odom_turn_bias_set(1.0);  // Set turn bias to 1
   *
   *   // Go to 24, 24
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   // Reset your angle and position
   *   chassis.pid_turn_set(0_deg, 110);
   *   chassis.pid_wait();
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);
   *
   *   chassis.odom_turn_bias_set(0.5);  // Set turn bias to 0.5
   *
   *   // Go to 24, 24 relative to where the robot ended, but with a new turn bias
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void odom_turn_bias_set(double bias);

  /**
   * Returns the proportion of how prioritized turning is during odometry motions.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.odom_turn_bias_set(1.0);  // Set turn bias to 1
   *   printf("Turn Bias: %.2f\n", chassis.odom_turn_bias_get());
   *
   *   // Go to 24, 24
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   // Reset your angle and position
   *   chassis.pid_turn_set(0_deg, 110);
   *   chassis.pid_wait();
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);
   *
   *   chassis.odom_turn_bias_set(0.5);  // Set turn bias to 0.5
   *   printf("Turn Bias: %.2f\n", chassis.odom_turn_bias_get());
   *
   *   // Go to 24, 24 relative to where the robot ended, but with a new turn bias
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  double odom_turn_bias_get();

  /**
   * Sets the spacing between points when points get injected into the path.
   *
   * \param spacing
   *        a small number in inches.  Must be greater than 0.  Otherwise it is rejected, a message is printed to the terminal, and the previous value is kept
   */
  void odom_path_spacing_set(double spacing);

  /**
   * Sets the spacing between points when points get injected into the path.
   *
   * \param p_spacing
   *        a small number in units.  Must be greater than 0.  Otherwise it is rejected, a message is printed to the terminal, and the previous value is kept
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.odom_path_spacing_set(0.5_in);  // Set path spacing to 0.5 inches
   *   printf("Path Spacing: %.2f\n", chassis.odom_path_spacing_get());
   *
   *   // Go to 24, 24
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void odom_path_spacing_set(ez::QLength p_spacing);

  /**
   * Returns the spacing between points when points get injected into the path.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.odom_path_spacing_set(0.5_in);  // Set path spacing to 0.5 inches
   *   printf("Path Spacing: %.2f\n", chassis.odom_path_spacing_get());
   *
   *   // Go to 24, 24
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  double odom_path_spacing_get();

  /**
   * Sets the constants for smoothing out a path.
   *
   * Path smoothing based on https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
   *
   * weight_data + 2 * weight_smooth must also be less than 2.  Smoothing makes repeated passes over the points, and at 2 or more each pass moves them
   * further from the path instead of settling, so it would never finish.  With the defaults that's 1.53.
   *
   * If any of these are out of range, all of the constants are rejected, a message is printed to the terminal naming the first one that is out of range,
   * and the previous constants are kept.
   *
   * \param weight_smooth
   *        how much weight to smooth the coordinates, 0 or more and less than 1
   * \param weight_data
   *        how much weight to keep the coordinates near the original path, 0 or more
   * \param tolerance
   *        how much change per iteration is necessary to keep iterating, greater than 0
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_drive_toggle(false);  // Disable the drive
   *   chassis.pid_print_toggle(false);  // Disable movement printing
   *
   *   // Print default smooth constants
   *   std::vector<double> smooth_consts = chassis.odom_path_smooth_constants_get();
   *   printf("Weight Smooth: %.4f   Weight Data: %.4f   Tolerance: %.4f\n", smooth_consts[0],
   *          smooth_consts[1], smooth_consts[2]);
   *   chassis.pid_odom_set(
   *       {
   *           {{0_in, 24_in}, ez::fwd, 110},
   *           {{24_in, 24_in}, ez::fwd, 110},
   *       },
   *       false);
   *   chassis.odom_path_print();  // Print the full path to terminal
   *
   *   // Print updated smooth constants
   *   chassis.odom_path_smooth_constants_set(0.5, 0.003, 0.0001);
   *   smooth_consts = chassis.odom_path_smooth_constants_get();
   *   printf("Weight Smooth: %.4f   Weight Data: %.4f   Tolerance: %.4f\n", smooth_consts[0],
   *          smooth_consts[1], smooth_consts[2]);
   *   chassis.pid_odom_set(
   *       {
   *           {{0_in, 24_in}, ez::fwd, 110},
   *           {{24_in, 24_in}, ez::fwd, 110},
   *       },
   *       false);
   *   chassis.odom_path_print();  // Print the full path to terminal
   * }
   * \endcode
   */
  void odom_path_smooth_constants_set(double weight_smooth, double weight_data, double tolerance);

  /**
   * Returns the constants for smoothing out a path.
   *
   * In order of:
   *  - weight_smooth
   *  - weight_data
   *  - tolerance
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_drive_toggle(false);  // Disable the drive
   *   chassis.pid_print_toggle(false);  // Disable movement printing
   *
   *   // Print default smooth constants
   *   std::vector<double> smooth_consts = chassis.odom_path_smooth_constants_get();
   *   printf("Weight Smooth: %.4f   Weight Data: %.4f   Tolerance: %.4f\n", smooth_consts[0],
   *          smooth_consts[1], smooth_consts[2]);
   *   chassis.pid_odom_set(
   *       {
   *           {{0_in, 24_in}, ez::fwd, 110},
   *           {{24_in, 24_in}, ez::fwd, 110},
   *       },
   *       false);
   *   chassis.odom_path_print();  // Print the full path to terminal
   *
   *   // Print updated smooth constants
   *   chassis.odom_path_smooth_constants_set(0.5, 0.003, 0.0001);
   *   smooth_consts = chassis.odom_path_smooth_constants_get();
   *   printf("Weight Smooth: %.4f   Weight Data: %.4f   Tolerance: %.4f\n", smooth_consts[0],
   *          smooth_consts[1], smooth_consts[2]);
   *   chassis.pid_odom_set(
   *       {
   *           {{0_in, 24_in}, ez::fwd, 110},
   *           {{24_in, 24_in}, ez::fwd, 110},
   *       },
   *       false);
   *   chassis.odom_path_print();  // Print the full path to terminal
   * }
   * \endcode
   */
  std::vector<double> odom_path_smooth_constants_get();

  /**
   * Prints the current path the robot is following.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_drive_toggle(false);  // Disable the drive
   *   chassis.pid_print_toggle(false);  // Disable movement printing
   *
   *   // Print default smooth constants
   *   std::vector<double> smooth_consts = chassis.odom_path_smooth_constants_get();
   *   printf("Weight Smooth: %.4f   Weight Data: %.4f   Tolerance: %.4f\n", smooth_consts[0],
   *          smooth_consts[1], smooth_consts[2]);
   *   chassis.pid_odom_set(
   *       {
   *           {{0_in, 24_in}, ez::fwd, 110},
   *           {{24_in, 24_in}, ez::fwd, 110},
   *       },
   *       false);
   *   chassis.odom_path_print();  // Print the full path to terminal
   *
   *   // Print updated smooth constants
   *   chassis.odom_path_smooth_constants_set(0.5, 0.003, 0.0001);
   *   smooth_consts = chassis.odom_path_smooth_constants_get();
   *   printf("Weight Smooth: %.4f   Weight Data: %.4f   Tolerance: %.4f\n", smooth_consts[0],
   *          smooth_consts[1], smooth_consts[2]);
   *   chassis.pid_odom_set(
   *       {
   *           {{0_in, 24_in}, ez::fwd, 110},
   *           {{24_in, 24_in}, ez::fwd, 110},
   *       },
   *       false);
   *   chassis.odom_path_print();  // Print the full path to terminal
   * }
   * \endcode
   */
  void odom_path_print();

  /**
   * Sets how far away the robot looks in the path during pure pursuits.
   *
   * \param distance
   *        how long the "carrot on a stick" is, in inches.  Must be greater than 0.  Otherwise it is rejected, a message is printed to the
   *        terminal, and the previous value is kept
   */
  void odom_look_ahead_set(double distance);

  /**
   * Sets how far away the robot looks in the path during pure pursuits.
   *
   * \param p_distance
   *        how long the "carrot on a stick" is, in units.  Must be greater than 0.  Otherwise it is rejected, a message is printed to the
   *        terminal, and the previous value is kept
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.odom_look_ahead_set(7_in);  // Set look ahead to 7in
   *
   *   // Go to 24, 24
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   // Reset your angle and position
   *   chassis.pid_turn_set(0_deg, 110);
   *   chassis.pid_wait();
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);
   *
   *   chassis.odom_look_ahead_set(14_in);  // Set look ahead to 14in
   *
   *   // Go to 24, 24 relative to where the robot ended, but with a new look ahead
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void odom_look_ahead_set(ez::QLength p_distance);

  /**
   * Returns how far away the robot looks in the path during pure pursuits.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.odom_look_ahead_set(7_in);  // Set look ahead to 7in
   *   printf("Look Ahead: %.2f\n", chassis.odom_look_ahead_get());
   *
   *   // Go to 24, 24
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   // Reset your angle and position
   *   chassis.pid_turn_set(0_deg, 110);
   *   chassis.pid_wait();
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);
   *
   *   chassis.odom_look_ahead_set(14_in);  // Set look ahead to 14in
   *   printf("Look Ahead: %.2f\n", chassis.odom_look_ahead_get());
   *
   *   // Go to 24, 24 relative to where the robot ended, but with a new look ahead
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  double odom_look_ahead_get();

  /**
   * Sets the parallel left tracking wheel for odometry.
   *
   * \param input
   *        an ez::tracking_wheel
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);  // ADI Encoders
   *
   * // ADI Encoders plugged into a Smart port
   * ez::tracking_wheel left_tracker(1, {'C', 'D'}, 2.75, 4.0);
   *
   * // Rotation sensors
   * ez::tracking_wheel horiz_tracker(2, 2.75, 4.0);
   *
   * void initialize() {
   *   // Print our branding over your terminal :D
   *   ez::ez_template_print();
   *
   *   pros::delay(500);  // Stop the user from doing anything while legacy ports configure
   *
   *   chassis.odom_tracker_right_set(&right_tracker);
   *   chassis.odom_tracker_left_set(&left_tracker);
   *
   *   // Replace `back` with `front` if your tracker is in the front!
   *   chassis.odom_tracker_back_set(&horiz_tracker);
   * }
   * \endcode
   */
  void odom_tracker_left_set(tracking_wheel* input);

  /**
   * Sets the parallel right tracking wheel for odometry.
   *
   * \param input
   *        an ez::tracking_wheel
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);  // ADI Encoders
   *
   * // ADI Encoders plugged into a Smart port
   * ez::tracking_wheel left_tracker(1, {'C', 'D'}, 2.75, 4.0);
   *
   * // Rotation sensors
   * ez::tracking_wheel horiz_tracker(2, 2.75, 4.0);
   *
   * void initialize() {
   *   // Print our branding over your terminal :D
   *   ez::ez_template_print();
   *
   *   pros::delay(500);  // Stop the user from doing anything while legacy ports configure
   *
   *   chassis.odom_tracker_right_set(&right_tracker);
   *   chassis.odom_tracker_left_set(&left_tracker);
   *
   *   // Replace `back` with `front` if your tracker is in the front!
   *   chassis.odom_tracker_back_set(&horiz_tracker);
   * }
   * \endcode
   */
  void odom_tracker_right_set(tracking_wheel* input);

  /**
   * Sets the perpendicular front tracking wheel for odometry.
   *
   * \param input
   *        an ez::tracking_wheel
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);  // ADI Encoders
   *
   * // ADI Encoders plugged into a Smart port
   * ez::tracking_wheel left_tracker(1, {'C', 'D'}, 2.75, 4.0);
   *
   * // Rotation sensors
   * ez::tracking_wheel horiz_tracker(2, 2.75, 4.0);
   *
   * void initialize() {
   *   // Print our branding over your terminal :D
   *   ez::ez_template_print();
   *
   *   pros::delay(500);  // Stop the user from doing anything while legacy ports configure
   *
   *   chassis.odom_tracker_right_set(&right_tracker);
   *   chassis.odom_tracker_left_set(&left_tracker);
   *
   *   // Replace `front` with `back` if your tracker is in the back!
   *   chassis.odom_tracker_front_set(&horiz_tracker);
   * }
   * \endcode
   */
  void odom_tracker_front_set(tracking_wheel* input);

  /**
   * Sets the perpendicular back tracking wheel for odometry.
   *
   * \param input
   *        an ez::tracking_wheel
   *
   * \code{.cpp}
   * ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);  // ADI Encoders
   *
   * // ADI Encoders plugged into a Smart port
   * ez::tracking_wheel left_tracker(1, {'C', 'D'}, 2.75, 4.0);
   *
   * // Rotation sensors
   * ez::tracking_wheel horiz_tracker(2, 2.75, 4.0);
   *
   * void initialize() {
   *   // Print our branding over your terminal :D
   *   ez::ez_template_print();
   *
   *   pros::delay(500);  // Stop the user from doing anything while legacy ports configure
   *
   *   chassis.odom_tracker_right_set(&right_tracker);
   *   chassis.odom_tracker_left_set(&left_tracker);
   *
   *   // Replace `back` with `front` if your tracker is in the front!
   *   chassis.odom_tracker_back_set(&horiz_tracker);
   * }
   * \endcode
   */
  void odom_tracker_back_set(tracking_wheel* input);

  /**
   * Sets the default behavior for turns in odom, swinging, and turning.  The default is ez::shortest.
   *
   * \param behavior
   *        ez::shortest, ez::longest, ez::ccw, ez::cw, ez::raw
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_angle_behavior_set(ez::longest);  // Set the robot to take the longest path there
   *
   *   // This will make the robot go the long way around to get to 90 degrees
   *   chassis.pid_turn_set(90_deg, 110);
   *   chassis.pid_wait();
   *
   *   // This will make the robot go the long way around to get to 0 degrees
   *   chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 110);
   *   chassis.pid_wait();
   *
   *   // This will make the robot go the long way around to get to 24, 0
   *   chassis.pid_odom_set({{24_in, 0_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_angle_behavior_set(e_angle_behavior behavior);

  /**
   * Sets the default behavior for turns in turning movements.  The default is ez::shortest.
   *
   * \param behavior
   *        ez::shortest, ez::longest, ez::ccw, ez::cw, ez::raw
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_turn_behavior_set(ez::longest);  // Set the robot to take the longest path there
   *
   *   // This will make the robot go the long way around to get to 90 degrees
   *   chassis.pid_turn_set(90_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_turn_behavior_set(e_angle_behavior behavior);

  /**
   * Sets the default behavior for turns in swinging movements.  The default is ez::shortest.
   *
   * \param behavior
   *        ez::shortest, ez::longest, ez::ccw, ez::cw, ez::raw
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_swing_behavior_set(ez::longest);  // Set the robot to take the longest path there
   *
   *   // This will make the robot go the long way around to get to 90 degrees
   *   chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_swing_behavior_set(e_angle_behavior behavior);

  /**
   * Sets the default behavior for the turn in odom movements.  The default is ez::shortest.  An odom movement that sets its own turn_behavior uses that
   * instead, and one that leaves it unset follows this.
   *
   * \param behavior
   *        ez::shortest, ez::longest, ez::ccw, ez::cw, ez::raw
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_odom_behavior_set(ez::longest);  // Set the robot to take the longest path there
   *
   *   // This will make the robot go the long way around to get to 24, 0
   *   chassis.pid_odom_set({{24_in, 0_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_odom_behavior_set(e_angle_behavior behavior);

  /**
   * Returns the default behavior for turns, ez::shortest unless changed.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_turn_behavior_set(ez::longest);  // Set the robot to take the longest path there
   *
   *   if (chassis.pid_turn_behavior_get() == ez::longest) {
   *     printf("Current behavior is ez::longest!\n");
   *   } else {
   *     printf("Current behavior is not ez::longest!\n");
   *   }
   *
   *   // This will make the robot go the long way around to get to 90 degrees
   *   chassis.pid_turn_set(90_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  e_angle_behavior pid_turn_behavior_get();

  /**
   * Returns the default behavior for swings, ez::shortest unless changed.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_swing_behavior_set(ez::longest);  // Set the robot to take the longest path there
   *
   *   if (chassis.pid_swing_behavior_get() == ez::longest) {
   *     printf("Current behavior is ez::longest!\n");
   *   } else {
   *     printf("Current behavior is not ez::longest!\n");
   *   }
   *
   *   // This will make the robot go the long way around to get to 90 degrees
   *   chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  e_angle_behavior pid_swing_behavior_get();

  /**
   * Returns the default behavior for the turn in odom movements, ez::shortest unless changed.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_odom_behavior_set(ez::longest);  // Set the robot to take the longest path there
   *
   *   if (chassis.pid_odom_behavior_get() == ez::longest) {
   *     printf("Current behavior is ez::longest!\n");
   *   } else {
   *     printf("Current behavior is not ez::longest!\n");
   *   }
   *
   *   // This will make the robot go the long way around to get to 24, 0
   *   chassis.pid_odom_set({{24_in, 0_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  e_angle_behavior pid_odom_behavior_get();

  /**
   * Gives some wiggle room in shortest vs longest, so a 180.1 and 179.9 degree turns have consistent behavior.
   *
   * \param p_tolerance
   *        angle wiggle room, a unit
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_angle_behavior_tolerance_set(3_deg);  // Set the behavior tolerance to 3 degrees
   *
   *   // A turn within the tolerance above uses this behavior by default
   *   chassis.pid_angle_behavior_bias_set(ez::cw);
   *   chassis.odom_theta_set(-1_deg);
   *
   *   // Even though the fastest way here is to go counter clockwise,
   *   // the robot will go clockwise
   *   chassis.pid_turn_set(180_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_angle_behavior_tolerance_set(ez::QAngle p_tolerance);

  /**
   * Gives some wiggle room in shortest vs longest, so a 180.1 and 179.9 degree turns have consistent behavior.
   *
   * \param tolerance
   *        angle wiggle room, in degrees
   */
  void pid_angle_behavior_tolerance_set(double tolerance);

  /**
   * Returns the wiggle room in shortest vs longest, so a 180.1 and 179.9 degree turns have consistent behavior.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_angle_behavior_tolerance_set(3_deg);  // Set the behavior tolerance to 3 degrees
   *
   *   // A turn within the tolerance above uses this behavior by default
   *   chassis.pid_angle_behavior_bias_set(ez::cw);
   *   printf("Tolerance is: %.2f\n", chassis.pid_angle_behavior_tolerance_get());
   *   chassis.odom_theta_set(-1_deg);
   *
   *   // Even though the fastest way here is to go counter clockwise,
   *   // the robot will go clockwise
   *   chassis.pid_turn_set(180_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  double pid_angle_behavior_tolerance_get();

  /**
   * When a turn is within its tolerance, you can have it bias left or right.
   *
   * \param behavior
   *        ez::ccw or ez::cw
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_angle_behavior_tolerance_set(3_deg);  // Set the behavior tolerance to 3 degrees
   *
   *   // A turn within the tolerance above uses this behavior by default
   *   chassis.pid_angle_behavior_bias_set(ez::cw);
   *   chassis.odom_theta_set(-1_deg);
   *
   *   // Even though the fastest way here is to go counter clockwise,
   *   // the robot will go clockwise
   *   chassis.pid_turn_set(180_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_angle_behavior_bias_set(e_angle_behavior behavior);

  /**
   * Returns the behavior when a turn is within its tolerance, you can have it bias left or right.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_angle_behavior_tolerance_set(3_deg);
   *   chassis.pid_angle_behavior_bias_set(ez::cw);
   *
   *   if (chassis.pid_angle_behavior_bias_get() == ez::cw) {
   *     printf("Behavior bias is cw!\n");
   *   } else {
   *     printf("Behavior bias is not cw!\n");
   *   }
   *
   *   chassis.odom_theta_set(-1_deg);
   *
   *   // Even though the fastest way here is to go counter clockwise,
   *   // the robot will go clockwise
   *   chassis.pid_turn_set(180_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
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
   * This passes the controller through the joystick curves.  A curve of 0 is no curve, which is the default.
   * The controller buttons that change the curves are disabled by default, use opcontrol_curve_buttons_toggle(true)
   * to turn them on.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void opcontrol_tank();

  /**
   * Sets the chassis to controller joysticks using standard arcade control, where left stick is fwd/rev.
   * Run in usercontrol.
   *
   * This passes the controller through the joystick curves.  A curve of 0 is no curve, which is the default.
   * The controller buttons that change the curves are disabled by default, use opcontrol_curve_buttons_toggle(true)
   * to turn them on.
   *
   * \param stick_type
   *        ez::SINGLE or ez::SPLIT control
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_arcade_standard(ez::SPLIT);  // For split arcade
   *
   *     // For single arcade
   *     // chassis.opcontrol_arcade_standard(ez::SINGLE);
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void opcontrol_arcade_standard(e_type stick_type);

  /**
   * Sets the chassis to controller joysticks using flipped arcade control, where right stick is fwd/rev.
   * Run in usercontrol.
   *
   * This passes the controller through the joystick curves.  A curve of 0 is no curve, which is the default.
   * The controller buttons that change the curves are disabled by default, use opcontrol_curve_buttons_toggle(true)
   * to turn them on.
   *
   * \param stick_type
   *        ez::SINGLE or ez::SPLIT control
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_arcade_flipped(ez::SPLIT);  // For split arcade
   *
   *     // For single arcade
   *     // chassis.opcontrol_arcade_flipped(ez::SINGLE);
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void opcontrol_arcade_flipped(e_type stick_type);

  /**
   * Sets the chassis to controller joysticks using standard curvature control, where left stick is fwd/rev.
   * Run in usercontrol.
   *
   * Curvature is like arcade, but the turn stick sets the curvature of the arc the robot drives instead of how fast it turns.  The robot follows the
   * same arc at any speed, like a steering wheel.
   *
   * Because the turn is scaled by how fast you're driving, the robot would not be able to turn while stopped.  opcontrol_curvature_point_turn_gain_set()
   * controls how much the robot can turn on a point.  See Control Schemes
   * (https://ez-robotics.github.io/EZ-Template/tutorials/control_schemes#curvature) for more on how it feels.
   *
   * This passes the controller through the joystick curves.  A curve of 0 is no curve, which is the default.
   * The controller buttons that change the curves are disabled by default, use opcontrol_curve_buttons_toggle(true)
   * to turn them on.
   *
   * \param stick_type
   *        ez::SINGLE or ez::SPLIT control
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_arcade_curvature_standard(ez::SPLIT);  // For split curvature
   *
   *     // For single curvature
   *     // chassis.opcontrol_arcade_curvature_standard(ez::SINGLE);
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void opcontrol_arcade_curvature_standard(e_type stick_type);

  /**
   * Sets the chassis to controller joysticks using flipped curvature control, where right stick is fwd/rev.
   * Run in usercontrol.
   *
   * Curvature is like arcade, but the turn stick sets the curvature of the arc the robot drives instead of how fast it turns.  The robot follows the
   * same arc at any speed, like a steering wheel.
   *
   * Because the turn is scaled by how fast you're driving, the robot would not be able to turn while stopped.  opcontrol_curvature_point_turn_gain_set()
   * controls how much the robot can turn on a point.  See Control Schemes
   * (https://ez-robotics.github.io/EZ-Template/tutorials/control_schemes#curvature) for more on how it feels.
   *
   * This passes the controller through the joystick curves.  A curve of 0 is no curve, which is the default.
   * The controller buttons that change the curves are disabled by default, use opcontrol_curve_buttons_toggle(true)
   * to turn them on.
   *
   * \param stick_type
   *        ez::SINGLE or ez::SPLIT control
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_arcade_curvature_flipped(ez::SPLIT);  // For split curvature
   *
   *     // For single curvature
   *     // chassis.opcontrol_arcade_curvature_flipped(ez::SINGLE);
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void opcontrol_arcade_curvature_flipped(e_type stick_type);

  /**
   * Sets how much the robot can turn when it is barely moving fwd/rev in curvature control.
   *
   * Curvature scales the turn stick by how fast you're driving, so at 0 fwd/rev speed the robot could not turn at all.  To fix this, the turn is scaled by
   * whichever is larger; your fwd/rev speed (as a fraction of full speed) or this gain.  There is no jump between the two, and no button is needed to turn
   * on a point.
   *
   * 0 is pure curvature, the robot cannot turn on a point.  1 is the same as arcade.  Anything in between turns on a point at a reduced sensitivity, and
   * becomes true curvature once your fwd/rev speed is above `gain * 127`.
   *
   * This is defaulted to 0.8.  Values outside of 0 - 1 are clamped.
   *
   * \param gain
   *        0 - 1
   *
   * \code{.cpp}
   * void initialize() {
   *   // . . .
   *   chassis.opcontrol_curvature_point_turn_gain_set(0.5);  // Turn on a point at half sensitivity
   *
   *   // . . .
   * }
   * \endcode
   */
  void opcontrol_curvature_point_turn_gain_set(double gain);

  /**
   * Returns how much the robot can turn when it is barely moving fwd/rev in curvature control.
   *
   * \code{.cpp}
   * void initialize() {
   *   // . . .
   *
   *   printf("Point turn gain: %.2f\n", chassis.opcontrol_curvature_point_turn_gain_get());
   *
   *   // . . .
   * }
   * \endcode
   */
  double opcontrol_curvature_point_turn_gain_get();

  /**
   * Initializes left and right curves with the SD card.  chassis.initialize() runs this for you, call it yourself only if you skip chassis.initialize().
   * Missing curve files are created.
   *
   * A value in left_curve.txt or right_curve.txt that is outside of 0 to 20 is changed to the nearest end of that range, and the terminal prints what it
   * was changed to.  If a file can't be read as a number, the current curve is kept.
   *
   * \code{.cpp}
   * void initialize() {
   *   // Only needed when chassis.initialize() is not called
   *   chassis.opcontrol_curve_sd_initialize();
   * }
   * \endcode
   */
  void opcontrol_curve_sd_initialize();

  /**
   * Sets the default joystick curves.  Curves are kept between 0 and 20, a value outside of that is changed to the nearest end.  0 is no curve.
   *
   * This also writes the curves to the SD card, so remove this line if you tune curves with the controller buttons and an SD card.  In tank only the
   * left curve is used.
   *
   * \param left
   *        left default curve
   * \param right
   *        right default curve, defaults to 0
   *
   * \code{.cpp}
   * void initialize() { chassis.opcontrol_curve_default_set(2, 2); }
   * \endcode
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
   *        error threshold to start integral, 0 means i always accumulates
   *
   * \code{.cpp}
   * void initialize() { chassis.opcontrol_drive_activebrake_set(2.0); }
   * \endcode
   */
  void opcontrol_drive_activebrake_set(double kp, double ki = 0.0, double kd = 0.0, double start_i = 0.0);

  /**
   * Returns kP for active brake.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.opcontrol_drive_activebrake_set(0.1);
   *   printf("kP is: %f\n", chassis.opcontrol_drive_activebrake_get());
   * }
   * \endcode
   */
  double opcontrol_drive_activebrake_get();

  /**
   * Returns all PID constants for active brake.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.opcontrol_drive_activebrake_set(0.1);
   *
   *   ez::PID::Constants consts = chassis.opcontrol_drive_activebrake_constants_get();
   *
   *   printf("kP is: %f\n", consts.kp);
   * }
   * \endcode
   */
  PID::Constants opcontrol_drive_activebrake_constants_get();

  /**
   * Enables/disables modifying the joystick input curves with the controller.  This is disabled by default.
   *
   * \param toggle
   *        true enables, false disables
   *
   * \code{.cpp}
   * void initialize() { chassis.opcontrol_curve_buttons_toggle(true); }
   * \endcode
   */
  void opcontrol_curve_buttons_toggle(bool toggle);

  /**
   * Gets the current state of the toggle. Enables/disables modifying the joystick input curves with the controller.
   *
   * True enabled, false disabled.
   *
   * \code{.cpp}
   * void initialize() {
   *   printf("Enabled? %i\n", chassis.opcontrol_curve_buttons_toggle_get());  // Returns false
   *
   *   chassis.opcontrol_curve_buttons_toggle(true);
   *   printf("Enabled? %i\n", chassis.opcontrol_curve_buttons_toggle_get());  // Returns true
   * }
   * \endcode
   */
  bool opcontrol_curve_buttons_toggle_get();

  /**
   * Sets buttons for modifying the left joystick curve.  The curve stops at 0 and at 20 when you hold the buttons.
   *
   * \param decrease
   *        a pros button enumerator
   * \param increase
   *        a pros button enumerator
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT,
   *                                            pros::E_CONTROLLER_DIGITAL_RIGHT);
   * }
   * \endcode
   */
  void opcontrol_curve_buttons_left_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

  /**
   * Returns a vector of pros controller buttons used for the left joystick curve, in {decrease, increase}
   */
  std::vector<pros::controller_digital_e_t> opcontrol_curve_buttons_left_get();

  /**
   * Sets buttons for modifying the right joystick curve.  The curve stops at 0 and at 20 when you hold the buttons.
   *
   * \param decrease
   *        a pros button enumerator
   * \param increase
   *        a pros button enumerator
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y,
   *                                             pros::E_CONTROLLER_DIGITAL_A);
   * }
   * \endcode
   */
  void opcontrol_curve_buttons_right_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

  /**
   * Returns a vector of pros controller buttons used for the right joystick curve, in {decrease, increase}
   */
  std::vector<pros::controller_digital_e_t> opcontrol_curve_buttons_right_get();

  /**
   * Outputs a curve from 5225A In the Zone.
   *
   * This gives more control over the robot at lower speeds.
   *
   * 5225A explained this curve here: https://www.vexforum.com/t/team-5225a-in-the-zone-code-release-yes-you-read-that-right/63199/10
   * and here is a graph of it: https://www.desmos.com/calculator/7oyvwwpmed
   *
   * \param x
   *        joystick input
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     int l_stick = chassis.opcontrol_curve_left(master.get_analog(ANALOG_LEFT_Y));
   *     int r_stick = chassis.opcontrol_curve_left(master.get_analog(ANALOG_RIGHT_Y));
   *
   *     chassis.opcontrol_joystick_threshold_iterate(l_stick, r_stick);
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double opcontrol_curve_left(double x);

  /**
   * Outputs a curve from 5225A In the Zone.
   *
   * This gives more control over the robot at lower speeds.
   *
   * 5225A explained this curve here: https://www.vexforum.com/t/team-5225a-in-the-zone-code-release-yes-you-read-that-right/63199/10
   * and here is a graph of it: https://www.desmos.com/calculator/7oyvwwpmed
   *
   * \param x
   *        joystick input
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     int l_stick = chassis.opcontrol_curve_left(master.get_analog(ANALOG_LEFT_Y));
   *     int r_stick = chassis.opcontrol_curve_right(master.get_analog(ANALOG_RIGHT_Y));
   *
   *     chassis.opcontrol_joystick_threshold_iterate(l_stick, r_stick);
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double opcontrol_curve_right(double x);

  /**
   * Sets a new threshold for the joystick.
   *
   * The joysticks will not return a value if they are within this.
   *
   * The default is 3.  A controller's stick at rest often reads 1 or 2 instead of 0, and active brake only runs while
   * both sticks read exactly 0, so with a threshold of 0 those sticks keep active brake off.  Set this to 0 to
   * pass every stick value through, including 1 and 2.
   *
   * \param threshold
   *        new threshold
   *
   * \code{.cpp}
   * void initialize() { chassis.opcontrol_joystick_threshold_set(5); }
   * \endcode
   */
  void opcontrol_joystick_threshold_set(int threshold);

  /**
   * Gets the threshold for the joystick.
   */
  int opcontrol_joystick_threshold_get();

  /**
   * Sets the active brake targets to the current drive sensor values, once after autonomous has run.  Called by the opcontrol_ drive functions.  Does not reset
   * any sensors.
   */
  void opcontrol_drive_sensors_reset();

  /**
   * Sets the left drive to l_stick and the right drive to r_stick, and runs active brake when both are 0.
   *
   * This applies practice mode, opcontrol_drive_reverse_set() and the opcontrol max speed.  It does not apply opcontrol_joystick_threshold_set(), the
   * opcontrol_ drive functions apply that before calling this.
   *
   * \param l_stick
   *        left drive input, -127 to 127
   * \param r_stick
   *        right drive input, -127 to 127
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     // Apply the joystick threshold first, so active brake can run when the sticks are at rest
   *     int l_stick = master.get_analog(ANALOG_LEFT_Y);
   *     if (abs(l_stick) < chassis.opcontrol_joystick_threshold_get()) l_stick = 0;
   *     int r_stick = master.get_analog(ANALOG_RIGHT_Y);
   *     if (abs(r_stick) < chassis.opcontrol_joystick_threshold_get()) r_stick = 0;
   *
   *     chassis.opcontrol_joystick_threshold_iterate(l_stick, r_stick);
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
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
   *
   * \code{.cpp}
   * void initialize() {
   *   pros::delay(500);
   *
   *   // This prints 0 0
   *   printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]),
   *          chassis.pto_check(chassis.right_motors[1]));
   *   chassis.pto_add({chassis.left_motors[1], chassis.right_motors[1]});
   *   // This prints 1 1
   *   printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]),
   *          chassis.pto_check(chassis.right_motors[1]));
   * }
   * \endcode
   */
  bool pto_check(pros::Motor check_if_pto);

  /**
   * Adds motors to the pto list, removing them from the drive.
   *
   * You cannot add the first index because it's used for autonomous.  A motor that can't be added, or is already
   * in the list, is skipped and the rest of pto_list is still added.
   *
   * \param pto_list
   *        list of motors to remove from the drive
   *
   * \code{.cpp}
   * void initialize() {
   *   pros::delay(500);
   *
   *   // This prints 0 0
   *   printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]),
   *          chassis.pto_check(chassis.right_motors[1]));
   *   chassis.pto_add({chassis.left_motors[1], chassis.right_motors[1]});
   *   // This prints 1 1
   *   printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]),
   *          chassis.pto_check(chassis.right_motors[1]));
   * }
   * \endcode
   */
  void pto_add(std::vector<pros::Motor> pto_list);

  /**
   * Removes motors from the pto list, adding them to the drive.
   *
   * A motor that isn't in the pto list is skipped and the rest of pto_list is still removed.
   *
   * \param pto_list
   *        list of motors to add to the drive
   *
   * \code{.cpp}
   * void initialize() {
   *   pros::delay(500);
   *
   *   // This prints 0 0
   *   printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]),
   *          chassis.pto_check(chassis.right_motors[1]));
   *   chassis.pto_add({chassis.left_motors[1], chassis.right_motors[1]});
   *   // This prints 1 1
   *   printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]),
   *          chassis.pto_check(chassis.right_motors[1]));
   *   chassis.pto_remove({chassis.left_motors[1], chassis.right_motors[1]});
   *   // This prints 0 0
   *   printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]),
   *          chassis.pto_check(chassis.right_motors[1]));
   * }
   * \endcode
   */
  void pto_remove(std::vector<pros::Motor> pto_list);

  /**
   * Moves motors between the drive and the pto list.
   *
   * You cannot add the first index because it's used for autonomous.
   *
   * \param pto_list
   *        list of motors to add/remove from the drive
   * \param toggle
   *        true removes the motors from the drive and adds them to the pto list, false removes them from the pto list and adds them back to the drive
   *
   * \code{.cpp}
   * ez::Piston pto_intake_piston('A');
   *
   * void pto_intake(bool toggle) {
   *   chassis.pto_toggle({chassis.left_motors[1], chassis.right_motors[1]}, toggle);
   *   pto_intake_piston.set(toggle);
   *   if (toggle) {
   *     chassis.left_motors[1].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   *     chassis.right_motors[1].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   *   }
   * }
   * \endcode
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
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.drive_set(127, 127);
   *   pros::delay(1000);  // Wait 1 second
   *   chassis.drive_set(0, 0);
   * }
   * \endcode
   */
  void drive_set(int left, int right);

  /**
   * Gets the chassis voltage, -127 to 127.  Returns {left, right}.
   */
  std::vector<int> drive_get();

  /**
   * Changes the way the drive behaves when it is not under active user control.
   *
   * \param brake_type
   *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
   *
   * \code{.cpp}
   * void initialize() { chassis.drive_brake_set(MOTOR_BRAKE_COAST); }
   * \endcode
   */
  void drive_brake_set(pros::motor_brake_mode_e_t brake_type);

  /**
   * Returns the brake mode of the drive as a pros::motor_brake_mode_e_t.
   */
  pros::motor_brake_mode_e_t drive_brake_get();

  /**
   * Sets the limit for the current on the drive.
   *
   * Values above 2500 mA are capped at 2500.  Motors in the pto list are not changed.
   *
   * \param mA
   *        input in milliamps
   *
   * \code{.cpp}
   * void initialize() { chassis.drive_current_limit_set(1000); }
   * \endcode
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
   *
   * \code{.cpp}
   * const int DRIVE_SPEED = 110;
   *
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_drive_set(12_in, DRIVE_SPEED);
   *   chassis.pid_wait();
   *
   *   chassis.pid_drive_toggle(false);  // Disable drive
   *   chassis.pid_drive_set(-12_in, DRIVE_SPEED);
   *   while (true) {
   *     printf(" Left Error: %f  Right Error: %f\n", chassis.leftPID.error, chassis.rightPID.error);
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
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
   *
   * \code{.cpp}
   * const int DRIVE_SPEED = 110;
   *
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_drive_set(12_in, DRIVE_SPEED);  // This will print
   *   chassis.pid_wait();                         // This will print
   *
   *   chassis.pid_print_toggle(false);  // Disable prints
   *
   *   chassis.pid_drive_set(-12_in, DRIVE_SPEED);  // This won't print
   *   chassis.pid_wait();                          // This won't print
   * }
   * \endcode
   */
  void pid_print_toggle(bool toggle);

  /**
   * Gets the current state of the toggle.
   *
   * True enabled, false disabled.
   */
  bool pid_print_toggle_get();

  /**
   * Sets the button to increment the focused PID constant.
   *
   * \param increase
   *        a pros controller button
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_tuner_button_increment_set(DIGITAL_R1); }
   * \endcode
   */
  void pid_tuner_button_increment_set(pros::controller_digital_e_t increase);

  /**
   * Returns the pros button that increments the focused PID constant.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_tuner_button_increment_set(DIGITAL_UP);
   *   printf("%i\n", chassis.pid_tuner_button_increment_get());  // Prints 10 (DIGITAL_UP)
   * }
   * \endcode
   */
  pros::controller_digital_e_t pid_tuner_button_increment_get();

  /**
   * Sets the button to decrement the focused PID constant.
   *
   * \param decrease
   *        a pros controller button
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_tuner_button_decrement_set(DIGITAL_R2); }
   * \endcode
   */
  void pid_tuner_button_decrement_set(pros::controller_digital_e_t decrease);

  /**
   * Returns the pros button that decrements the focused PID constant.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_tuner_button_decrement_set(DIGITAL_DOWN);
   *   printf("%i\n", chassis.pid_tuner_button_decrement_get());  // Prints 11 (DIGITAL_DOWN)
   * }
   * \endcode
   */
  pros::controller_digital_e_t pid_tuner_button_decrement_get();

  /**
   * Sets the button to move up between the PID constants.
   *
   * \param pageUp
   *        a pros controller button
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_tuner_button_up_set(DIGITAL_L1); }
   * \endcode
   */
  void pid_tuner_button_up_set(pros::controller_digital_e_t pageUp);

  /**
   * Returns the pros button that moves up between the PID constants.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_tuner_button_up_set(DIGITAL_Y);
   *   printf("%i\n", chassis.pid_tuner_button_up_get());  // Prints 16 (DIGITAL_Y)
   * }
   * \endcode
   */
  pros::controller_digital_e_t pid_tuner_button_up_get();

  /**
   * Sets the button to move down between the PID constants.
   *
   * \param pageDown
   *        a pros controller button
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_tuner_button_down_set(DIGITAL_L2); }
   * \endcode
   */
  void pid_tuner_button_down_set(pros::controller_digital_e_t pageDown);

  /**
   * Returns the pros button that moves down between the PID constants.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_tuner_button_down_set(DIGITAL_A);
   *   printf("%i\n", chassis.pid_tuner_button_down_get());  // Prints 17 (DIGITAL_A)
   * }
   * \endcode
   */
  pros::controller_digital_e_t pid_tuner_button_down_get();

  /**
   * Sets the button to go left in the PID Tuner.
   *
   * \param pageLeft
   *        a pros controller button
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_tuner_button_left_set(DIGITAL_LEFT); }
   * \endcode
   */
  void pid_tuner_button_left_set(pros::controller_digital_e_t pageLeft);

  /**
   * Returns the pros button that goes left in the PID Tuner.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_tuner_button_left_set(DIGITAL_LEFT);
   *   printf("%i\n", chassis.pid_tuner_button_left_get());  // Prints 12 (DIGITAL_LEFT)
   * }
   * \endcode
   */
  pros::controller_digital_e_t pid_tuner_button_left_get();

  /**
   * Sets the button to go right in the PID Tuner.
   *
   * \param pageRight
   *        a pros controller button
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_tuner_button_right_set(DIGITAL_RIGHT); }
   * \endcode
   */
  void pid_tuner_button_right_set(pros::controller_digital_e_t pageRight);

  /**
   * Returns the pros button that goes right in the PID Tuner.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_tuner_button_right_set(DIGITAL_RIGHT);
   *   printf("%i\n", chassis.pid_tuner_button_right_get());  // Prints 13 (DIGITAL_RIGHT)
   * }
   * \endcode
   */
  pros::controller_digital_e_t pid_tuner_button_right_get();

  /////
  //
  // Telemetry
  //
  /////

  /**
   * The position of the right sensor in inches.
   *
   * If you have two parallel tracking wheels, this will return tracking wheel position.  Otherwise this returns motor position.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();
   *
   *     printf("Right Sensor: %f \n", chassis.drive_sensor_right());
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double drive_sensor_right();

  /**
   * The position of the right sensor.
   *
   * If you have two parallel tracking wheels, this will return tracking wheel position.  Otherwise this returns motor position.
   *
   * On a failed sensor read, returns the last successfully-read raw value instead of the
   * PROS_ERR/PROS_ERR_F sentinel the underlying read failed with.
   *
   * This is the raw count since the last reset, in encoder ticks. It does not change when drive_rpm_set()
   * changes what a tick is worth, so after one of those it is no longer a fixed multiple of drive_sensor_right().
   */
  int drive_sensor_right_raw();

  /**
   * The velocity of the right motor.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();
   *
   *     printf("Right Velocity: %i \n", chassis.drive_velocity_right());
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  int drive_velocity_right();

  /**
   * The current draw of the first right motor, in milliamps.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();
   *
   *     printf("Right mA: %f \n", chassis.drive_mA_right());
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double drive_mA_right();

  /**
   * Return true when the motor is over current.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();
   *
   *     printf("Right Over Current: %i \n", chassis.drive_current_right_over());
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  bool drive_current_right_over();

  /**
   * The position of the left sensor in inches.
   *
   * If you have two parallel tracking wheels, this will return tracking wheel position.  Otherwise this returns motor position.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();
   *
   *     printf("Left Sensor: %f \n", chassis.drive_sensor_left());
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double drive_sensor_left();

  /**
   * The position of the left sensor.
   *
   * If you have two parallel tracking wheels, this will return tracking wheel position.  Otherwise this returns motor position.
   *
   * On a failed sensor read, returns the last successfully-read raw value instead of the
   * PROS_ERR/PROS_ERR_F sentinel the underlying read failed with.
   *
   * This is the raw count since the last reset, in encoder ticks. It does not change when drive_rpm_set()
   * changes what a tick is worth, so after one of those it is no longer a fixed multiple of drive_sensor_left().
   */
  int drive_sensor_left_raw();

  /**
   * The velocity of the left motor.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();
   *
   *     printf("Left Velocity: %i \n", chassis.drive_velocity_left());
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  int drive_velocity_left();

  /**
   * The current draw of the first left motor, in milliamps.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();
   *
   *     printf("Left mA: %f \n", chassis.drive_mA_left());
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double drive_mA_left();

  /**
   * Return true when the motor is over current.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();
   *
   *     printf("Left Over Current: %i \n", chassis.drive_current_left_over());
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  bool drive_current_left_over();

  /**
   * Reset all the chassis motors and tracking wheels, recommended to run at the start of your autonomous routine.
   *
   * \code{.cpp}
   * const int DRIVE_SPEED = 110;
   *
   * void autonomous() {
   *   // Reset drive sensors to 0
   *   chassis.drive_sensor_reset();
   *
   *   // Drive forward from the new 0
   *   chassis.pid_drive_set(24_in, DRIVE_SPEED);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void drive_sensor_reset();

  /**
   * Resets the current imu value.  Defaults to 0, recommended to run at the start of your autonomous routine.
   *
   * Also sets the heading odom_theta_get() reads, so an odom motion started right after a reset starts from the new heading.
   *
   * \param new_heading
   *        new heading value
   *
   * \code{.cpp}
   * const int TURN_SPEED = 90;
   *
   * void autonomous() {
   *   // Reset gyro position to 0
   *   chassis.drive_imu_reset();
   *
   *   // Turn from the new 0
   *   chassis.pid_turn_set(90_deg, TURN_SPEED);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void drive_imu_reset(double new_heading = 0);

  /**
   * Returns the current imu rotation value in degrees.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();
   *
   *     printf("Gyro: %f \n", chassis.drive_imu_get());
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double drive_imu_get();

  /**
   * Returns the size of the imu's acceleration in the x-y plane, sqrt(x^2 + y^2), so it is never negative.
   * Returns NaN if there is no imu, not 0 -- 0 would read as "not accelerating".
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();
   *
   *     printf("Accel: %f \n", chassis.drive_imu_accel_get());
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double drive_imu_accel_get();

  /**
   * Calibrates the imu's scale using a physical turn.
   *
   * Physically turn the robot 3600 degrees (10 full rotations) and pass in
   * what the imu reported for that turn.  Internally, this is used to divide
   * the imu's raw reading so it reports the true 3600.
   *
   * A value under 100, which includes any negative value, is rejected and the previous scale is kept.  See the IMU Scaling tutorial
   * (https://ez-robotics.github.io/EZ-Template/tutorials/tuning_imu_scale) for the full calibration walkthrough.
   *
   * \param imu_value_after_3600
   *        what the imu reads after physically turning the robot 3600 degrees
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.initialize();
   *
   *   // After physically turning the robot 3600 degrees and reading what the
   *   // imu reported, pass that value in here.
   *   chassis.drive_imu_scaler_3600_set(3625.42);
   * }
   * \endcode
   */
  void drive_imu_scaler_3600_set(double imu_value_after_3600);

  /**
   * Returns the imu value after a 3600 degree turn that produces the imu's current scale.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.initialize();
   *   chassis.drive_imu_scaler_3600_set(3625.42);
   *   printf("%.2f\n", chassis.drive_imu_scaler_3600_get());  // Prints 3625.42
   * }
   * \endcode
   */
  double drive_imu_scaler_3600_get();

  std::map<int, double> imu_scale_map;
  std::map<int, std::pair<double, int>> prev_imu_values;

  /**
   * Calibrates the scale of all IMUs using a physical turn.  Works with one or more IMUs.  Values past the number of IMUs are ignored.
   *
   * Physically turn the robot 3600 degrees (10 full rotations) and pass in
   * what each imu reported for that turn.
   *
   * A value under 100, which includes any negative value, is rejected and that imu's previous scale is kept.
   *
   * \param imu_values_after_3600
   *        what each imu reads after physically turning the robot 3600 degrees, in the same order as the IMU ports passed to the constructor, input {3550,
   *        3625...}
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.initialize();
   *
   *   // After physically turning the robot 3600 degrees and reading what each
   *   // imu reported, pass those values in here, in the same order as the imu
   *   // ports passed to the constructor.
   *   chassis.drive_imus_scalers_3600_set({3550.0, 3625.0});
   * }
   * \endcode
   */
  void drive_imus_scalers_3600_set(std::vector<double> imu_values_after_3600);

  /**
   * Returns the imu value after a 3600 degree turn that produces each imu's current scale.  Works with one or more IMUs.
   *
   * Returned as a map of imu port to value.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.drive_imus_scalers_3600_set({3550.0, 3625.0});
   *
   *   for (auto& [port, value] : chassis.drive_imus_scalers_3600_get())
   *     printf("port %i: %.2f\n", port, value);
   * }
   * \endcode
   */
  std::map<int, double> drive_imus_scalers_3600_get();

  /**
   * Sets how many degrees apart the most and least agreeing good IMUs' scaled readings may spread before it's reported as disagreement, for drives built with
   * the redundant IMU constructor.
   *
   * The spread has to hold for about half a second to rule out a single noisy sample.  This only reports, in imu_drift_deg and in
   * ez::health::preflight().  It never ejects an IMU or changes which one drives the heading, because with only two IMUs there is no way to tell which
   * one drifted.  The default is 15 degrees.
   *
   * \param degrees
   *        disagreement threshold in degrees.  A non-positive value is rejected and the previous threshold is kept
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.initialize();
   *   chassis.imu_drift_threshold_set(10.0);
   * }
   * \endcode
   */
  void imu_drift_threshold_set(double degrees);

  /**
   * Returns the current IMU disagreement threshold in degrees, see imu_drift_threshold_set().
   *
   * \code{.cpp}
   * void initialize() { printf("drift threshold: %.0f deg\n", chassis.imu_drift_threshold_get()); }
   * \endcode
   */
  double imu_drift_threshold_get();

  /**
   * Degrees between the most and least agreeing good IMU's scaled reading,
   * once that spread has held above imu_drift_threshold_get() for enough
   * consecutive passes to rule out a single noisy sample. 0 while the good
   * IMUs agree, or while there are fewer than 2 of them. Updated every pass
   * by check_imu_task().
   */
  double imu_drift_deg = 0.0;

  /**
   * Calibrates the IMU.  Returns true if at least one IMU calibrated.
   *
   * chassis.initialize() already calls this, only call it yourself to recalibrate.
   *
   * \param run_loading_animation
   *        true runs the animation, false doesn't
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     // Recalibrate the IMU while the robot is sitting still
   *     if (master.get_digital_new_press(DIGITAL_A)) {
   *       bool calibrated = chassis.drive_imu_calibrate();
   *       master.rumble(calibrated ? "." : "---");
   *     }
   *
   *     chassis.opcontrol_tank();
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  bool drive_imu_calibrate(bool run_loading_animation = true);

  /**
   * Checks if the imu calibrated successfully or if it took longer than expected.
   *
   * Returns true if calibrated successfully, and false if unsuccessful.
   *
   * \code{.cpp}
   * void initialize() {
   *   // Print our branding over your terminal :D
   *   ez::ez_template_print();
   *
   *   pros::delay(500);  // Stop the user from doing anything while legacy ports configure
   *
   *   // Initialize chassis and auton selector
   *   chassis.initialize();
   *   ez::as::initialize();
   *
   *   master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
   * }
   * \endcode
   */
  bool drive_imu_calibrated();

  /**
   * Loading display while the IMU calibrates.
   *
   * \param iter
   *        milliseconds since calibration started
   */
  void drive_imu_display_loading(int iter);

  /**
   * Returns the angle of the robot, from whichever IMU is currently focused.
   *
   * On a redundant IMU drive, this is the one to read instead of talking to an individual pros::Imu directly, since it stays correct across a failover.
   *
   * \code{.cpp}
   * void initialize() { printf("%.2f\n", chassis.drive_angle_get()); }
   * \endcode
   */
  double drive_angle_get();

  /**
   * Practice mode for driver practice that shuts off the drive if either side of the drive is asked to go faster
   * than 120 (out of 127).  In arcade this is checked after the forward and turn sticks are combined.
   *
   * \param toggle
   *        true enables, false disables
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // Toggle practice mode
   *     if (master.get_digital_new_press(DIGITAL_L1)) {
   *       chassis.opcontrol_joystick_practicemode_toggle(
   *           !chassis.opcontrol_joystick_practicemode_toggle_get());
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void opcontrol_joystick_practicemode_toggle(bool toggle);

  /**
   * Gets current state of the toggle.
   *
   * True is enabled, false is disabled.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // Toggle practice mode
   *     if (master.get_digital_new_press(DIGITAL_L1)) {
   *       chassis.opcontrol_joystick_practicemode_toggle(
   *           !chassis.opcontrol_joystick_practicemode_toggle_get());
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  bool opcontrol_joystick_practicemode_toggle_get();

  /**
   * Slow mode for driver practice that scales the drive by opcontrol_joystick_slowmode_speed_set() / 127,
   * on top of opcontrol_speed_max_set() (the two multiply together, they don't replace each other), instead
   * of cutting the drive off like opcontrol_joystick_practicemode_toggle() does.  This also scales down
   * active brake's holding power while a joystick is released, the same way opcontrol_speed_max_set() does.
   * Meant as a training mode, not something to leave on for a competition match.
   *
   * \param toggle
   *        true enables, false disables
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // Toggle slow mode
   *     if (master.get_digital_new_press(DIGITAL_L1)) {
   *       chassis.opcontrol_joystick_slowmode_toggle(!chassis.opcontrol_joystick_slowmode_toggle_get());
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void opcontrol_joystick_slowmode_toggle(bool toggle);

  /**
   * Gets current state of the toggle.
   *
   * True is enabled, false is disabled.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // Toggle slow mode
   *     if (master.get_digital_new_press(DIGITAL_L1)) {
   *       chassis.opcontrol_joystick_slowmode_toggle(!chassis.opcontrol_joystick_slowmode_toggle_get());
   *     }
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  bool opcontrol_joystick_slowmode_toggle_get();

  /**
   * Sets the speed used while opcontrol_joystick_slowmode_toggle() is enabled.  This multiplies with
   * opcontrol_speed_max_set() rather than overriding it, so the actual cap while slow mode is on is
   * opcontrol_speed_max_set() * speed / 127.
   *
   * \param speed
   *        the speed limit, out of 127
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.opcontrol_joystick_slowmode_speed_set(90);  // A gentler slow mode than the default of 64
   * }
   * \endcode
   */
  void opcontrol_joystick_slowmode_speed_set(int speed);

  /**
   * Returns the speed used while opcontrol_joystick_slowmode_toggle() is enabled.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.opcontrol_joystick_slowmode_speed_set(90);
   *   printf("%d\n", chassis.opcontrol_joystick_slowmode_speed_get());  // Prints 90
   * }
   * \endcode
   */
  int opcontrol_joystick_slowmode_speed_get();

  /**
   * Reversal for drivetrain in opcontrol that flips the left and right side and the direction of the drive.
   *
   * \param toggle
   *        true if you want your drivetrain reversed and false if you do not
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     if (master.get_digital_new_press(DIGITAL_B))
   *       chassis.opcontrol_drive_reverse_set(!chassis.opcontrol_drive_reverse_get());
   *
   *     chassis.opcontrol_tank();
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void opcontrol_drive_reverse_set(bool toggle);

  /**
   * Returns `true` if the drive is currently reversed, and `false` if it isn't.
   *
   * Reversal for drivetrain in opcontrol that flips the left and right side and the direction of the drive.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   while (true) {
   *     if (master.get_digital_new_press(DIGITAL_B))
   *       chassis.opcontrol_drive_reverse_set(!chassis.opcontrol_drive_reverse_get());
   *
   *     chassis.opcontrol_tank();
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  bool opcontrol_drive_reverse_get();

  /////
  //
  // Autonomous Functions
  //
  /////

  /**
   * Sets the robot to move forward using PID without units, only using slew if globally enabled.
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
   * Sets the robot to move forward using PID without units, using slew if enabled for this motion.
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
   * Sets the robot to go forward/backward the distance you give it, but it uses odometry, using slew only if it is enabled globally.
   *
   * \param p_target
   *        distance to go, in a length unit
   * \param speed
   *        0 to 127, max speed during motion.  It's recommended to keep this at 110
   */
  void pid_odom_set(ez::QLength p_target, int speed);

  /**
   * Sets the robot to go forward/backward the distance you give it, but it uses odometry, using slew if enabled for this motion.
   *
   * \param p_target
   *        distance to go, in a length unit
   * \param speed
   *        0 to 127, max speed during motion.  It's recommended to keep this at 110
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Uses the global slew setting
   *   chassis.pid_odom_set(24_in, 110);
   *   chassis.pid_wait();
   *
   *   // Slew on for this motion only
   *   chassis.pid_odom_set(24_in, 110, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_odom_set(ez::QLength p_target, int speed, bool slew_on);

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
   * Takes in an odom movement to go to a single point using point to point.  The angle in the movement is not used to steer (use pid_odom_boomerang_set for
   * that).  Uses slew if globally enabled.
   *
   * \param imovement
   *        {{x, y, t}, fwd/rev, 1-127}  an odom movement
   */
  void pid_odom_ptp_set(odom imovement);

  /**
   * Takes in an odom movement to go to a single point using point to point.  The angle in the movement is not used to steer (use pid_odom_boomerang_set for
   * that).  Uses slew if enabled for this motion.
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
   * \param p_imovement
   *        united_odom, `{{x, y, theta}, ez::fwd or ez::rev, speed 1-127}`, for example `{{24_in, 24_in, 90_deg}, ez::fwd, 110}`
   */
  void pid_odom_boomerang_set(united_odom p_imovement);

  /**
   * Takes in an odom movement to go to a single point using boomerang.  If an angle is set, this will run boomerang.  Uses slew if enabled for this motion.
   *
   * \param p_imovement
   *        united_odom, `{{x, y, theta}, ez::fwd or ez::rev, speed 1-127}`, for example `{{24_in, 24_in, 90_deg}, ez::fwd, 110}`
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Uses the global slew setting
   *   chassis.pid_odom_boomerang_set({{24_in, 24_in, 0_deg}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   // Slew on for this motion only
   *   chassis.pid_odom_boomerang_set({{48_in, 24_in, 0_deg}, ez::fwd, 110}, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_odom_boomerang_set(united_odom p_imovement, bool slew_on);

  /**
   * Takes in an odom movement to go to a single point using point to point.  The angle in the movement is not used to steer (use pid_odom_boomerang_set for
   * that).  Uses slew if globally enabled.
   *
   * \param p_imovement
   *        united_odom, `{{x, y}, ez::fwd or ez::rev, speed 1-127}`, for example `{{24_in, 24_in}, ez::fwd, 110}`
   */
  void pid_odom_ptp_set(united_odom p_imovement);

  /**
   * Takes in an odom movement to go to a single point using point to point.  The angle in the movement is not used to steer (use pid_odom_boomerang_set for
   * that).  Uses slew if enabled for this motion.
   *
   * \param p_imovement
   *        united_odom, `{{x, y}, ez::fwd or ez::rev, speed 1-127}`, for example `{{24_in, 24_in}, ez::fwd, 110}`
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Uses the global slew setting
   *   chassis.pid_odom_ptp_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   // Slew on for this motion only
   *   chassis.pid_odom_ptp_set({{48_in, 24_in}, ez::fwd, 110}, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_odom_ptp_set(united_odom p_imovement, bool slew_on);

  /**
   * Goes to an xy coordinate.  If the point has an angle this runs boomerang, otherwise it runs injected pure pursuit.  This is a wrapper for
   * pid_odom_boomerang_set() and pid_odom_injected_pp_set().  Uses slew if globally enabled.
   *
   * \param p_imovement
   *        united_odom, `{{x, y}, ez::fwd or ez::rev, speed 1-127}`.  Add a theta, `{x, y, theta}`, to run boomerang.
   *        For example `{{24_in, 24_in}, ez::fwd, 110}`
   */
  void pid_odom_set(united_odom p_imovement);

  /**
   * Goes to an xy coordinate.  If the point has an angle this runs boomerang, otherwise it runs injected pure pursuit.  This is a wrapper for
   * pid_odom_boomerang_set() and pid_odom_injected_pp_set().  Uses slew if enabled for this motion.
   *
   * \param p_imovement
   *        united_odom, `{{x, y}, ez::fwd or ez::rev, speed 1-127}`.  Add a theta, `{x, y, theta}`, to run boomerang.
   *        For example `{{24_in, 24_in}, ez::fwd, 110}`
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Uses the global slew setting
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *   chassis.pid_wait();
   *
   *   // Slew on for this motion only
   *   chassis.pid_odom_set({{48_in, 24_in}, ez::fwd, 110}, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_odom_set(united_odom p_imovement, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses
   * slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   */
  void pid_odom_set(std::vector<odom> imovements);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses
   * slew if enabled for this motion.
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
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew
   * if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   */
  void pid_odom_injected_pp_set(std::vector<odom> imovements);

  /**
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew
   * if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_injected_pp_set(std::vector<odom> imovements, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses
   * slew if globally enabled.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   */
  void pid_odom_smooth_pp_set(std::vector<odom> imovements);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses
   * slew if enabled for this motion.
   *
   * \param imovements
   *        {{{x, y, t}, fwd/rev, 1-127}, {{x, y, t}, fwd/rev, 1-127}}  odom movements
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_odom_smooth_pp_set(std::vector<odom> imovements, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses
   * slew if globally enabled.
   *
   * \param p_imovements
   *        list of points, `{{{x, y}, ez::fwd, 110}, {{x, y, theta}, ez::rev, 90}}`.  theta is optional and makes that point boomerang, speed is 1-127
   */
  void pid_odom_smooth_pp_set(std::vector<united_odom> p_imovements);

  /**
   * Takes in odom movements to go through multiple points, will inject and smooth the path.  If an angle is set, this will run boomerang for that point.  Uses
   * slew if enabled for this motion.
   *
   * \param p_imovements
   *        list of points, `{{{x, y}, ez::fwd, 110}, {{x, y, theta}, ez::rev, 90}}`.  theta is optional and makes that point boomerang, speed is 1-127
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Uses the global slew setting
   *   chassis.pid_odom_smooth_pp_set({
   *       {{0_in, 24_in}, ez::fwd, 110},
   *       {{24_in, 24_in}, ez::fwd, 110},
   *   });
   *   chassis.pid_wait();
   *
   *   // Slew on for this motion only
   *   chassis.pid_odom_smooth_pp_set(
   *       {
   *           {{24_in, 48_in}, ez::fwd, 110},
   *           {{48_in, 48_in}, ez::fwd, 110},
   *       },
   *       true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_odom_smooth_pp_set(std::vector<united_odom> p_imovements, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew
   * if globally enabled.
   *
   * \param p_imovements
   *        list of points, `{{{x, y}, ez::fwd, 110}, {{x, y, theta}, ez::rev, 90}}`.  theta is optional and makes that point boomerang, speed is 1-127
   */
  void pid_odom_injected_pp_set(std::vector<united_odom> p_imovements);

  /**
   * Takes in odom movements to go through multiple points, will inject into the path.  If an angle is set, this will run boomerang for that point.  Uses slew
   * if enabled for this motion.
   *
   * \param p_imovements
   *        list of points, `{{{x, y}, ez::fwd, 110}, {{x, y, theta}, ez::rev, 90}}`.  theta is optional and makes that point boomerang, speed is 1-127
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Uses the global slew setting
   *   chassis.pid_odom_injected_pp_set({
   *       {{0_in, 24_in}, ez::fwd, 110},
   *       {{24_in, 24_in}, ez::fwd, 110},
   *   });
   *   chassis.pid_wait();
   *
   *   // Slew on for this motion only
   *   chassis.pid_odom_injected_pp_set(
   *       {
   *           {{24_in, 48_in}, ez::fwd, 110},
   *           {{48_in, 48_in}, ez::fwd, 110},
   *       },
   *       true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_odom_injected_pp_set(std::vector<united_odom> p_imovements, bool slew_on);

  /**
   * Takes in odom movements to go through multiple points.  If an angle is set, this will run boomerang for that point.  Uses slew if globally enabled.
   *
   * \param p_imovements
   *        list of points, `{{{x, y}, ez::fwd, 110}, {{x, y, theta}, ez::rev, 90}}`.  theta is optional and makes that point boomerang, speed is 1-127
   */
  void pid_odom_pp_set(std::vector<united_odom> p_imovements);

  /**
   * Takes in odom movements to go through multiple points.  If an angle is set, this will run boomerang for that point.  Uses slew if enabled for this motion.
   *
   * \param p_imovements
   *        list of points, `{{{x, y}, ez::fwd, 110}, {{x, y, theta}, ez::rev, 90}}`.  theta is optional and makes that point boomerang, speed is 1-127
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Uses the global slew setting
   *   chassis.pid_odom_pp_set({
   *       {{0_in, 24_in}, ez::fwd, 110},
   *       {{24_in, 24_in}, ez::fwd, 110},
   *   });
   *   chassis.pid_wait();
   *
   *   // Slew on for this motion only
   *   chassis.pid_odom_pp_set(
   *       {
   *           {{24_in, 48_in}, ez::fwd, 110},
   *           {{48_in, 48_in}, ez::fwd, 110},
   *       },
   *       true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_odom_pp_set(std::vector<united_odom> p_imovements, bool slew_on);

  /**
   * Create and smooth out a path from the given points.  The path will switch to boomerang if an angle is specified for that point.  This is a
   * wrapper for pid_odom_smooth_pp_set.  Uses slew if globally enabled.
   *
   * \param p_imovements
   *        list of points, `{{{x, y}, ez::fwd, 110}, {{x, y, theta}, ez::rev, 90}}`.  theta is optional and makes that point boomerang, speed is 1-127
   */
  void pid_odom_set(std::vector<united_odom> p_imovements);

  /**
   * Create and smooth out a path from the given points.  The path will switch to boomerang if an angle is specified for that point.  This is a
   * wrapper for pid_odom_smooth_pp_set.  Uses slew if enabled for this motion.
   *
   * \param p_imovements
   *        list of points, `{{{x, y}, ez::fwd, 110}, {{x, y, theta}, ez::rev, 90}}`.  theta is optional and makes that point boomerang, speed is 1-127
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Uses the global slew setting
   *   chassis.pid_odom_set({
   *       {{0_in, 24_in}, ez::fwd, 110},
   *       {{24_in, 24_in}, ez::fwd, 110},
   *   });
   *   chassis.pid_wait();
   *
   *   // Slew on for this motion only
   *   chassis.pid_odom_set(
   *       {
   *           {{24_in, 48_in}, ez::fwd, 110},
   *           {{48_in, 48_in}, ez::fwd, 110},
   *       },
   *       true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_odom_set(std::vector<united_odom> p_imovements, bool slew_on);

  /**
   * Sets the robot to move forward using PID with units, only using slew if globally enabled.
   *
   * \param p_target
   *        target unit
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_drive_set(ez::QLength p_target, int speed);

  /**
   * Sets the robot to move forward using PID with units, using slew if slew_on is true.
   *
   * Pass `slew_on` to turn slew on or off for just this motion.  Use the 2-argument version to follow the global slew setting for the direction you
   * are driving (see `slew_drive_set()`), with heading correction on.
   *
   * \param p_target
   *        target unit
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        true ramps up from a lower speed to your target speed, false doesn't
   * \param toggle_heading
   *        toggle for heading correction.  true enables, false disables
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_drive_set(24_in, 110);  // Uses the global slew setting
   *   chassis.pid_wait();
   *
   *   chassis.pid_drive_set(24_in, 110, true);  // Slew on for this motion only
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_drive_set(ez::QLength p_target, int speed, bool slew_on, bool toggle_heading = true);

  /**
   * Sets the robot to move forward using PID without units, only using slew if globally enabled.
   *
   * \param target
   *        target value in inches
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_drive_set(double target, int speed);

  /**
   * Sets the robot to move forward using PID without units, using slew if enabled for this motion.
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
   * Sets the robot to turn to face a point using PID and odometry.
   *
   * \param itarget
   *        {x, y}  a target point to face
   * \param dir
   *        fwd or rev, which end of the robot faces the point
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_turn_set(pose itarget, drive_directions dir, int speed);

  /**
   * Sets the robot to turn to face a point using PID and odometry.
   *
   * \param itarget
   *        {x, y}  a target point to face
   * \param dir
   *        fwd or rev, which end of the robot faces the point
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(pose itarget, drive_directions dir, int speed, bool slew_on);

  /**
   * Sets the robot to turn to face a point using PID and odometry.
   *
   * \param itarget
   *        {x, y}  a target point to face
   * \param dir
   *        fwd or rev, which end of the robot faces the point
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_turn_set(pose itarget, drive_directions dir, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn to face a point using PID and odometry.
   *
   * \param itarget
   *        {x, y}  a target point to face
   * \param dir
   *        fwd or rev, which end of the robot faces the point
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(pose itarget, drive_directions dir, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn to face a point using PID and odometry.
   *
   * \param p_itarget
   *        {x, y}  a target point to face.  this uses units
   * \param dir
   *        fwd or rev, which end of the robot faces the point
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed);

  /**
   * Sets the robot to turn to face a point using PID and odometry.
   *
   * \param p_itarget
   *        {x, y}  a target point to face.  this uses units
   * \param dir
   *        fwd or rev, which end of the robot faces the point
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, bool slew_on);

  /**
   * Sets the robot to turn to face a point using PID and odometry.
   *
   * \param p_itarget
   *        {x, y}  a target point to face.  this uses units
   * \param dir
   *        fwd or rev, which end of the robot faces the point
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn to face a point using PID and odometry.
   *
   * \param p_itarget
   *        {x, y}  a target point to face.  this uses units
   * \param dir
   *        fwd or rev, which end of the robot faces the point
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Face 12, 12 with the default behavior and slew
   *   chassis.pid_turn_set({12_in, 12_in}, ez::fwd, 110);
   *   chassis.pid_wait();
   *
   *   // Face -12, 12 the long way around
   *   chassis.pid_turn_set({-12_in, 12_in}, ez::fwd, 110, ez::longest);
   *   chassis.pid_wait();
   *
   *   // Face 0, 24 with slew on for this motion only
   *   chassis.pid_turn_set({0_in, 24_in}, ez::fwd, 110, true);
   *   chassis.pid_wait();
   *
   *   // Face 12, 12 the long way around, with slew
   *   chassis.pid_turn_set({12_in, 12_in}, ez::fwd, 110, ez::longest, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn relative to initial heading using PID.
   *
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
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
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
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
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(double target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn relative to initial heading using PID with units.
   *
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_turn_set(ez::QAngle p_target, int speed);

  /**
   * Sets the robot to turn relative to initial heading using PID with units.
   *
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_turn_set(ez::QAngle p_target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn relative to initial heading using PID with units, using slew if enabled for this motion.
   *
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_set(ez::QAngle p_target, int speed, bool slew_on);

  /**
   * Sets the robot to turn relative to initial heading using PID with units, using slew if enabled for this motion.
   *
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Turn to 90 degrees with the default behavior and slew
   *   chassis.pid_turn_set(90_deg, 110);
   *   chassis.pid_wait();
   *
   *   // Turn to 0 degrees the long way around, this spins 270 the other way
   *   chassis.pid_turn_set(0_deg, 110, ez::longest);
   *   chassis.pid_wait();
   *
   *   // Turn to 90 degrees with slew on for this motion only
   *   chassis.pid_turn_set(90_deg, 110, true);
   *   chassis.pid_wait();
   *
   *   // Turn to 0 degrees the long way around, with slew
   *   chassis.pid_turn_set(0_deg, 110, ez::longest, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_turn_set(ez::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn relative to the last commanded heading target using PID with units, only using slew if globally enabled. Adds to the last target
   * rather than the current heading, so error doesn't accumulate across chained relative turns.
   *
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_turn_relative_set(ez::QAngle p_target, int speed);

  /**
   * Sets the robot to turn relative to the last commanded heading target using PID with units, only using slew if globally enabled. Adds to the last target
   * rather than the current heading, so error doesn't accumulate across chained relative turns.
   *
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_turn_relative_set(ez::QAngle p_target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn relative to the last commanded heading target using PID with units, using slew if enabled for this motion. Adds to the last target
   * rather than the current heading, so error doesn't accumulate across chained relative turns.
   *
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_relative_set(ez::QAngle p_target, int speed, bool slew_on);

  /**
   * Sets the robot to turn relative to the last commanded heading target using PID with units, using slew if enabled for this motion. Adds to the last target
   * rather than the current heading, so error doesn't accumulate across chained relative turns.
   *
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Turns to 90 deg
   *   chassis.pid_turn_relative_set(90_deg, 110);
   *   chassis.pid_wait();
   *
   *   // Turns 45 degrees counterclockwise, to 45 deg
   *   chassis.pid_turn_relative_set(-45_deg, 110, true);
   *   chassis.pid_wait();
   *
   *   // Turns the long way around to go -45 from where it started
   *   chassis.pid_turn_relative_set(-45_deg, 110, ez::longest, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_turn_relative_set(ez::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn relative to the last commanded heading target using PID without units, only using slew if globally enabled. Adds to the last target
   * rather than the current heading, so error doesn't accumulate across chained relative turns.
   *
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_turn_relative_set(double target, int speed);

  /**
   * Sets the robot to turn relative to the last commanded heading target using PID without units, only using slew if globally enabled. Adds to the last target
   * rather than the current heading, so error doesn't accumulate across chained relative turns.
   *
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_turn_relative_set(double target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn relative to the last commanded heading target using PID without units, using slew if enabled for this motion. Adds to the last
   * target rather than the current heading, so error doesn't accumulate across chained relative turns.
   *
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_relative_set(double target, int speed, bool slew_on);

  /**
   * Sets the robot to turn relative to the last commanded heading target using PID without units, using slew if enabled for this motion. Adds to the last
   * target rather than the current heading, so error doesn't accumulate across chained relative turns.
   *
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_turn_relative_set(double target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without units, only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_set(e_swing type, double target, int speed);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without units, only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_swing_set(e_swing type, double target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without units, using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_set(e_swing type, double target, int speed, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without units, using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_set(e_swing type, double target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without units, only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   */
  void pid_swing_set(e_swing type, double target, int speed, int opposite_speed);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without units, only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_swing_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without units, using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_set(e_swing type, double target, int speed, int opposite_speed, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading without units, using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with units, only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_set(e_swing type, ez::QAngle p_target, int speed);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with units, only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_swing_set(e_swing type, ez::QAngle p_target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with units, using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_set(e_swing type, ez::QAngle p_target, int speed, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with units, using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_set(e_swing type, ez::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with units, only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   */
  void pid_swing_set(e_swing type, ez::QAngle p_target, int speed, int opposite_speed);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with units, only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_swing_set(e_swing type, ez::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn using only the left or right side relative to initial heading with units, using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_set(e_swing type, ez::QAngle p_target, int speed, int opposite_speed, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to initial heading using PID with units, using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Swing to 45 degrees with the default behavior and slew
   *   chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 110);
   *   chassis.pid_wait();
   *
   *   // Swing to 0 degrees the long way around
   *   chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 110, ez::longest);
   *   chassis.pid_wait();
   *
   *   // Swing to 45 degrees with slew on for this motion only
   *   chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 110, true);
   *   chassis.pid_wait();
   *
   *   // Swing to 0 degrees, the opposite side drives at up to 30 for an arc
   *   chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 110, 30);
   *   chassis.pid_wait();
   *
   *   // Swing to 45 degrees in an arc, taking the long way around, with slew
   *   chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 110, 30, ez::longest, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_swing_set(e_swing type, ez::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID with units, only
   * using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_relative_set(e_swing type, ez::QAngle p_target, int speed);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID with units, only
   * using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_swing_relative_set(e_swing type, ez::QAngle p_target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID with units, using
   * slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_relative_set(e_swing type, ez::QAngle p_target, int speed, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID with units, using
   * slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_relative_set(e_swing type, ez::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID with units, only
   * using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   */
  void pid_swing_relative_set(e_swing type, ez::QAngle p_target, int speed, int opposite_speed);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID with units, only
   * using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_swing_relative_set(e_swing type, ez::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID with units, using
   * slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_relative_set(e_swing type, ez::QAngle p_target, int speed, int opposite_speed, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID with units, using
   * slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param p_target
   *        target value in angle units
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Swing to 90 deg
   *   chassis.pid_swing_relative_set(ez::LEFT_SWING, 90_deg, 110);
   *   chassis.pid_wait();
   *
   *   // Swing -45 from the previous target, to 45 deg
   *   chassis.pid_swing_relative_set(ez::RIGHT_SWING, -45_deg, 110, true);
   *   chassis.pid_wait();
   *
   *   // Swing 45 more from the previous target, to 90 deg, as an arc
   *   chassis.pid_swing_relative_set(ez::LEFT_SWING, 45_deg, 110, 30, ez::shortest, true);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_swing_relative_set(e_swing type, ez::QAngle p_target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID without units,
   * only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_relative_set(e_swing type, double target, int speed);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID without units,
   * only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID without units,
   * using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID without units,
   * using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID without units,
   * only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID without units,
   * only using slew if globally enabled.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID without units,
   * using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed, bool slew_on);

  /**
   * Sets the robot to turn only using the left or right side relative to the last heading target, not the robot's measured heading, using PID without units,
   * using slew if enabled for this motion.
   *
   * \param type
   *        ez::LEFT_SWING or ez::RIGHT_SWING
   * \param target
   *        target value as a double, unit is degrees
   * \param speed
   *        0 to 127, max speed during motion
   * \param opposite_speed
   *        -127 to 127, max speed of the opposite side of the drive during the swing, scaled down as the swing slows.  positive moves the opposite side in the
   *        same direction as the swinging side.  0 (the default) holds the opposite side in place with the drive PID.  this is used for arcs
   * \param behavior
   *        changes what direction the robot will turn.  can be ez::ccw, ez::cw, ez::shortest, ez::longest, ez::raw
   * \param slew_on
   *        ramp up from a lower speed to your target speed
   */
  void pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on);

  /**
   * Resets all PID targets to 0.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
   * }
   * \endcode
   */
  void pid_targets_reset();

  /**
   * Sets the angle of the robot.  This is useful when your robot is set up at an unconventional angle and you want 0 to be when you're square with the
   * field.  It sets the heading of the imu and the target of the PID.
   *
   * \param p_angle
   *        angle that the robot will think it is now facing, angle unit
   *
   * \code{.cpp}
   * const int TURN_SPEED = 90;
   *
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.drive_angle_set(45_deg);
   *   chassis.pid_turn_set(0_deg, TURN_SPEED);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void drive_angle_set(ez::QAngle p_angle);

  /**
   * Sets the angle of the robot.  This is useful when your robot is set up at an unconventional angle and you want 0 to be when you're square with the
   * field.  It sets the heading of the imu and the target of the PID.
   *
   * \param angle
   *        angle that the robot will think it is now facing, in degrees
   */
  void drive_angle_set(double angle);

  /**
   * Lock the code in a while loop until the robot has settled.
   *
   * Returns when the motion's small, big, velocity or mA exit fires.  `interfered` is set when the velocity or mA exit ended the motion with the error outside
   * `big_error`.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_turn_set(90_deg, 110);
   *   chassis.pid_wait();
   *
   *   chassis.pid_turn_set(0_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_wait();

  /**
   * Lock the code in a while loop until the robot has turned or swung past this heading, with units.
   *
   * The target is an absolute heading, the same as the heading you gave the turn or swing.
   * `pid_turn_set(90_deg, 90); pid_wait_until(45_deg);` returns when the robot faces 45 degrees, with 45 degrees still to go.
   *
   * If the checkpoint can't be reached (past the heading the motion goes to, or on the other side of where it started), the wait returns when the
   * motion finishes, and prints why when printing is on.  `interfered` is only set when something actually stopped the robot.
   *
   * \param target
   *        absolute heading for a turn or swing, using units
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_turn_set(90_deg, 110);
   *   chassis.pid_wait_until(45_deg);
   *   chassis.pid_speed_max_set(40);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_wait_until(ez::QAngle target);

  /**
   * Lock the code in a while loop until the robot has driven this far, with units.
   *
   * The target is how far the robot has driven since THIS motion started, with the same sign as the drive.  It is not measured from
   * the end of the drive.  `pid_drive_set(24_in, 110); pid_wait_until(6_in);` returns after 6 inches, with 18 inches still to go.
   * Backward drives use negative numbers: `pid_drive_set(-24_in, 110); pid_wait_until(-6_in);`.
   *
   * If the checkpoint can't be reached (past the distance the drive goes, or the wrong sign), the wait returns when the motion finishes, and prints why
   * when printing is on.  `interfered` is only set when something actually stopped the robot.  For example, an odom movement that ends before the robot
   * has traveled `target`, like a 24 inch move waiting until 30 inches, releases the loop when the movement finishes.
   *
   * \param target
   *        distance driven since this motion started, using units, the same sign as the drive
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_drive_set(48_in, 110);
   *   chassis.pid_wait_until(24_in);
   *   chassis.pid_speed_max_set(40);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_wait_until(ez::QLength target);

  /**
   * Lock the code in a while loop until the robot has driven this far, or turned or swung past this heading, without units.
   *
   * For drives the target is how far the robot has driven since THIS motion started, in inches, with the same sign as the drive.
   * `pid_drive_set(24_in, 110); pid_wait_until(6);` returns after 6 inches, with 18 inches still to go.  Backward drives use negative
   * numbers: `pid_drive_set(-24_in, 110); pid_wait_until(-6);`.
   *
   * For turns and swings the target is an absolute heading in degrees, the same as the turn target.
   * `pid_turn_set(90_deg, 90); pid_wait_until(45);` returns when the robot faces 45 degrees.
   *
   * If the checkpoint can't be reached (past the target, or the wrong sign), the wait returns when the motion finishes, and prints why when printing is
   * on.  `interfered` is only set when something actually stopped the robot.  This includes odom motions, for example a 24 inch move waiting until 30
   * inches.
   *
   * \param target
   *        for driving, inches driven since this motion started (same sign as the drive).  for turns/swings, an absolute heading in degrees
   */
  void pid_wait_until(double target);

  /**
   * Lock the code in a while loop until the robot has passed its target (or an exit condition fires first).
   *
   * Wrapper for pid_wait_until(target), target is your previously input target.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_turn_set(90_deg, 110);
   *   chassis.pid_wait_quick();
   *
   *   chassis.pid_turn_set(0_deg, 110);
   *   chassis.pid_wait_quick();
   * }
   * \endcode
   */
  void pid_wait_quick();

  /**
   * Lock the code in a while loop until the robot has passed its target (or an exit condition fires first).
   *
   * This also adds distance to target, and then exits with pid_wait_quick.
   *
   * This will exit the motion while carrying momentum into the next motion.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_turn_set(90_deg, 110);
   *   chassis.pid_wait_quick_chain();
   *
   *   chassis.pid_turn_set(0_deg, 110);
   *   chassis.pid_wait_quick();
   * }
   * \endcode
   */
  void pid_wait_quick_chain();

  /**
   * Lock the code in a while loop until the robot has passed this point of your path.
   *
   * `pid_odom_set({{{0_in, 24_in}, fwd, 110}, {{24_in, 24_in}, fwd, 110}}); pid_wait_until_index(0);` returns once the robot has
   * passed the first point, (0, 24).
   *
   * \param index
   *        index of your input points, 0 is the first point in the index
   *
   * \code{.cpp}
   * pros::Motor intake(9);
   *
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Drive through three points, with slew
   *   chassis.pid_odom_set(
   *       {
   *           {{0_in, 24_in}, ez::fwd, 110},
   *           {{12_in, 24_in}, ez::rev, 110},
   *           {{24_in, 24_in}, ez::rev, 110},
   *       },
   *       true);
   *   chassis.pid_wait_until_index(1);  // Wait until the robot passes 12, 24
   *
   *   // Run the intake
   *   intake.move(127);
   *
   *   // Wait for the path to finish
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_wait_until_index(int index);

  /**
   * Lock the code in a while loop until this point becomes the target.
   *
   * \param index
   *        index of your input points, 0 is the first point in the index
   *
   * \code{.cpp}
   * pros::Motor intake(9);
   *
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Drive through three points, with slew
   *   chassis.pid_odom_set(
   *       {
   *           {{0_in, 24_in}, ez::fwd, 110},
   *           {{12_in, 24_in}, ez::rev, 110},
   *           {{24_in, 24_in}, ez::rev, 110},
   *       },
   *       true);
   *   chassis.pid_wait_until_index_started(1);  // Wait until the target becomes 12, 24
   *
   *   // Run the intake
   *   intake.move(127);
   *
   *   // Wait for the path to finish
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_wait_until_index_started(int index);

  /**
   * Lock the code in a while loop until the robot has passed this point on the field.
   *
   * The target is a field position, not a distance: `pid_wait_until_point({24, 24});` returns when the robot passes (24, 24).
   *
   * \param target
   *        {x, y} pose for the robot to pass through before the while loop is released
   */
  void pid_wait_until_point(pose target);

  /**
   * Lock the code in a while loop until the robot has passed this point on the field, with units.
   *
   * The target is a field position, not a distance: `pid_wait_until_point({24_in, 24_in});` returns when the robot passes (24, 24).
   *
   * Use this on a single movement.  For a point on a multi-point path, use pid_wait_until_index().
   *
   * \param target
   *        {x, y} pose with units for the robot to pass through before the while loop is released
   *
   * \code{.cpp}
   * pros::Motor intake(9);
   *
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // Drive to 24, 24
   *   chassis.pid_odom_set({{24_in, 24_in}, ez::fwd, 110});
   *
   *   // Wait until the robot passes 12, 12
   *   chassis.pid_wait_until_point({12_in, 12_in});
   *
   *   // Run the intake
   *   intake.move(127);
   *
   *   // Wait for the movement to finish
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_wait_until_point(united_pose target);

  /**
   * Lock the code in a while loop until the robot has passed this point on the field.
   *
   * Wrapper for pid_wait_until_point.  `pid_wait_until({24, 24});` returns when the robot passes (24, 24).
   *
   * \param target
   *        {x, y}  a pose for the robot to pass through before the while loop is released
   */
  void pid_wait_until(pose target);

  /**
   * Lock the code in a while loop until the robot has passed this point on the field, with units.
   *
   * Wrapper for pid_wait_until_point.  `pid_wait_until({24_in, 24_in});` returns when the robot passes (24, 24).
   *
   * \param target
   *        {x, y}  a pose with units for the robot to pass through before the while loop is released
   */
  void pid_wait_until(united_pose target);

  /**
   * Autonomous interference detection.
   *
   * Returns true when interfered, and false when nothing happened.
   *
   * A wait that ended on the velocity or mA exit with the error outside `big_error` sets this.
   */
  bool interfered = false;

  /**
   * Removed in 4.0. Calling it is a compile error that says what to call instead. It is a template only so the error can carry that message
   * on every compiler (`= delete("reason")` needs GCC 15, and the projects build with GCC 14): a project that never calls it compiles as if it were not there.
   *
   * To shift a transmission, call drive_rpm_set() with the wheel RPM of the new gear: cartridge RPM / ratio.
   */
  template <bool Called = true>
  void drive_ratio_set(double) {
    static_assert(!Called,
                  "drive_ratio_set() was removed in 4.0. To shift, call drive_rpm_set(wheel_rpm) with the wheel RPM of the new gear (cartridge_rpm / ratio).");
  }

  /**
   * Sets the wheel's RPM: the same number as the constructor's `ticks`, cartridge RPM * (motor gear / wheel gear). It is the
   * one number EZ-Template has for your gearing, and it is how a shifting transmission tells the drive about a gear
   * change: call it with the wheel RPM of the new gear when you shift.
   *
   * You can call it at any time, even in the middle of a motion: odom, the running motion, the active
   * brake and the waits carry on from where the robot is, and only the distance traveled after the change
   * uses the new RPM.
   *
   * Has no effect if you have two tracking wheels.
   *
   * \param rpm
   *        the wheel's RPM, as in the constructor's `ticks`
   */
  void drive_rpm_set(double rpm);

  /**
   * Removed in 4.0, like drive_ratio_set(). Calling it is a compile error that says to use drive_rpm_get().
   */
  template <bool Called = true>
  double drive_ratio_get() {
    static_assert(!Called, "drive_ratio_get() was removed in 4.0. Use drive_rpm_get(), the wheel RPM.");
    return 0.0;
  }

  /**
   * Returns the wheel's RPM: the constructor's `ticks`, or the last value given to drive_rpm_set(). It is the one number
   * the drive has for its gearing, and it is what a shifting transmission changes when it shifts.
   */
  double drive_rpm_get();

  /**
   * Changes max speed during a drive motion.
   *
   * Also applies mid-motion to a running odom motion (pid_odom_set, pid_odom_pp_set,
   * pid_odom_injected_pp_set, pid_odom_smooth_pp_set, pid_odom_boomerang_set, pid_odom_ptp_set):
   * the new cap replaces the stored speed on every remaining point of the path, not just the
   * current one, and lasts only for the motion currently running -- the next pid_*_set call starts
   * fresh with its own speed.
   *
   * \param speed
   *        new clipped speed, between 0 and 127
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_drive_set(48_in, 110);
   *   chassis.pid_wait_until(24_in);
   *   chassis.pid_speed_max_set(40);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_speed_max_set(int speed);

  /**
   * Returns max speed of drive during autonomous.
   */
  int pid_speed_max_get();

  /**
   * Set PID constants for turns.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral, 0 means i always accumulates
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_turn_constants_set(3, 0, 20); }
   * \endcode
   */
  void pid_turn_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns the PID constants for turns, as a PID::Constants with kp, ki, kd and start_i.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);
   *   ez::PID::Constants c = chassis.pid_turn_constants_get();
   *   // Prints kp 3.00  ki 0.05  kd 20.00  start_i 15.00
   *   printf("kp %.2f  ki %.2f  kd %.2f  start_i %.2f\n", c.kp, c.ki, c.kd, c.start_i);
   * }
   * \endcode
   */
  PID::Constants pid_turn_constants_get();

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This sets turning constants.
   *
   * \param input
   *        angle unit
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_turn_chain_constant_set(3_deg); }
   * \endcode
   */
  void pid_turn_chain_constant_set(ez::QAngle input);

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
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_turn_chain_constant_set(3_deg);
   *   printf("%.2f\n", chassis.pid_turn_chain_constant_get());
   * }
   * \endcode
   */
  double pid_turn_chain_constant_get();

  /**
   * Set PID constants for forward and backward swings.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral, 0 means i always accumulates
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_swing_constants_set(5, 0, 30); }
   * \endcode
   */
  void pid_swing_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns the PID constants for swings, as a PID::Constants with kp, ki, kd and start_i.
   *
   * If the forward and backward constants were set to different values, this prints `Forward and Reverse constants are not the same!` and returns
   * {-1, -1, -1, -1}.  Use the forward and backward getters when they differ.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_swing_constants_set(6.0, 0.0, 65.0);
   *   ez::PID::Constants c = chassis.pid_swing_constants_get();
   *   // Prints kp 6.00  ki 0.00  kd 65.00  start_i 0.00
   *   printf("kp %.2f  ki %.2f  kd %.2f  start_i %.2f\n", c.kp, c.ki, c.kd, c.start_i);
   * }
   * \endcode
   */
  PID::Constants pid_swing_constants_get();

  /**
   * Set PID constants for forward swings.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral, 0 means i always accumulates
   */
  void pid_swing_constants_forward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns the PID constants for forward swings, as a PID::Constants with kp, ki, kd and start_i.
   *
   * These are the constants a forward swing will use.  If no forward constants were set,
   * this returns the constants from pid_swing_constants_set().
   */
  PID::Constants pid_swing_constants_forward_get();

  /**
   * Set PID constants for backward swings.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral, 0 means i always accumulates
   */
  void pid_swing_constants_backward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns the PID constants for backward swings, as a PID::Constants with kp, ki, kd and start_i.
   *
   * These are the constants a backward swing will use.  If no backward constants were set,
   * this returns the constants from pid_swing_constants_set().
   */
  PID::Constants pid_swing_constants_backward_get();

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This sets forward and backwards swing constants.
   *
   * \param input
   *        angle unit
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_swing_chain_constant_set(5_deg); }
   * \endcode
   */
  void pid_swing_chain_constant_set(ez::QAngle input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets forward swing constants.
   *
   * \param input
   *        angle unit
   */
  void pid_swing_chain_forward_constant_set(ez::QAngle input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets backward swing constants.
   *
   * \param input
   *        angle unit
   */
  void pid_swing_chain_backward_constant_set(ez::QAngle input);

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
   * This only sets forward swing constants.
   *
   * \param input
   *        angle in degrees
   */
  void pid_swing_chain_forward_constant_set(double input);

  /**
   * Returns the amount that the PID will overshoot target by to maintain momentum into the next motion for swinging forward.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_swing_chain_constant_set(5_deg);
   *   printf("%.2f\n", chassis.pid_swing_chain_forward_constant_get());
   * }
   * \endcode
   */
  double pid_swing_chain_forward_constant_get();

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets backward swing constants.
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
   * Set PID constants for the heading correction during drive motions.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral, 0 means i always accumulates
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_heading_constants_set(11, 0, 20); }
   * \endcode
   */
  void pid_heading_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns the PID constants that correct the robot's heading during drive motions, as a `PID::Constants` with `kp`, `ki`, `kd` and `start_i`.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
   *   ez::PID::Constants c = chassis.pid_heading_constants_get();
   *   // Prints kp 11.00  ki 0.00  kd 20.00  start_i 0.00
   *   printf("kp %.2f  ki %.2f  kd %.2f  start_i %.2f\n", c.kp, c.ki, c.kd, c.start_i);
   * }
   * \endcode
   */
  PID::Constants pid_heading_constants_get();

  /**
   * Set PID drive constants for forwards and backwards.
   *
   * Calling this clears any constants set with pid_drive_constants_forward_set() or pid_drive_constants_backward_set(), so call it first.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral, 0 means i always accumulates
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_drive_constants_set(20, 0, 100); }
   * \endcode
   */
  void pid_drive_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns the PID constants for driving, as a `PID::Constants` with `kp`, `ki`, `kd` and `start_i`.
   *
   * If the forward and backward constants were set to different values, this prints `Forward and Reverse constants are not the same!` and returns
   * `{-1, -1, -1, -1}`.  Use the forward and backward getters when they differ.
   *
   * `pid_drive_constants_forward_get()` and `pid_drive_constants_backward_get()` return the constants a forward or backward drive will use.  If none
   * were set for that direction, they return the constants from `pid_drive_constants_set()`.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
   *   ez::PID::Constants c = chassis.pid_drive_constants_get();
   *   // Prints kp 20.00  ki 0.00  kd 100.00  start_i 0.00
   *   printf("kp %.2f  ki %.2f  kd %.2f  start_i %.2f\n", c.kp, c.ki, c.kd, c.start_i);
   * }
   * \endcode
   */
  PID::Constants pid_drive_constants_get();

  /**
   * Set PID drive constants for forwards movements.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral, 0 means i always accumulates
   */
  void pid_drive_constants_forward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns the PID constants for driving forward, as a `PID::Constants` with `kp`, `ki`, `kd` and `start_i`.
   *
   * These are the constants a forward drive will use.  If no forward constants were set, this returns the constants from `pid_drive_constants_set()`.
   */
  PID::Constants pid_drive_constants_forward_get();

  /**
   * Set PID drive constants for backwards movements.
   *
   * \param p
   *        proportional term
   * \param i
   *        integral term
   * \param d
   *        derivative term
   * \param p_start_i
   *        error threshold to start integral, 0 means i always accumulates
   */
  void pid_drive_constants_backward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Returns the PID constants for driving backward, as a `PID::Constants` with `kp`, `ki`, `kd` and `start_i`.
   *
   * These are the constants a backward drive will use.  If no backward constants were set, this returns the constants from `pid_drive_constants_set()`.
   */
  PID::Constants pid_drive_constants_backward_get();

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This sets forward and backwards driving constants.
   *
   * \param input
   *        length unit
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_drive_chain_constant_set(3_in); }
   * \endcode
   */
  void pid_drive_chain_constant_set(ez::QLength input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets forward driving constants.
   *
   * \param input
   *        length unit
   */
  void pid_drive_chain_forward_constant_set(ez::QLength input);

  /**
   * Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.
   *
   * This only sets backward driving constants.
   *
   * \param input
   *        length unit
   */
  void pid_drive_chain_backward_constant_set(ez::QLength input);

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
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_drive_chain_constant_set(3_in);
   *   printf("%.2f\n", chassis.pid_drive_chain_forward_constant_get());
   * }
   * \endcode
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
   * When kI and startI are enabled, sets the maximum output allowed while error is inside
   * startI, for swings larger than startI. This lets I accumulate without overshoot.  Despite the name this is a cap on the output, not a floor.
   * Defaults to 30.
   *
   * \param min
   *        new clipped speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   // The swing min only has an effect when kI and startI are set
   *   chassis.pid_swing_constants_set(6, 0.05, 65, 15);
   *   chassis.pid_swing_min_set(40);
   *
   *   // Swing to 45 degrees
   *   chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_swing_min_set(int min);

  /**
   * When kI and startI are enabled, sets the maximum output allowed while error is inside
   * startI, for turns larger than startI. This lets I accumulate without overshoot.  Despite the name this is a cap on the output, not a floor.
   * Defaults to 30.
   *
   * \param min
   *        new clipped speed
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_turn_min_set(30);
   *
   *   chassis.pid_turn_set(45_deg, 110);
   *   chassis.pid_wait();
   * }
   * \endcode
   */
  void pid_turn_min_set(int min);

  /**
   * Returns the maximum output allowed while error is inside startI, for swings larger than startI,
   * when kI and startI are enabled.  This is a cap on the output, not a floor.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_swing_min_set(30);
   *
   *   printf("Swing Min: %i", chassis.pid_swing_min_get());
   * }
   * \endcode
   */
  int pid_swing_min_get();

  /**
   * Returns the maximum output allowed while error is inside startI, for turns larger than startI,
   * when kI and startI are enabled.  This is a cap on the output, not a floor.
   *
   * \code{.cpp}
   * void autonomous() {
   *   chassis.pid_targets_reset();                // Resets PID targets to 0
   *   chassis.drive_imu_reset();                  // Reset gyro position to 0
   *   chassis.drive_sensor_reset();               // Reset drive sensors to 0
   *   chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the start position
   *   chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold motors for consistency
   *
   *   chassis.pid_turn_min_set(30);
   *
   *   printf("Turn Min: %i", chassis.pid_turn_min_get());
   * }
   * \endcode
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
   *        error threshold to start integral, 0 means i always accumulates
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_odom_angular_constants_set(6.5, 0, 52.5); }
   * \endcode
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
   *        error threshold to start integral, 0 means i always accumulates
   *
   * \code{.cpp}
   * void initialize() { chassis.pid_odom_boomerang_constants_set(5.8, 0, 32.5); }
   * \endcode
   */
  void pid_odom_boomerang_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Sets constants for odom driving exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within small_error, in ms
   * \param p_small_error
   *        small timer will start when error is within this, in inches
   * \param p_big_exit_time
   *        time to exit when within big_error, in ms
   * \param p_big_error
   *        big timer will start when error is within this, in inches
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0 after the robot has moved (or after 1 second if it never moves), in ms
   * \param p_mA_timeout
   *        mA timer will start when the first motor on the side(s) being driven is over its current limit, in ms
   * \param use_imu
   *        true adds a second velocity exit timer from the imu's acceleration.  Off by default: a steady cruise also reads near 0 acceleration,
   *        so it can exit early.  It's the only exit that can catch wheels spinning free (lifted or high-centered), which the main sensor and
   *        the mA timeout both miss.  Turn it on if that case matters more to you than early exits on a slow cruise.
   */
  void pid_odom_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time,
                                         int p_mA_timeout, bool use_imu = false);

  /**
   * Sets constants for odom turning exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within small_error, in ms
   * \param p_small_error
   *        small timer will start when error is within this, in degrees
   * \param p_big_exit_time
   *        time to exit when within big_error, in ms
   * \param p_big_error
   *        big timer will start when error is within this, in degrees
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0 after the robot has moved (or after 1 second if it never moves), in ms
   * \param p_mA_timeout
   *        mA timer will start when the first motor on the side(s) being driven is over its current limit, in ms
   * \param use_imu
   *        true adds a second velocity exit timer from the imu's acceleration.  Off by default: a steady cruise also reads near 0 acceleration,
   *        so it can exit early.  It's the only exit that can catch wheels spinning free (lifted or high-centered), which the main sensor and
   *        the mA timeout both miss.  Turn it on if that case matters more to you than early exits on a slow cruise.
   */
  void pid_odom_turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time,
                                        int p_mA_timeout, bool use_imu = false);

  /**
   * Sets constants for odom turning exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within small_error, in units
   * \param p_small_error
   *        small timer will start when error is within this, in units
   * \param p_big_exit_time
   *        time to exit when within big_error, in units
   * \param p_big_error
   *        big timer will start when error is within this, in units
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0 after the robot has moved (or after 1 second if it never moves), in units
   * \param p_mA_timeout
   *        mA timer will start when the first motor on the side(s) being driven is over its current limit, in units
   * \param use_imu
   *        true adds a second velocity exit timer from the imu's acceleration.  Off by default: a steady cruise also reads near 0 acceleration,
   *        so it can exit early.  It's the only exit that can catch wheels spinning free (lifted or high-centered), which the main sensor and
   *        the mA timeout both miss.  Turn it on if that case matters more to you than early exits on a slow cruise.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
   * }
   * \endcode
   */
  void pid_odom_turn_exit_condition_set(ez::QTime p_small_exit_time, ez::QAngle p_small_error, ez::QTime p_big_exit_time, ez::QAngle p_big_error,
                                        ez::QTime p_velocity_exit_time, ez::QTime p_mA_timeout, bool use_imu = false);

  /**
   * Sets constants for odom driving exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within small_error, in units
   * \param p_small_error
   *        small timer will start when error is within this, in units
   * \param p_big_exit_time
   *        time to exit when within big_error, in units
   * \param p_big_error
   *        big timer will start when error is within this, in units
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0 after the robot has moved (or after 1 second if it never moves), in units
   * \param p_mA_timeout
   *        mA timer will start when the first motor on the side(s) being driven is over its current limit, in units
   * \param use_imu
   *        true adds a second velocity exit timer from the imu's acceleration.  Off by default: a steady cruise also reads near 0 acceleration,
   *        so it can exit early.  It's the only exit that can catch wheels spinning free (lifted or high-centered), which the main sensor and
   *        the mA timeout both miss.  Turn it on if that case matters more to you than early exits on a slow cruise.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
   * }
   * \endcode
   */
  void pid_odom_drive_exit_condition_set(ez::QTime p_small_exit_time, ez::QLength p_small_error, ez::QTime p_big_exit_time, ez::QLength p_big_error,
                                         ez::QTime p_velocity_exit_time, ez::QTime p_mA_timeout, bool use_imu = false);

  /**
   * Sets constants for drive exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within small_error, in units
   * \param p_small_error
   *        small timer will start when error is within this, in units
   * \param p_big_exit_time
   *        time to exit when within big_error, in units
   * \param p_big_error
   *        big timer will start when error is within this, in units
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0 after the robot has moved (or after 1 second if it never moves), in units
   * \param p_mA_timeout
   *        mA timer will start when the first motor on the side(s) being driven is over its current limit, in units
   * \param use_imu
   *        true adds a second velocity exit timer from the imu's acceleration.  Off by default: a steady cruise also reads near 0 acceleration,
   *        so it can exit early.  It's the only exit that can catch wheels spinning free (lifted or high-centered), which the main sensor and
   *        the mA timeout both miss.  Turn it on if that case matters more to you than early exits on a slow cruise.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
   * }
   * \endcode
   */
  void pid_drive_exit_condition_set(ez::QTime p_small_exit_time, ez::QLength p_small_error, ez::QTime p_big_exit_time, ez::QLength p_big_error,
                                    ez::QTime p_velocity_exit_time, ez::QTime p_mA_timeout, bool use_imu = false);

  /**
   * Sets constants for turn exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within small_error, in units
   * \param p_small_error
   *        small timer will start when error is within this, in units
   * \param p_big_exit_time
   *        time to exit when within big_error, in units
   * \param p_big_error
   *        big timer will start when error is within this, in units
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0 after the robot has moved (or after 1 second if it never moves), in units
   * \param p_mA_timeout
   *        mA timer will start when the first motor on the side(s) being driven is over its current limit, in units
   * \param use_imu
   *        true adds a second velocity exit timer from the imu's acceleration.  Off by default: a steady cruise also reads near 0 acceleration,
   *        so it can exit early.  It's the only exit that can catch wheels spinning free (lifted or high-centered), which the main sensor and
   *        the mA timeout both miss.  Turn it on if that case matters more to you than early exits on a slow cruise.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
   * }
   * \endcode
   */
  void pid_turn_exit_condition_set(ez::QTime p_small_exit_time, ez::QAngle p_small_error, ez::QTime p_big_exit_time, ez::QAngle p_big_error,
                                   ez::QTime p_velocity_exit_time, ez::QTime p_mA_timeout, bool use_imu = false);

  /**
   * Sets constants for swing exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within small_error, in units
   * \param p_small_error
   *        small timer will start when error is within this, in units
   * \param p_big_exit_time
   *        time to exit when within big_error, in units
   * \param p_big_error
   *        big timer will start when error is within this, in units
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0 after the robot has moved (or after 1 second if it never moves), in units
   * \param p_mA_timeout
   *        mA timer will start when the first motor on the side(s) being driven is over its current limit, in units
   * \param use_imu
   *        true adds a second velocity exit timer from the imu's acceleration.  Off by default: a steady cruise also reads near 0 acceleration,
   *        so it can exit early.  It's the only exit that can catch wheels spinning free (lifted or high-centered), which the main sensor and
   *        the mA timeout both miss.  Turn it on if that case matters more to you than early exits on a slow cruise.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
   * }
   * \endcode
   */
  void pid_swing_exit_condition_set(ez::QTime p_small_exit_time, ez::QAngle p_small_error, ez::QTime p_big_exit_time, ez::QAngle p_big_error,
                                    ez::QTime p_velocity_exit_time, ez::QTime p_mA_timeout, bool use_imu = false);

  /**
   * Sets constants for drive exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within small_error, in ms
   * \param p_small_error
   *        small timer will start when error is within this, in inches
   * \param p_big_exit_time
   *        time to exit when within big_error, in ms
   * \param p_big_error
   *        big timer will start when error is within this, in inches
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0 after the robot has moved (or after 1 second if it never moves), in ms
   * \param p_mA_timeout
   *        mA timer will start when the first motor on the side(s) being driven is over its current limit, in ms
   * \param use_imu
   *        true adds a second velocity exit timer from the imu's acceleration.  Off by default: a steady cruise also reads near 0 acceleration,
   *        so it can exit early.  It's the only exit that can catch wheels spinning free (lifted or high-centered), which the main sensor and
   *        the mA timeout both miss.  Turn it on if that case matters more to you than early exits on a slow cruise.
   */
  void pid_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time,
                                    int p_mA_timeout, bool use_imu = false);

  /**
   * Sets constants for turn exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within small_error, in ms
   * \param p_small_error
   *        small timer will start when error is within this, in degrees
   * \param p_big_exit_time
   *        time to exit when within big_error, in ms
   * \param p_big_error
   *        big timer will start when error is within this, in degrees
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0 after the robot has moved (or after 1 second if it never moves), in ms
   * \param p_mA_timeout
   *        mA timer will start when the first motor on the side(s) being driven is over its current limit, in ms
   * \param use_imu
   *        true adds a second velocity exit timer from the imu's acceleration.  Off by default: a steady cruise also reads near 0 acceleration,
   *        so it can exit early.  It's the only exit that can catch wheels spinning free (lifted or high-centered), which the main sensor and
   *        the mA timeout both miss.  Turn it on if that case matters more to you than early exits on a slow cruise.
   */
  void pid_turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time,
                                   int p_mA_timeout, bool use_imu = false);

  /**
   * Sets constants for swing exit conditions.
   *
   * \param p_small_exit_time
   *        time to exit when within small_error, in ms
   * \param p_small_error
   *        small timer will start when error is within this, in degrees
   * \param p_big_exit_time
   *        time to exit when within big_error, in ms
   * \param p_big_error
   *        big timer will start when error is within this, in degrees
   * \param p_velocity_exit_time
   *        velocity timer will start when velocity is 0 after the robot has moved (or after 1 second if it never moves), in ms
   * \param p_mA_timeout
   *        mA timer will start when the first motor on the side(s) being driven is over its current limit, in ms
   * \param use_imu
   *        true adds a second velocity exit timer from the imu's acceleration.  Off by default: a steady cruise also reads near 0 acceleration,
   *        so it can exit early.  It's the only exit that can catch wheels spinning free (lifted or high-centered), which the main sensor and
   *        the mA timeout both miss.  Turn it on if that case matters more to you than early exits on a slow cruise.
   */
  void pid_swing_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time,
                                    int p_mA_timeout, bool use_imu = false);

  /**
   * Sets how slow the robot has to be moving to count as stopped when it drives, in inches per second.  This is a real speed, not the 0 to 127
   * motor speed.  The default is 1.5.
   *
   * The robot's speed is how far it moved over the last 100 ms.  The small and big exits only count time while the robot is under this speed (the
   * timer pauses while it is moving, it does not start over).  The velocity exit does not use it: it ends a wait when the robot has been under 5
   * inches per second (5 degrees per second for turns) for the velocity exit time.  A higher speed ends motions sooner, with the
   * robot still moving a little.  A lower one waits for the robot to be stiller.  A speed of 0 or less, or one that isn't a number, is refused with
   * a printed warning and the old speed is kept.
   *
   * Applies to drives and odom moves.  The default is a placeholder until it is tuned from a robot log.
   *
   * \param in_per_s
   *        inches per second
   */
  void pid_distance_exit_stop_speed_set(double in_per_s);

  /**
   * Same as pid_distance_exit_stop_speed_set(double), with a speed that has units: `2_in / 1_s`.
   *
   * \param speed
   *        speed with units
   */
  void pid_distance_exit_stop_speed_set(ez::QSpeed speed);

  /**
   * Returns how slow a drive has to be moving to count as stopped, in inches per second.
   */
  double pid_distance_exit_stop_speed_get();

  /**
   * Sets how slow the robot has to be turning to count as stopped when it turns or swings, in degrees per second.  This is a real speed, not the
   * 0 to 127 motor speed.  The default is 4.
   *
   * Works the same as pid_distance_exit_stop_speed_set(): measured over the last 100 ms, the small and big exits pause their timers while the
   * robot is above it, and the velocity exit does not use it.  Applies to turns, swings, and the heading of odom moves.  The default is a
   * placeholder until it is tuned from a robot log.
   *
   * \param deg_per_s
   *        degrees per second
   */
  void pid_angle_exit_stop_speed_set(double deg_per_s);

  /**
   * Same as pid_angle_exit_stop_speed_set(double), with a speed that has units: `6_deg / 1_s`.
   *
   * \param speed
   *        speed with units
   */
  void pid_angle_exit_stop_speed_set(ez::QAngularSpeed speed);

  /**
   * Returns how slow a turn or swing has to be moving to count as stopped, in degrees per second.
   */
  double pid_angle_exit_stop_speed_get();

  /**
   * Returns the conversion between raw sensor value and inches.
   *
   * \code{.cpp}
   * void initialize() { printf("Tick Per Inch: %f\n", chassis.drive_tick_per_inch()); }
   * \endcode
   */
  double drive_tick_per_inch();

  /**
   * Allows the user to modify the curve with the controller.
   *
   * The curve buttons are off by default, turn them on with opcontrol_curve_buttons_toggle(true).
   * opcontrol_tank() and the arcade functions already call this, so it is only needed in a custom drive loop
   * that also applies opcontrol_curve_left() and opcontrol_curve_right().
   *
   * \code{.cpp}
   * void initialize() {
   *   // The curve buttons are off by default
   *   chassis.opcontrol_curve_buttons_toggle(true);
   *   chassis.initialize();
   * }
   *
   * void opcontrol() {
   *   while (true) {
   *     // Change the curves with the controller buttons
   *     chassis.opcontrol_curve_buttons_iterate();
   *
   *     // Apply the curves to the joysticks and drive with them
   *     int l_stick = chassis.opcontrol_curve_left(master.get_analog(ANALOG_LEFT_Y));
   *     int r_stick = chassis.opcontrol_curve_left(master.get_analog(ANALOG_RIGHT_Y));
   *     chassis.opcontrol_joystick_threshold_iterate(l_stick, r_stick);
   *
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void opcontrol_curve_buttons_iterate();

  /**
   * Enables PID Tuner.
   *
   * Only use the PID Tuner when the robot is not connected to a competition switch.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *
   *   // Enable PID Tuner, only when not connected to a competition switch
   *   if (!pros::competition::is_connected()) chassis.pid_tuner_enable();
   *
   *   while (true) {
   *     if (!pros::competition::is_connected()) {
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) {
   *         // Autonomous sets the brake to hold, so save the brake mode and restore it after
   *         pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
   *         autonomous();
   *         chassis.drive_brake_set(preference);
   *       }
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void pid_tuner_enable();

  /**
   * Disables PID Tuner.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *
   *   // Enable PID Tuner, only when not connected to a competition switch
   *   if (!pros::competition::is_connected()) chassis.pid_tuner_enable();
   *
   *   while (true) {
   *     if (!pros::competition::is_connected()) {
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) {
   *         // Autonomous sets the brake to hold, so save the brake mode and restore it after
   *         pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
   *         autonomous();
   *         chassis.drive_brake_set(preference);
   *       }
   *
   *       // Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_disable();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void pid_tuner_disable();

  /**
   * Toggles PID tuner between enabled and disabled.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void pid_tuner_toggle();

  /**
   * Checks if PID Tuner is enabled.
   *
   * True is enabled, false is disabled.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) {
   *         if (chassis.pid_tuner_enabled())
   *           chassis.pid_tuner_disable();
   *         else
   *           chassis.pid_tuner_enable();
   *       }
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  bool pid_tuner_enabled();

  /**
   * Iterates through controller inputs to modify PID constants.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void pid_tuner_iterate();

  /**
   * Toggle for printing the display of the PID Tuner to the brain.  It prints to the brain by default.  This can be set
   * before the PID Tuner is enabled.  With this off the PID Tuner leaves the brain screen alone, including the auton selector.
   *
   * \param input
   *        true prints to brain, false doesn't
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *
   *   // Can be set before the PID Tuner is enabled
   *   // Keeps the auton selector on the brain, and prints the PID Tuner to the terminal instead
   *   chassis.pid_tuner_print_brain_set(false);
   *   chassis.pid_tuner_print_terminal_set(true);
   *
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void pid_tuner_print_brain_set(bool input);

  /**
   * Toggle for printing the display of the PID Tuner to the terminal.
   *
   * \param input
   *        true prints to terminal, false doesn't
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   chassis.pid_tuner_print_terminal_set(true);
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void pid_tuner_print_terminal_set(bool input);

  /**
   * Returns true if printing to terminal is enabled.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   chassis.pid_tuner_print_terminal_set(true);
   *
   *   // Prints 1
   *   printf("Printing to Terminal? %i\n", chassis.pid_tuner_print_terminal_enabled());
   *
   *   chassis.pid_tuner_print_terminal_set(false);
   *
   *   // Prints 0
   *   printf("Printing to Terminal? %i\n", chassis.pid_tuner_print_terminal_enabled());
   *
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  bool pid_tuner_print_terminal_enabled();

  /**
   * Returns true if printing to brain is enabled.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *
   *   chassis.pid_tuner_print_brain_set(false);
   *   printf("Printing to Brain? %i\n", chassis.pid_tuner_print_brain_enabled());  // Prints 0
   *
   *   chassis.pid_tuner_print_brain_set(true);
   *   printf("Printing to Brain? %i\n", chassis.pid_tuner_print_brain_enabled());  // Prints 1
   *
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  bool pid_tuner_print_brain_enabled();

  /**
   * Sets the value that PID Tuner increments P.
   *
   * \param p
   *        p will increase by this
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   chassis.pid_tuner_increment_p_set(0.25);
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void pid_tuner_increment_p_set(double p);

  /**
   * Sets the value that PID Tuner increments I.
   *
   * \param i
   *        i will increase by this
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   chassis.pid_tuner_increment_i_set(0.01);
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void pid_tuner_increment_i_set(double i);

  /**
   * Sets the value that PID Tuner increments D.
   *
   * \param d
   *        d will increase by this
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   chassis.pid_tuner_increment_d_set(1.0);
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void pid_tuner_increment_d_set(double d);

  /**
   * Sets the value that PID Tuner increments Start I.
   *
   * \param start_i
   *        start i will increase by this
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   chassis.pid_tuner_increment_start_i_set(5.0);
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  void pid_tuner_increment_start_i_set(double start_i);

  /**
   * Returns the value that PID Tuner increments P.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   chassis.pid_tuner_increment_p_set(0.25);
   *   printf("%f\n", chassis.pid_tuner_increment_p_get());
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double pid_tuner_increment_p_get();

  /**
   * Returns the value that PID Tuner increments I.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   chassis.pid_tuner_increment_i_set(0.01);
   *   printf("%f\n", chassis.pid_tuner_increment_i_get());
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double pid_tuner_increment_i_get();

  /**
   * Returns the value that PID Tuner increments D.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   chassis.pid_tuner_increment_d_set(1.0);
   *   printf("%f\n", chassis.pid_tuner_increment_d_get());
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
   */
  double pid_tuner_increment_d_get();

  /**
   * Returns the value that PID Tuner increments Start I.
   *
   * \code{.cpp}
   * void opcontrol() {
   *   // This is preference to what you like to drive on
   *   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
   *   chassis.pid_tuner_increment_start_i_set(5.0);
   *   printf("%f\n", chassis.pid_tuner_increment_start_i_get());
   *   while (true) {
   *     // PID Tuner
   *     // After you find values that you're happy with, you'll have to
   *     // set them in auton.cpp
   *     if (!pros::competition::is_connected()) {
   *       // Enable / Disable PID Tuner
   *       if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();
   *
   *       // Trigger the selected autonomous routine
   *       if (master.get_digital_new_press(DIGITAL_B)) autonomous();
   *
   *       chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
   *     }
   *
   *     chassis.opcontrol_tank();  // Tank control
   *
   *     // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
   *     pros::delay(ez::util::DELAY_TIME);
   *   }
   * }
   * \endcode
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
   * Adds a PID to the PID Tuner.
   *
   * This adds to both the default tuner and the full tuner.
   *
   * \param new_pid_and_name
   *        {"name", &pid.constants}
   *
   * \code{.cpp}
   * ez::PID lift_pid{20.0, 0.0, 100.0};
   *
   * void initialize() { chassis.pid_tuner_add({"Lift", &lift_pid.constants}); }
   * \endcode
   */
  void pid_tuner_add(const_and_name new_pid_and_name);

  /**
   * Vector used for a simplified PID Tuner
   */
  std::vector<const_and_name> pid_tuner_pids = {{"Drive PID Constants", &fwd_rev_drivePID.constants},
                                                {"Odom Angular PID Constants", &odom_angularPID.constants},
                                                {"Boomerang Angular PID Constants", &boomerangPID.constants},
                                                {"Heading PID Constants", &headingPID.constants},
                                                {"Turn PID Constants", &turnPID.constants},
                                                {"Swing PID Constants", &fwd_rev_swingPID.constants}};

  /**
   * Vector used for the full PID Tuner
   */
  std::vector<const_and_name> pid_tuner_full_pids = {{"Drive Forward PID Constants", &forward_drivePID.constants},
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
   * This scales the joystick output, it is not just a ceiling.  A full stick gives the speed limit, and a half stick gives half of it.
   *
   * \param speed
   *        the speed limit, 0 - 127.  A value above 127 is set to 127 and a negative value is treated as its magnitude
   *
   * \code{.cpp}
   * void initialize() {
   *   // Scales user control so a full stick gives 100 instead of 127
   *   chassis.opcontrol_speed_max_set(100);
   * }
   * \endcode
   */
  void opcontrol_speed_max_set(int speed);

  /**
   * Returns the max speed for user control.
   *
   * \code{.cpp}
   * void initialize() {
   *   chassis.opcontrol_speed_max_set(100);
   *   printf("%d\n", chassis.opcontrol_speed_max_get());  // Prints 100
   * }
   * \endcode
   */
  int opcontrol_speed_max_get();

  /**
   * Toggles vector scaling for arcade control.  True enables, false disables.
   *
   * \param enable
   *        true enables, false disables
   */
  void opcontrol_arcade_scaling(bool enable);

  /**
   * Returns true if vector scaling for arcade control is enabled, false if it is disabled.
   */
  bool opcontrol_arcade_scaling_enabled();

  /**
   * Current odom position.
   */
  pose odom_current = {0.0, 0.0, 0.0};

  /**
   * The default tracking task, used unless odom_tracking_set() is called.  Documented here for reference - most users will never call this directly.
   */
  void tracking_wheels_tracking();

  /**
   * Sets a new task to use for tracking.
   *
   * In this function, write the pose your tracking measured, either directly:
   *  - odom_current.x =
   *  - odom_current.y =
   *  - odom_current.theta =
   *
   * or with odom_xyt_set(), odom_xy_set(), odom_x_set(), odom_y_set() or odom_pose_set().  Both work the same: however
   * far your function moves the pose on a pass is how far the robot moved, for the xy D term (kD) and for knowing when the
   * robot has stopped.  A correction your function makes (snapping to a wall, a GPS update) counts as movement on that pass.
   *
   * When your own code calls one of those setters outside this function while it is running, the pass right after counts as
   * no movement for the xy D term.  The library cannot tell if your tracking function kept the pose that was set or wrote its
   * own over it, like a GPS does.  Setting the pose from your own code on every pass leaves the xy D term at 0.
   *
   * This function does not need to loop, that is done for you in EZ-Template.  EZ-Template calls it about every 10 ms from its own
   * background task, while that task holds the drive lock.  Setters like pid_drive_set() wait for that lock, so don't call pros::delay()
   * or anything else that blocks in your function, or every setter will wait on it.
   *
   * To go back to EZ-Template's own tracking, call drive_defaults_set() (it sets every other default again too).  Odom picks up
   * from the last pose your function wrote.
   *
   * \param tracking_task
   *        new function for tracking
   *
   * \code{.cpp}
   * // Would combine your sensors and return the robot's x position in inches
   * double my_sensor_fusion_x() { return 0.0; }
   *
   * // Would combine your sensors and return the robot's y position in inches
   * double my_sensor_fusion_y() { return 0.0; }
   *
   * void custom_tracking() {
   *   chassis.odom_current.x = my_sensor_fusion_x();
   *   chassis.odom_current.y = my_sensor_fusion_y();
   *   chassis.odom_current.theta = chassis.drive_angle_get();
   * }
   *
   * void initialize() {
   *   chassis.odom_tracking_set(custom_tracking);
   *   chassis.initialize();
   * }
   * \endcode
   */
  void odom_tracking_set(std::function<void(void)> tracking_task);

private:
  // Grants the host test suite (test/) access to private state and task
  // bodies (e.g. ez_auto_task(), pp_task()) so motions can be driven and
  // asserted on directly, since pros::Task never actually runs them here.
  friend struct DriveTestAccess;

  /**
   * Guards state shared between the ez_auto task and the public setters.
   * Recursive so nested public calls and user callbacks that call setters are safe.
   */
  ez::Lock<pros::RecursiveMutex> drive_mutex;

  // What motion a wait is waiting on.  A wait is for the motion that was set when it started, so when another task starts a new one (a
  // different mode, or a different target) the wait starts over on the new one.
  struct WaitSnapshot {
    explicit WaitSnapshot(Drive& drive) : d(drive), mode(drive.mode), chain_target(drive.chain_target_start), odom_target(drive.odom_target_start) {}
    bool changed() const {
      return d.mode != mode || d.chain_target_start != chain_target || d.odom_target_start.x != odom_target.x || d.odom_target_start.y != odom_target.y ||
             d.odom_target_start.theta != odom_target.theta;
    }
    Drive& d;
    e_mode mode;
    double chain_target;
    pose odom_target;
  };

  std::function<void(void)> tracking;
  void opcontrol_drive_activebrake_targets_set();
  double odom_smooth_weight_smooth = 0.0;
  double odom_smooth_weight_data = 0.0;
  double odom_smooth_tolerance = 0.0;
  bool odom_use_left = true;
  double odom_ime_track_width_left = 0.0;
  double odom_ime_track_width_right = 0.0;
  bool imu_calibrate_took_too_long = false;
  bool is_full_pid_tuner_enabled = false;
  std::vector<const_and_name>* used_pid_tuner_pids = &pid_tuner_pids;
  double opcontrol_speed_max = 127.0;
  bool arcade_vector_scaling = false;
  double curvature_point_turn_gain = 0.8;
  double prev_imu_value = 0;
  // odom privates
  std::vector<odom> pp_movements;
  std::vector<int> injected_pp_index;
  int pp_index = 0;
  // Sets max_speed and the slew caps only, no odom path rewrite. Assumes the caller already holds
  // drive_mutex. See its own doc comment in set_pid.cpp for why this exists separately from the
  // public pid_speed_max_set().
  void pid_speed_max_set_internal(int speed);
  std::vector<odom> smooth_path(std::vector<odom> ipath, double weight_smooth, double weight_data, double tolerance);
  double is_past_target(pose target, pose current);
  void raw_pid_odom_pp_set(std::vector<odom> imovements, bool slew_on);
  bool ptf1_running = false;
  std::vector<pose> find_point_to_face(pose current, pose target, drive_directions dir, bool set_global);
  void raw_pid_odom_ptp_set(odom imovement, bool slew_on, bool is_boomerang);
  // out_injected_pp_index null (the default): computes and publishes injected_pp_index itself, under
  // its own lock, exactly as before -- for a caller with no other locked publish to pair it with.
  // Non-null: skips its own publish and hands the computed index back through here instead, so a
  // caller that also needs to publish something else of its own (pp_movements, via
  // raw_pid_odom_pp_set()) can take ONE lock and publish both together -- see pid_odom_set() and
  // friends in set_odom_pid.cpp. Never partially published either way: injected_pp_index and
  // pp_movements must always change together under the same lock, or a reader that trusts them as a
  // consistent pair (pid_wait_until_index_started(), exit_conditions.cpp) could observe one updated
  // and the other stale.
  std::vector<odom> inject_points(std::vector<odom> imovements, std::vector<int>* out_injected_pp_index = nullptr);
  std::vector<pose> point_to_face = {{0, 0, 0}, {0, 0, 0}};
  double turn_is_toleranced(double target, double current, double input, double longest, double shortest);
  double turn_short(double target, double current, bool print = false);
  double turn_long(double target, double current, bool print = false);
  double new_turn_target_compute(double target, double current, ez::e_angle_behavior behavior);
  void odom_heading_target_resolve();
  ez::exit_output xy_exit_condition();
  double turn_left(double target, double current, bool print = false);
  double turn_right(double target, double current, bool print = false);
  bool imu_calibration_complete = false;
  bool last_was_autonomous = false;
  bool last_was_disabled = false;
  int loading_bar_last_x = 0;

  // IMU watchdog state: every IMU ever constructed (never shrinks), and
  // per-port health tracking used by check_imu_task() to eject/re-add IMUs.
  std::deque<pros::Imu*> all_imus;
  std::map<int, int> imu_stuck_passes;
  double last_good_angle = 0.0;
  double watchdog_l_last = 0.0, watchdog_r_last = 0.0;

  // Same fallback pattern as last_good_angle, for drive_sensor_left_raw()/right_raw(): the
  // last raw reading that wasn't a PROS_ERR/PROS_ERR_F/non-finite sensor-read failure.
  int last_good_raw_left = 0;
  int last_good_raw_right = 0;

  // Keeps drive_sensor_left()/right() continuous when ticks per inch changes (drive_rpm_set()): the inches
  // the sensor read at the last change, and the raw count it was read at. The reading is
  // offset_in + (raw - offset_raw) / ticks per inch, so a change in scale only applies to the distance traveled after it.
  // All four are 0 until a change, and drive_sensor_reset() zeroes them with the sensors. Written under drive_mutex.
  double sensor_offset_in_left = 0.0;
  double sensor_offset_in_right = 0.0;
  int sensor_offset_raw_left = 0;
  int sensor_offset_raw_right = 0;

  // Cross-check state for imu_drift_deg, kept separate from the stuck/eject
  // tracking above -- disagreement between good IMUs is a different failure
  // mode than one of them freezing. See imu_drift_threshold_set().
  double imu_drift_threshold_deg = 15.0;
  int imu_drift_passes = 0;

  bool is_swing_slew_enabled(e_swing type, double target, double current, e_angle_behavior behavior);
  void swing_set_internal(e_swing type, double target, int speed, int opposite_speed, e_angle_behavior behavior, bool slew_on);
  bool slew_reenables_when_max_speed_changes = true;
  int slew_min_when_it_enabled = 0;
  bool slew_will_enable_later = false;
  bool current_slew_on = false;
  bool is_odom_turn_bias_enabled = true;
  bool odom_turn_bias_enabled();
  void odom_turn_bias_enable(bool set);
  // Set every tick by ptp_task() (pid_tasks.cpp) right after it computes turn bias's xy_out scale.
  // True exactly when turn bias has fully zeroed xy_out to prioritize turning: the robot is not
  // translating then, but it isn't stalled either.
  bool xy_translation_bias_gated = false;
  double angle_rad = 0.0;
  double global_track_width = 0.0;
  bool odometry_enabled = true;
  pose odom_target = {0.0, 0.0, 0.0};
  pose odom_second_to_last = {0.0, 0.0, 0.0};
  pose odom_start = {0.0, 0.0, 0.0};
  pose odom_target_start = {0.0, 0.0, 0.0};
  pose turn_to_point_target = {0.0, 0.0, 0.0};
  void turn_set_internal(double target, int speed, e_angle_behavior behavior, bool slew_on);
  double odom_imu_start = 0.0;
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
  // xyPID's "sensor" (new_current_fake) and how much it moved on the last pass (xy_delta_fake).  It moves by how much
  // the robot's own movement changed xyPID's error on that pass (ptp_task()), so xyPID's derivative is the robot's real
  // speed toward its target, wherever on the field the robot is.
  double xy_delta_fake = 0.0;
  double new_current_fake = 0.0;
  pose xy_last_pose{0.0, 0.0, 0.0};  // odom pose at the end of the last tracking pass
  // how far odom moved over the last tracking pass, not counting pose sets from the team's code (0 on the pass after one with
  // custom tracking).  A custom tracking function's own setter calls are tracking and do count.
  pose xy_pose_delta{0.0, 0.0, 0.0};
  bool xy_last_pose_valid = false;          // false until a tracking pass has run, after tracking was paused or reset, and after a non-finite pose
  bool was_odom_just_set = false;           // a pose set happened since the last tracking pass
  bool tracking_is_custom = false;          // odom_tracking_set() was called by the user (drive_defaults_set() clears it)
  pose xy_last_finite_pose{0.0, 0.0, 0.0};  // the last odom pose that was finite (a custom tracker may stop writing one)
  bool xy_movement_unknown = false;         // the last tracking pass had custom tracking and a pose set from the team's code
  // EZ-Template's own tracking was put back after a custom one, pick up from odom_current on the next tracking pass
  bool tracking_resync_pending = false;
  std::pair<float, float> decide_vert_sensor(ez::tracking_wheel* tracker, bool is_tracker_enabled, float ime = 0.0, float ime_track = 0.0);
  pose solve_xy_vert(float p_track_width, float current_t, float delta_vert, float delta_t);
  pose solve_xy_horiz(float p_track_width, float current_t, float delta_horiz, float delta_t);
  void tracking_prime();
  bool was_last_pp_mode_boomerang = false;
  bool global_forward_drive_slew_enabled = false;
  bool global_backward_drive_slew_enabled = false;
  bool global_forward_swing_slew_enabled = false;
  bool global_backward_swing_slew_enabled = false;
  double turn_tolerance = 3.0;
  bool global_turn_slew_enabled = false;
  e_angle_behavior current_angle_behavior = raw;
  e_angle_behavior default_swing_type = shortest;
  e_angle_behavior default_turn_type = shortest;
  e_angle_behavior default_odom_type = shortest;
  bool turn_biased_left = false;
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

  bool drive_toggle = true;
  bool print_toggle = true;
  bool no_pinned_exit_warned = false;
  int swing_min = 0;
  int turn_min = 0;
  bool practice_mode_is_on = false;
  bool slow_mode_is_on = false;
  double slow_mode_speed = 64.0;
  int swing_opposite_speed = 0;
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

  pros::controller_digital_e_t pid_tuner_increase = pros::E_CONTROLLER_DIGITAL_A;  // is this place good?
  pros::controller_digital_e_t pid_tuner_decrease = pros::E_CONTROLLER_DIGITAL_Y;  // ^yes this placement is fine :D
  pros::controller_digital_e_t pid_tuner_pageLeft = pros::E_CONTROLLER_DIGITAL_LEFT;
  pros::controller_digital_e_t pid_tuner_pageRight = pros::E_CONTROLLER_DIGITAL_RIGHT;
  pros::controller_digital_e_t pid_tuner_pageUp = pros::E_CONTROLLER_DIGITAL_UP;
  pros::controller_digital_e_t pid_tuner_pageDown = pros::E_CONTROLLER_DIGITAL_DOWN;

  /**
   * @brief
   * Get the scaled imu value from given imu
   *
   * \param imu
   *        the imu to read
   */
  double get_this_imu(pros::Imu* imu);
  /**
   * Private wait until for drive
   *
   * \param target
   *        distance from where the movement started to wait for, in inches
   */
  void wait_until_drive(double target);
  void wait_until_turn_swing(double target);
  // Expects an already-resolved, internal-frame absolute target.  Does not flip or re-resolve behavior.
  void wait_until_turn_swing_internal(double target);

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
   * Returns joystick value clipped to JOYSTICK_THRESHOLD
   *
   * \param joystick
   *        joystick value, -127 to 127
   */
  int clipped_joystick(int joystick);

  /**
   * Heading bool.
   */
  bool heading_on = true;

  /**
   * Tick per inch calculation.
   */
  double TICK_PER_REV = 0.0;
  double CIRCUMFERENCE = 0.0;

  double CARTRIDGE = 0.0;
  double WHEEL_DIAMETER = 0.0;

  /**
   * Max speed for autonomous.
   */
  int max_speed = 0;

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
  void check_imu_task();

  /**
   * Starting value for left/right
   */
  double l_start = 0;
  double r_start = 0;

  /**
   * Enable/disable modifying controller curve with controller.
   */
  bool disable_controller = false;  // True enables, false disables.  Disabled until the user turns it on.

  /**
   * Is tank drive running?
   */
  bool is_tank = false;

#define DRIVE_INTEGRATED 1
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
    int increase_timer = 0;
    pros::controller_digital_e_t button = pros::E_CONTROLLER_DIGITAL_A;
  };

  button_ l_increase_;
  button_ l_decrease_;
  button_ r_increase_;
  button_ r_decrease_;

  /**
   * Function for button presses.
   *
   * \param input_name
   *        state of the button that is being handled
   * \param button
   *        nonzero while the button is held
   * \param changeCurve
   *        changes the curve, runs when the button is first pressed and repeats while it is held
   * \param save
   *        saves the curve, runs once the button has been released for 250 ms after the curve changed
   */
  void button_press(button_* input_name, int button, std::function<void()> changeCurve, std::function<void()> save);

  /**
   * The left and right curve scalers.
   */
  double left_curve_scale = 0.0;
  double right_curve_scale = 0.0;

  /**
   * The curve text last written to the controller, and when.  The controller link can't take a write every 10 ms,
   * so the text is only written again when it changes, or slowly to keep the screen right after a reconnect.
   */
  std::string last_controller_text = "";
  uint32_t last_controller_text_ms = 0;

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
