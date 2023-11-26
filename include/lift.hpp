/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <iostream>
#include <tuple>

#include "EZ-Template/util.hpp"
#include "pros/motors.h"

using namespace ez;

class Lift {
 public:
  /**
   * Joysticks will return 0 when they are within this number.  Set with set_joystick_threshold()
   */
  int JOYSTICK_THRESHOLD;

  /**
   * Global current brake mode.
   */
  pros::motor_brake_mode_e_t CURRENT_BRAKE = pros::E_MOTOR_BRAKE_HOLD;

  /**
   * Right rotation tracker.
   */
  pros::Rotation lift_rotation;

  /**
   * PID objects.
   */
  PID liftPID;

  /**
   * Current state of the lift.
   */
  l_state state;

  /**
   * Sets current mode of lift.
   */
  void set_mode(l_state t_state);

  /**
   * Returns current mode of lift.
   */
  l_state get_state();

  /**
   * Calibrates imu and initializes sd card to curve.
   */
  void initialize();

  /**
   * Tasks for autonomous.
   */
  pros::Task lift_auto;

  /**
   * Creates a Lift Controller using rotation sensor.
   *
   * \param left_motor_ports
   *        Input {1, -2...}.  Make ports negative if reversed!
   * \param right_motor_ports
   *        Input {-3, 4...}.  Make ports negative if reversed!
   * \param lift_tracker_port
   *        Make ports negative if reversed!
   */
  Lift(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int lift_rotation_port);

  /**
   * Changes the way the lift behaves when it is not under active user control
   *
   * \param brake_type
   *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
   */
  void set_lift_brake(pros::motor_brake_mode_e_t brake_type);

  /**
   * Sets the limit for the current on the lift.
   *
   * \param mA
   *        input in milliamps
   */
  void set_lift_current_limit(int mA);

  /////
  //
  // Debug
  //
  //////

  /**
   * Toggles set lift in autonomous. True enables, false disables.
   */
  void toggle_auto_lift(bool toggle);

  /**
   * Toggles printing in autonomous. True enables, false disables.
   */
  void toggle_auto_print(bool toggle);

  /////
  //
  // Telemetry
  //
  /////

  /**
   * The position of the lift.
   */
  int lift_sensor();

  /**
   * The velocity of the lift.
   */
  int lift_velocity();

  /**
   * The watts of the lift.
   */
  double lift_mA();

  /**
   * Return TRUE when the motors are over current.
   */
  bool lift_over_current();

  /////
  //
  // Autonomous Functions
  //
  /////

  /**
   * Sets the robot to move forward using PID.
   *
   * \param target
   *        target value in inches
   * \param speed
   *        0 to 127, max speed during motion
   * \param slew_on
   *        ramp up from slew_min to speed over slew_distance.  only use when you're going over about 14"
   * \param toggle_heading
   *        toggle for heading correction
   */
  void set_lift_pid(double target, int speed, bool slew_on = false, bool toggle_heading = true);

  /**
   * Lock the code in a while loop until the robot has settled.
   */
  void wait_lift();

  /**
   * Lock the code in a while loop until this position has passed.
   *
   * \param target
   *        when driving, this is inches.  when turning, this is degrees.
   */
  void wait_until(double target);
};
