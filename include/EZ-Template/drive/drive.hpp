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
#include "EZ-Template/util.hpp"
#include "okapi/api/units/QLength.hpp"
#include "pros/motors.h"

using namespace ez;

class Drive {
 public:
  /**
   * Joysticks will return 0 when they are within this number.  Set with joystick_threshold_set()
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
  e_swing swing_current;

  /**
   * Vector of pros motors for the left chassis.
   */
  std::vector<pros::Motor> motors_left;

  /**
   * Vector of pros motors for the right chassis.
   */
  std::vector<pros::Motor> motors_right;

  /**
   * Vector of pros motors that are disconnected from the drive.
   */
  std::vector<int> pto_active;

  /**
   * Inertial sensor.
   */
  pros::Imu imu;

  /**
   * Left tracking wheel.
   */
  pros::ADIEncoder tracker_left;

  /**
   * Right tracking wheel.
   */
  pros::ADIEncoder tracker_right;

  /**
   * Left rotation tracker.
   */
  pros::Rotation rotation_left;

  /**
   * Right rotation tracker.
   */
  pros::Rotation rotation_right;

  /**
   * PID objects.
   */
  PID pid_heading;
  PID pid_turn;
  PID pid_forward_drive;
  PID pid_left;
  PID pid_right;
  PID pid_backward_drive;
  PID pid_swing;

  /**
   * Current mode of the drive.
   */
  e_mode mode;

  /**
   * Sets current mode of drive.
   */
  void mode_set(e_mode p_mode);

  /**
   * Returns current mode of drive.
   */
  e_mode mode_get();

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
   *        Input {1, -2...}.  Make ports negative if reversed!
   * \param right_motor_ports
   *        Input {-3, 4...}.  Make ports negative if reversed!
   * \param imu_port
   *        Port the IMU is plugged into.
   * \param wheel_diameter
   *        Diameter of your drive wheels.  Remember 4" is 4.125"!
   * \param ticks
   *        Motor cartridge RPM
   * \param ratio
   *        External gear ratio, wheel gear / motor gear.
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio);

  /**
   * Creates a Drive Controller using encoders plugged into the brain.
   *
   * \param left_motor_ports
   *        Input {1, -2...}.  Make ports negative if reversed!
   * \param right_motor_ports
   *        Input {-3, 4...}.  Make ports negative if reversed!
   * \param imu_port
   *        Port the IMU is plugged into.
   * \param wheel_diameter
   *        Diameter of your sensored wheels.  Remember 4" is 4.125"!
   * \param ticks
   *        Ticks per revolution of your encoder.
   * \param ratio
   *        External gear ratio, wheel gear / sensor gear.
   * \param left_tracker_ports
   *        Input {1, 2}.  Make ports negative if reversed!
   * \param right_tracker_ports
   *        Input {3, 4}.  Make ports negative if reversed!
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports);

  /**
   * Creates a Drive Controller using encoders plugged into a 3 wire expander.
   *
   * \param left_motor_ports
   *        Input {1, -2...}.  Make ports negative if reversed!
   * \param right_motor_ports
   *        Input {-3, 4...}.  Make ports negative if reversed!
   * \param imu_port
   *        Port the IMU is plugged into.
   * \param wheel_diameter
   *        Diameter of your sensored wheels.  Remember 4" is 4.125"!
   * \param ticks
   *        Ticks per revolution of your encoder.
   * \param ratio
   *        External gear ratio, wheel gear / sensor gear.
   * \param left_tracker_ports
   *        Input {1, 2}.  Make ports negative if reversed!
   * \param right_tracker_ports
   *        Input {3, 4}.  Make ports negative if reversed!
   * \param expander_smart_port
   *        Port the expander is plugged into.
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio, std::vector<int> left_tracker_ports, std::vector<int> right_tracker_ports, int expander_smart_port);

  /**
   * Creates a Drive Controller using rotation sensors.
   *
   * \param left_motor_ports
   *        Input {1, -2...}.  Make ports negative if reversed!
   * \param right_motor_ports
   *        Input {-3, 4...}.  Make ports negative if reversed!
   * \param imu_port
   *        Port the IMU is plugged into.
   * \param wheel_diameter
   *        Diameter of your sensored wheels.  Remember 4" is 4.125"!
   * \param ratio
   *        External gear ratio, wheel gear / sensor gear.
   * \param left_tracker_port
   *        Make ports negative if reversed!
   * \param right_tracker_port
   *        Make ports negative if reversed!
   */
  Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ratio, int left_rotation_port, int right_rotation_port);

  /**
   * Sets drive defaults.
   */
  void defaults_set();

  /////
  //
  // User Control
  //
  /////

  /**
   * Sets the chassis to controller joysticks using tank control.  Run is usercontrol.
   * This passes the controller through the curve functions, but is disabled by default.  Use toggle_controller_curve_modifier() to enable it.
   */
  void tank();

  /**
   * Sets the chassis to controller joysticks using standard arcade control.  Run is usercontrol.
   * This passes the controller through the curve functions, but is disabled by default.  Use toggle_controller_curve_modifier() to enable it.
   *
   * \param stick_type
   *        ez::SINGLE or ez::SPLIT control
   */
  void arcade_standard(e_type stick_type);

  /**
   * Sets the chassis to controller joysticks using flipped arcade control.  Run is usercontrol.
   * This passes the controller through the curve functions, but is disabled by default.  Use toggle_controller_curve_modifier() to enable it.
   *
   * \param stick_type
   *        ez::SINGLE or ez::SPLIT control
   */
  void arcade_flipped(e_type stick_type);

  /**
   * Initializes left and right curves with the SD card, recommended to run in initialize().
   */
  void curve_init_sd();

  /**
   * Sets the default joystick curves.
   *
   * \param left
   *        Left default curve.
   * \param right
   *        Right default curve.
   */
  void curve_set_default(double left, double right = 0);

  /**
   * Runs a P loop on the drive when the joysticks are released.
   *
   * \param kp
   *        Constant for the p loop.
   */
  void active_brake_set(double kp);

  /**
   * Enables/disables modifying the joystick input curves with the controller.  True enables, false disables.
   *
   * \param input
   *        bool input
   */
  void curve_toggle_modify_with_controller(bool toggle);

  /**
   * Sets buttons for modifying the left joystick curve.
   *
   * \param decrease
   *        a pros button enumerator
   * \param increase
   *        a pros button enumerator
   */
  void curve_buttons_left_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

  /**
   * Sets buttons for modifying the right joystick curve.
   *
   * \param decrease
   *        a pros button enumerator
   * \param increase
   *        a pros button enumerator
   */
  void curve_buttons_right_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

  /**
   * Outputs a curve from 5225A In the Zone.  This gives more control over the robot at lower speeds.  https://www.desmos.com/calculator/rcfjjg83zx
   *
   * \param x
   *        joystick input
   */
  double curve_left_function(double x);

  /**
   * Outputs a curve from 5225A In the Zone.  This gives more control over the robot at lower speeds.  https://www.desmos.com/calculator/rcfjjg83zx
   *
   * \param x
   *        joystick input
   */
  double curve_right_function(double x);

  /**
   * Sets a new threshold for the joystick.  The joysticks wil not return a value if they are within this.
   *
   * \param threshold
   *        new threshold
   */
  void joystick_threshold_set(int threshold);

  /**
   * Resets drive sensors at the start of opcontrol.
   */
  void drive_sensors_reset_opcontrol();

  /**
   * Sets minimum slew distance constants.
   *
   * \param l_stick
   *        input for left joystick
   * \param r_stick
   *        input for right joystick
   */
  void joystick_threshold_opcontrol(int l_stick, int r_stick);

  /////
  //
  // PTO
  //
  /////

  /**
   * Checks if the motor is currently in pto_list.  Returns true if it's already in pto_list.
   *
   * \param check_if_pto
   *        motor to check.
   */
  bool pto_check(pros::Motor check_if_pto);

  /**
   * Adds motors to the pto list, removing them from the drive.
   *
   * \param pto_list
   *        list of motors to remove from the drive.
   */
  void pto_add(std::vector<pros::Motor> pto_list);

  /**
   * Removes motors from the pto list, adding them to the drive.  You cannot use the first index in a pto.
   *
   * \param pto_list
   *        list of motors to add to the drive.
   */
  void pto_remove(std::vector<pros::Motor> pto_list);

  /**
   * Adds/removes motors from drive.  You cannot use the first index in a pto.
   *
   * \param pto_list
   *        list of motors to add/remove from the drive.
   * \param toggle
   *        if true, adds to list.  if false, removes from list.
   */
  void pto_toggle(std::vector<pros::Motor> pto_list, bool toggle);

  /////
  //
  // PROS Wrapers
  //
  /////

  /**
   * Sets the chassis to voltage.  Disables PID when called.
   *
   * \param left
   *        voltage for left side, -127 to 127
   * \param right
   *        voltage for right side, -127 to 127
   */
  void tank_set(int left, int right);

  /**
   * Changes the way the drive behaves when it is not under active user control
   *
   * \param brake_type
   *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
   */
  void drive_brake_set(pros::motor_brake_mode_e_t brake_type);

  /**
   * Sets the limit for the current on the drive.
   *
   * \param mA
   *        input in milliamps
   */
  void drive_current_limit_set(int mA);

  /**
   * Toggles set drive in autonomous. True enables, false disables.
   */
  void auto_drive_toggle(bool toggle);

  /**
   * Toggles printing in autonomous. True enables, false disables.
   */
  void auto_print_toggle(bool toggle);

  /////
  //
  // Telemetry
  //
  /////

  /**
   * The position of the right motor.
   */
  double sensor_right();

  /**
   * The position of the right motor.
   */
  int sensor_right_raw();

  /**
   * The velocity of the right motor.
   */
  int velocity_right();

  /**
   * The watts of the right motor.
   */
  double mA_right();

  /**
   * Return TRUE when the motor is over current.
   */
  bool over_current_right();

  /**
   * The position of the left motor.
   */
  double sensor_left();

  /**
   * The position of the left motor.
   */
  int sensor_left_raw();

  /**
   * The velocity of the left motor.
   */
  int velocity_left();

  /**
   * The watts of the left motor.
   */
  double mA_left();

  /**
   * Return TRUE when the motor is over current.
   */
  bool over_current_left();

  /**
   * Reset all the chassis motors, recommended to run at the start of your autonomous routine.
   */
  void drive_sensors_reset();

  /**
   * Resets the current gyro value.  Defaults to 0, recommended to run at the start of your autonomous routine.
   *
   * \param new_heading
   *        New heading value.
   */
  void imu_reset(double new_heading = 0);

  /**
   * Returns the current gyro value.
   */
  double imu_get();

  /**
   * Calibrates the IMU, recommended to run in initialize().
   *
   * \param run_loading_animation
   *        bool for running loading animation
   */
  bool imu_calibrate(bool run_loading_animation = true);

  /**
   * Loading display while the IMU calibrates.
   */
  void imu_loading_display(int iter);

  /**
   * Practice mode for driver practice that shuts off the drive if you go max speed.
   *
   * @param toggle True if you want this mode enables and False if you want it disabled.
   */
  void practice_mode_toggle(bool toggle);

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
  void pid_drive_set(okapi::QLength p_target, int speed, bool slew_on = false, bool toggle_heading = true);

  /**
   * Sets the robot to turn using PID.
   *
   * \param target
   *        target value in degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_turn_set(double target, int speed);

  /**
   * Sets the robot to turn relative to current heading using PID.
   *
   * \param target
   *        target in degrees relative to current heading
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_relative_turn_set(double target, int speed);

  /**
   * Turn using only the left or right side.
   *
   * \param type
   *        L_SWING or R_SWING
   * \param target
   *        target value in degrees
   * \param speed
   *        0 to 127, max speed during motion
   */
  void pid_swing_set(e_swing type, double target, int speed);

  /**
   * Resets all PID targets to 0.
   */
  void pid_targets_reset();

  /**
   * Sets heading of gyro and target of PID.
   */
  void angle_set(double angle);

  /**
   * Lock the code in a while loop until the robot has settled.
   */
  void drive_wait_exit();

  /**
   * Lock the code in a while loop until this position has passed.
   *
   * \param target
   *        when driving, this is inches.  when turning, this is degrees.
   */
  void drive_wait_distance(double target);

  /**
   * Lock the code in a while loop until this position has passed for driving with okapi units.
   *
   * \param target
   *        for driving, using okapi units
   */
  void drive_wait_distance(okapi::QLength target);

  /**
   * Autonomous interference detection.  Returns true when interfered, and false when nothing happened.
   */
  bool interfered = false;

  /**
   * @brief Set the ratio of the robot
   *
   * @param ratio
   *        ratio of the gears
   */
  void ratio_set(double ratio);
  /**
   * Changes max speed during a drive motion.
   *
   * \param speed
   *        new clipped speed
   */
  void max_speed_set(int speed);

  /**
   * @brief Set the drive pid constants object
   *
   * @param p           kP
   * @param i           kI
   * @param d           kD
   * @param p_start_i   start_I
   */
  void pid_drive_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * @brief Set the turn pid constants object
   *
   * @param p           kP
   * @param i           kI
   * @param d           kD
   * @param p_start_i   start_I
   */
  void pid_turn_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * @brief Set the swing pid constants object
   *
   * @param p           kP
   * @param i           kI
   * @param d           kD
   * @param p_start_i   start_I
   */
  void pid_swing_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * @brief Set the heading pid constants object
   *
   * @param p           kP
   * @param i           kI
   * @param d           kD
   * @param p_start_i   start_I
   */
  void pid_heading_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * @brief Set the forward pid constants object
   *
   * @param p           kP
   * @param i           kI
   * @param d           kD
   * @param p_start_i   start_I
   */
  void pid_drive_forward_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * @brief Set the backwards pid constants object
   *
   * @param p           kP
   * @param i           kI
   * @param d           kD
   * @param p_start_i   start_I
   */
  void pid_drive_backwards_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);

  /**
   * Sets minimum power for swings when kI and startI are enabled.
   *
   * \param min
   *        new clipped speed
   */
  void swing_min_set(int min);

  /**
   * The minimum power for turns when kI and startI are enabled.
   *
   * \param min
   *        new clipped speed
   */
  void turn_min_set(int min);

  /**
   * Returns minimum power for swings when kI and startI are enabled.
   */
  int swing_min_get();

  /**
   * Returns minimum power for turns when kI and startI are enabled.
   */
  int turn_min_get();

  /**
   * Sets minimum slew speed constants.
   *
   * \param fwd
   *        minimum power for forward drive pd
   * \param rev
   *        minimum power for backwards drive pd
   */
  void slew_min_power_set(int fwd, int rev);

  /**
   * Sets minimum slew distance constants.
   *
   * \param fwd
   *        minimum distance for forward drive pd, okapi unit
   * \param rev
   *        minimum distance for backwards drive pd, okapi unit
   */
  void slew_distance_set(okapi::QLength fwd, okapi::QLength rev);

  /**
   * Set's constants for drive exit conditions.
   *
   * \param p_small_exit_time
   *        Sets small_exit_time.  Timer for to exit within smalL_error.
   * \param p_small_error
   *        Sets smalL_error. Timer will start when error is within this.  Okapi unit.
   * \param p_big_exit_time
   *        Sets big_exit_time.  Timer for to exit within big_error.
   * \param p_big_error
   *        Sets big_error. Timer will start when error is within this. Okapi unit.
   * \param p_velocity_exit_time
   *        Sets velocity_exit_time.  Timer will start when velocity is 0.
   */
  void drive_exit_condition_set(int p_small_exit_time, okapi::QLength p_small_error, int p_big_exit_time, okapi::QLength p_big_error, int p_velocity_exit_time, int p_mA_timeout);

  /**
   * Set's constants for turn exit conditions.
   *
   * \param p_small_exit_time
   *        Sets small_exit_time.  Timer for to exit within smalL_error.
   * \param p_small_error
   *        Sets smalL_error. Timer will start when error is within this.
   * \param p_big_exit_time
   *        Sets big_exit_time.  Timer for to exit within big_error.
   * \param p_big_error
   *        Sets big_error. Timer will start when error is within this.
   * \param p_velocity_exit_time
   *        Sets velocity_exit_time.  Timer will start when velocity is 0.
   */
  void turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout);

  /**
   * Set's constants for swing exit conditions.
   *
   * \param p_small_exit_time
   *        Sets small_exit_time.  Timer for to exit within smalL_error.
   * \param p_small_error
   *        Sets smalL_error. Timer will start when error is within this.
   * \param p_big_exit_time
   *        Sets big_exit_time.  Timer for to exit within big_error.
   * \param p_big_error
   *        Sets big_error. Timer will start when error is within this.
   * \param p_velocity_exit_time
   *        Sets velocity_exit_time.  Timer will start when velocity is 0.
   */
  void swing_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout);

  /**
   * Returns current tick_per_inch()
   */
  double tick_per_inch_get();

  /**
   * Returns current tick_per_inch()
   */
  void curve_modify_with_controller();

  // Slew
  struct slew_ {
    int sign = 0;
    double error = 0;
    double x_intercept = 0;
    double y_intercept = 0;
    double slope = 0;
    double output = 0;
    bool enabled = false;
    double speed_max = 0;
  };

  slew_ slew_left;
  slew_ slew_right;

  /**
   * Initialize slew.
   *
   * \param input
   *        slew_ enum
   * \param slew_on
   *        is slew on?
   * \param speed_max
   *        target speed during the slew
   * \param target
   *        target sensor value
   * \param current
   *        current sensor value
   * \param start
   *        starting position
   * \param backwards
   *        slew direction for constants
   */
  void slew_initialize(slew_ &input, bool slew_on, double speed_max, double target, double current, double start, bool backwards);

  /**
   * Calculate slew.
   *
   * \param input
   *        slew_ enum
   * \param current
   *        current sensor value
   */
  double slew_calculate(slew_ &input, double current);

 private:  // !Auton
  bool drive_toggle = true;
  bool print_toggle = true;
  int swing_min = 0;
  int turn_min = 0;
  bool practice_mode_is_on = false;

  /**
   * Private wait until for drive
   */
  void drive_wait_until(double target);

  /**
   * Sets the chassis to voltage.
   *
   * \param left
   *        voltage for left side, -127 to 127
   * \param right
   *        voltage for right side, -127 to 127
   */
  void tank_set_private(int left, int right);

  /**
   * Returns joystick value clipped to JOYSTICK_THRESH
   */
  int joystick_clamped(int joystick);

  /**
   * Heading bool.
   */
  bool heading_on = true;

  /**
   * Active brake kp constant.
   */
  double active_brake_kp = 0;

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
  int speed_max;

  /**
   * Tasks
   */
  void pid_drive_task();
  void pid_swing_task();
  void pid_turn_task();
  void ez_auto_task();

  /**
   * Constants for slew
   */
  double SLEW_DISTANCE[2];
  double SLEW_MIN_POWER[2];

  /**
   * Starting value for left/right
   */
  double start_left = 0;
  double start_right = 0;

  /**
   * Enable/disable modifying controller curve with controller.
   */
  bool controller_disable = true;  // True enables, false disables.

  /**
   * Is tank drive running?
   */
  bool tank_active;

#define DRIVE_INTEGRATED 1
#define DRIVE_ADI_ENCODER 2
#define DRIVE_ROTATION 3

  /**
   * Is tracking?
   */
  int tracker_active = DRIVE_INTEGRATED;

  /**
   * Save input to sd card
   */
  void curve_left_save_sd();
  void curve_right_save_sd();

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

  button_ left_increase_;
  button_ left_decrease_;
  button_ right_increase_;
  button_ right_decrease_;

  /**
   * Function for button presses.
   */
  void button_press(button_ *input_name, int button, std::function<void()> changeCurve, std::function<void()> save);

  /**
   * The left and right curve scalers.
   */
  double curve_left_scale;
  double curve_right_scale;

  /**
   * Increase and decrease left and right curve scale.
   */
  void curve_left_scale_decrease();
  void curve_left_scale_increase();
  void curve_right_scale_decrease();
  void curve_right_scale_increase();
};
