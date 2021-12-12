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

using namespace ez;

class Drive {
 public:
  /**
    * Joysticks will return 0 when they are within this number.  Set with set_joystick_threshold()
    */
  int JOYSTICK_THRESHOLD;

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
    * Inertial sensor.
    */
  pros::Imu imu;

  /**
    * Left tracking wheel. 
    */
  pros::ADIEncoder left_tracker;

  /**
    * Right tracking wheel. 
    */
  pros::ADIEncoder right_tracker;

  /**
    * PID objects.
    */
  PID headingPID;
  PID turnPID;
  PID forward_drivePID;
  PID leftPID;
  PID rightPID;
  PID backward_drivePID;
  PID swingPID;

   
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
    *        Motor cartidge RPM
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
    * Sets drive defaults. 
    */
  void set_defaults();

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
    * Initializes left and right curves with the SD card, reccomended to run in initialize().
    */
  void init_curve_sd();

  /**
    * Sets the default joystick curves.
    *
    * \param left
    *        Left default curve.
    * \param right
    *        Right default curve.
    */
  void set_curve_default(double left, double right);

  /**
    * Runs a P loop on the drive when the joysticks are released.
    *
    * \param kp
    *        Constant for the p loop.
    */
  void set_active_brake(double kp);

  /**
    * Enables/disables modifying the joystick input curves with the controller.  True enables, false disables.
    *
    * \param input
    *        bool input
    */
  void toggle_modify_curve_with_controller(bool toggle);

  /**
    * Sets buttons for modifying the left joystick curve.
    *
    * \param decrease
    *        a pros button enumerator
    * \param increase
    *        a pros button enumerator
    */
  void set_left_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

  /**
    * Sets buttons for modifying the right joystick curve.
    *
    * \param decrease
    *        a pros button enumerator
    * \param increase
    *        a pros button enumerator
    */
  void set_right_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

  /**
    * Outputs a curve from 5225A In the Zone.  This gives more control over the robot at lower speeds.  https://www.desmos.com/calculator/rcfjjg83zx
    *
    * \param x
    *        joystick input
    */
  double left_curve_function(double x);

  /**
    * Outputs a curve from 5225A In the Zone.  This gives more control over the robot at lower speeds.  https://www.desmos.com/calculator/rcfjjg83zx
    *
    * \param x
    *        joystick input
    */
  double right_curve_function(double x);

  /**
    * Sets a new threshold for the joystick.  The joysticks wil not return a value if they are within this.
    *
    * \param threshold
    *        new threshold
    */
  void set_joystick_threshold(int threshold);

  /////
  //
  // PROS Wrapers
  //
  /////

  /**
    * Sets the chassis to voltage
    *
    * \param left
    *        voltage for left side, -127 to 127
    * \param right
    *        voltage for right side, -127 to 127
    */
  void set_tank(int left, int right);

  /**
    * Changes the way the drive behaves when it is not under active user control
    *
    * \param brake_type
    *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
    */
  void set_drive_brake(pros::motor_brake_mode_e_t brake_type);

  /**
    * Sets the limit for the current on the drive.
    *
    * \param mA
    *        input in milliamps
    */
  void set_drive_current_limit(int mA);

  /////
  //
  // Telemetry
  //
  /////

  /**
    * The position of the right motor.
    */
  int right_sensor();

  /**
    * The velocity of the right motor.
    */
  int right_velocity();

  /**
    * The watts of the right motor.
    */
  double right_mA();

  /**
    * Return TRUE when the motor is over current.
    */
  bool right_over_current();

  /**
    * The position of the left motor.
    */
  int left_sensor();

  /**
    * The velocity of the left motor.
    */
  int left_velocity();

  /**
    * The watts of the left motor.
    */
  double left_mA();

  /**
    * Return TRUE when the motor is over current.
    */
  bool left_over_current();

  /**
    * Reset all the chassis motors, reccomended to run at the start of your autonomous routine.
    */
  void reset_drive_sensor();

  /**
    * Reets the current gyro value.  Defaults to 0, reccomended to run at the start of your autonomous routine.
    *
    * \param new_heading
    *        New heading value.
    */
  void reset_gyro(double new_heading = 0);

  /**
    * Resets the imu so that where the drive is pointing is zero in set_drive_pid(turn)
    */
  double get_gyro();

  /**
    * Calibrates the IMU, reccomended to run in initialize().
    */
  bool imu_calibrate();

  /**
    * Loading display whlie the IMU calibrates. 
    */
  void imu_loading_display(int iter);

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
  void set_drive_pid(double target, int speed, bool slew_on = false, bool toggle_heading = true);

  /**
    * Sets the robot to turn using PID.
    *
    * \param target
    *        target value in degrees
    * \param speed
    *        0 to 127, max speed during motion
    */
  void set_turn_pid(double target, int speed);

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
  void set_swing_pid(e_swing type, double target, int speed);

  /**
    * Lock the code in a while loop until the robot has settled.
    */
  void wait_drive();

  /**
    * Lock the code in a while loop until this position has passed.
    *
    * \param target
    *        when driving, this is inches.  when turning, this is degrees.
    */
  void wait_until(double target);

  /**
    * Autonomous interference detection.  Returns true when interfered, and false when nothing happened.
    */
  bool interfered = false;

  /**
    * Changes max speed during a drive motion.
    *
    * \param speed
    *        new clipped speed
    */
  void set_max_speed(int speed);

  /**
    * Set Either the headingPID, turnPID, forwardPID, backwardPID, activeBrakePID, or swingPID
    * IF NOT DONE PID WILL DEFAULT TO 0!
    */
  void set_pid_constants(PID pid, double p, double i, double d, double p_start_i);

  /**
    * Sets minimum slew speed constants.
    *
    * \param fwd
    *        minimum power for forward drive pd
    * \param rev
    *        minimum power for backwards drive pd
    */
  void set_slew_min_power(int fwd, int rev);

  /**
    * Sets minimum slew distance constants.
    *
    * \param fw
    *        minimum distance for forward drive pd
    * \param bw
    *        minimum distance for backwards drive pd
    */
  void set_slew_distance(int fwd, int rev);

  /**
    * Exit condition struct.
    */
  struct exit_condition_ {
    int small_exit_time = 0;
    int small_error = 0;
    int big_exit_time = 0;
    int big_error = 0;
    int velocity_exit_time = 0;
    int mA_timeout = 0;
  };

  /**
    * Exit condition for turning.
    */
  exit_condition_ turn_exit;

  /**
    * Exit condition for swinging.
    */
  exit_condition_ swing_exit;

  /**
    * Exit condition for driving.
    */
  exit_condition_ drive_exit;

  /**
    * Set's constants for exit conditions.
    *
    * \param &type
    *        turn_exit, swing_exit, or drive_exit
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
  void set_exit_condition(exit_condition_ &type, int p_small_exit_time, int p_small_error, int p_big_exit_time, int p_big_error, int p_velocity_exit_timeint, int p_mA_timeout);

 private:  // !Auton
  void reset_drive_sensors_opcontrol();

  // Heading toggle
  bool heading_on = true;

  // Active brakeKP
  double active_brake_kp = 0;

  // Slew
  struct slew_ {
    int sign = 0;
    double error = 0;
    double x_intercept = 0;
    double y_intercept = 0;
    double slope = 0;
    double output = 0;
    bool enabled = false;
    double max_speed = 0;
  };

  slew_ left_slew;
  slew_ right_slew;

  double slew_calculate(slew_ &input, double current);
  void slew_initialize(slew_ &input, bool slew_on, double max_speed, double target, double current, double start, bool backwards);

  // Tick per inch calculation
  double TICK_PER_REV;
  double TICK_PER_INCH;
  double CIRCUMFERENCE;

  double get_tick_per_inch();

  double CARTRIDGE;
  double RATIO;
  double WHEEL_DIAMETER;

  // auto max speed
  int max_speed;

  // Exit conditions
  bool exit_condition(std::tuple<double, std::optional<double>> targets, exit_condition_ exitConditions, bool wait_until = false);
  // Tasks
  void drive_pid_task();
  void swing_pid_task();
  void turn_pid_task();
  void ez_auto_task();

  // Slew constants
  double SLEW_DISTANCE[2];
  double SLEW_MIN_POWER[2];

  // Starting value for left and right.
  double l_start = 0;
  double r_start = 0;

  // Enable/disable modifying controller curve with controller.
  bool disable_controller = true;  // True enables, false disables.

  // Is tank drive running?
  bool is_tank;

  // Is tracker?
  bool is_tracker = false;

  // Save input curve to SD card
  void save_l_curve_sd();
  void save_r_curve_sd();

  // Structs for the buttons.
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

  // Uses button presses to modify controller curve.
  void modify_curve_with_controller();

  // Function for button presses.
  void button_press(button_ *input_name, int button, std::function<void()> changeCurve, std::function<void()> save);

  // The left and right curve scalers.
  double left_curve_scale;
  double right_curve_scale;

  // Increase and decrease left and right curve scale.
  void l_decrease();
  void l_increase();
  void r_decrease();
  void r_increase();
};
