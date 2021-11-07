/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "EZ-Template/Helper.hpp"
#include "EZ-Template/PID.hpp"

#include <iostream>
#include <functional>
#include <tuple>

using namespace ez;

class Drive {
  public:

    /**
     * Current swing type.
    */
    e_swing current_swing;


    /**
     * Vector of pros motors for the left and right chassis.
    */
    std::vector<pros::Motor> LeftMotors;
    std::vector<pros::Motor> RightMotors;

    /**
     * Inertial sensor.
    */
    pros::Imu imu;

    /**
     * Controller.
    */
    pros::Controller master;

    /**
     * PID objects.
    */
    PID headingPID;
    PID turnPID;
    PID forwardDrivePID;
    PID leftPID;
    PID rightPID;
    PID backwardDrivePID;
    PID swingPID;

    /**
     * Tasks for autonomous.
    */
    pros::Task drive_pid;
    pros::Task turn_pid;
    pros::Task swing_pid;



    /**
     * Creates A Controller.
     * Give Sensor Ports, Motor Ports (give a negative port if motor is reversed), Drive Measurements
     * Set PID Constants
    */
    Drive(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double motor_cartridge, double ratio);
    //~Drive(); // deconstructor (DELETE POINTERS HERE) (DONT LEEK MEMORY) (I DONT THINK WE NEED THIS)



    /////
    //
    // User Control
    //
    /////

    /**
     * Sets the chassis to controller joysticks using tank control.  Run is usercontrol.
     * This passes the controller through the curve functions, but is disabled by default.  Use toggle_controller_curve_modifier() to enable it.
    */
    void chassis_tank();

    /**
     * Sets the chassis to controller joysticks using standard arcade control.  Run is usercontrol.
     * This passes the controller through the curve functions, but is disabled by default.  Use toggle_controller_curve_modifier() to enable it.
     * \param t
     *        enum e_type, k_single or k_split control
    */
    void chassis_arcade_standard(e_type t);

    /**
     * Sets the chassis to controller joysticks using flipped arcade control.  Run is usercontrol.
     * This passes the controller through the curve functions, but is disabled by default.  Use toggle_controller_curve_modifier() to enable it.
     * \param t
     *        enum e_type, k_single or k_split control
    */
    void chassis_arcade_flipped(e_type t);

    /**
     * Initializes left and right curves with the SD card, reccomended to run in initialize().
    */
    void init_curve_sd();

    /**
     * Sets the chassis to controller joysticks using flipped arcade control.
     * \param t
     *        enum e_type, k_single or k_split control
    */
    void set_curve_default(int left, int right);

    /**
     * Runs a P loop on the drive when the joysticks are released.
     * \param kp
     *        double kp, constant for the p loop
    */
    void set_active_brake(double kp);

    /**
     * Enables/disables modifying the joystick input curves with the controller.  True enables, false disables.
     * \param input
     *        bool input
    */
    void toggle_controller_curve_modifier(bool toggle);

    /**
     * Sets buttons for modifying the left joystick curve.
     * \param decrease
     *        a pros button enumerator
     * \param increase
     *        a pros button enumerator
    */
    void left_curve_modify_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

    /**
     * Sets buttons for modifying the right joystick curve.
     * \param decrease
     *        a pros button enumerator
     * \param increase
     *        a pros button enumerator
    */
    void right_curve_modify_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

    /**
     * Outputs a curve from 5225A In the Zone.  This gives more control over the robot at lower speeds.  https://www.desmos.com/calculator/rcfjjg83zx
     * \param x
     *        joystick input
    */
    double left_curve_function(double x);

    /**
     * Outputs a curve from 5225A In the Zone.  This gives more control over the robot at lower speeds.  https://www.desmos.com/calculator/rcfjjg83zx
     * \param x
     *        joystick input
    */
    double right_curve_function(double x);



    /////
    //
    // PROS Wrapers
    //
    /////

    /**
     * Sets the chassis to voltage
     * \param input_l
     *        voltage for left side, -127 to 127
     * \param input_r
     *        voltage for right side, -127 to 127
    */
    void set_tank(int left, int right);

    /**
     * Changes the way the drive behaves when it is not under active user control
     * \param input
     *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
    */
    void set_drive_brake(pros::motor_brake_mode_e_t brake);



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
     * The position of the left motor.
    */
    int left_sensor();

    /**
     * The velocity of the left motor.
    */
    int left_velocity();

    /**
     * Reset all the chassis motors, reccomended to run at the start of your autonomous routine.
    */
    void reset_drive_sensor();

    /**
     * Reets the current gyro value.  Defaults to 0, reccomended to run at the start of your autonomous routine.
     * \param new_heading
     *        double new_heading, new heading value.
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



    /////
    //
    // Autonomous Functions
    //
    /////

    /**
     * Changes max speed during a drive motion.
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
     * Changes max speed during a drive motion.
     * \param target
     *        target value in degrees
     * \param speed
     *        0 to 127, max speed during motion
    */
    void set_turn_pid(double target, int speed);

    /**
     * Turn using only the left or right side.
     * \param type
     *        e_swing type, l_swing or r_swing
     * \param target
     *        double target, value in degrees
     * \param speed
     *        int speed, 0 to 127, max speed during motion
    */
    void set_swing_pid(e_swing type, double target, int speed);

    /**
     * Lock the code in a while loop until the robot has settled.
    */
    void wait_drive();

    /**
     * Lock the code in a while loop until this position has passed.
     * \param input
     *        when driving, this is inches.  when turning, this is degrees.
    */
    void wait_until(double target);

    /**
     * Changes max speed during a drive motion.
     * \param speed
     *        new clipped speed
    */
    void set_max_speed(int speed);

    /**
     * Set Either the headingPID, turnPID, forwardPID, backwardPID, activeBrakePID, or swingPID
     * IF NOT DONE PID WILL DEFAULT TO 0!
    */
    void SetPIDConstants(PID pid, double kP, double kI, double kD, double startI);

    /**
     * Sets minimum slew speed constants.
     * \param fw
     *        minimum power for forward drive pd
     * \param bw
     *        minimum power for backwards drive pd
    */
    void set_slew_min_power(int fwd, int rev);

    /**
     * Sets minimum slew distance constants.
     * \param fw
     *        minimum distance for forward drive pd
     * \param bw
     *        minimum distance for backwards drive pd
    */
    void set_slew_distance (int fwd, int rev);

    /**
     * Exit condition struct.
    */
    struct exit_condition_{
      int small_exit_time = 0;
      int small_error = 0;
      int big_exit_time = 0;
      int big_error = 0;
      int velocity_exit_time = 0;
    };

    /**
     * Exit condition for turning.
    */
    exit_condition_ turn_exit;

    /**
     * Exit condition for swinging. .
    */
    exit_condition_ swing_exit;

    /**
     * Exit condition for driving. .
    */
    exit_condition_ drive_exit;


    /**
     * Set's constants for exit conditions.
     * \param &type
     *        turn_exit, swing_exit, or drive_exit
     * \param p_small_exit_time
     *        sets small_exit_time.  time for to exit within smalL_error.
     * \param p_small_error
     *        sets smalL_error. timer will start when error is within this.
     * \param p_big_exit_time
     *        sets big_exit_time.  time for to exit within big_error.
     * \param p_big_error
     *        sets big_error. timer will start when error is within this.
     * \param p_velocity_exit_time
     *        sets velocity_exit_time.  timer will start when velocity is 0.
    */
    void set_exit_condition(exit_condition_ &type, int p_small_exit_time, int p_small_error, int p_big_exit_time, int p_big_error, int p_velocity_exit_time);



  private:  // !Auton

    // Heading toggle
    bool heading_on = true;

    // Active brakeKP
    double active_brake_kp = 0;

    // Slew
    struct slew_{
      int sign = 0;
      double error = 0;
      double x_intercept = 0;
      double y_intercept = 0;
      double slope = 0;
      double output = 0;
      bool enabled = false;
      double max_speed = 0;
    };

    slew_ l;
    slew_ r;

    double slew_calculate(slew_ &input, double current);

    // Tick per inch calculation
    double TICK_PER_REV;
    double CIRCUMFERENCE;
    double TICK_PER_INCH;
    int max_speed;

    // Exit conditions
  /*  bool drive_exit_condition(double l_target, double r_target);
    bool turn_exit_condition(double target);
    bool swing_exit_condition(double target);*/
    bool exit_condition(tuple<double, std::optional<double>> targets, exit_condition_ exitConditions);
    // Tasks
    void drive_pid_task();
    void swing_pid_task();
    void turn_pid_task();

    // ?
    float unction(int x);

    // Slew constants
    double SLEW_DISTANCE [2];
    double SLEW_MIN_POWER[2];

    // Starting value for left and right.
    double l_start = 0;
    double r_start = 0;

    // Enable/disable modifying controller curve with controller.
    bool disable_controller = true; // True enables, false disables.

    // Is tank drive running?
    bool is_tank;

    // Save input curve to SD card
    void save_l_curve_sd();
    void save_r_curve_sd();

    // Structs for the buttons.
    struct button_{
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
