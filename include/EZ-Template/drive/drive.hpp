/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "main.h"

#include "EZ-Template/Helper.hpp"
#include "EZ-Template/PID.hpp"

#include <iostream>
#include <functional>

class Drive {
    public:
        enum e_type{ k_single=0, k_split=1 };
        enum e_swing{ l_swing=0, r_swing=1 };

        e_swing current_swing;

        std::vector<pros::Motor> LeftMotors;
        std::vector<pros::Motor> RightMotors;
        pros::Imu gyro;
        pros::Controller master;

        PID headingPID;
        PID turnPID;
        PID forwardDrivePID;
        PID leftPID;
        PID rightPID;
        PID backwardDrivePID;
        PID swingPID;

        struct button_{
          bool lock = false;
          bool release_reset = false;
          int release_timer = 0;
          int hold_timer = 0;
          double increase_timer;
        };

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

        /**
        * Set Either the headingPID, turnPID, drivePID, activeBrakePID, or swingPID(IF NOT DONE PID WILL DEFAULT TO 0!)
        */
        void SetPIDConstants(PID pid, double kP, double kI, double kD, double startI);

        /**
         * Sets the chassis to voltage
         * \param input_l
         *        voltage for left side, -127 to 127
         * \param input_r
         *        voltage for right side, -127 to 127
        */
        void set_tank(int input_l, int input_r);

        /**
         * Sets the chassis to controller joysticks, using the best control, tank.
        */
        void chassis_tank();

        /**
         * Sets the chassis to controller joysticks, using standard arcade control.
         * \param t
         *        enum e_type, k_single or k_split control
        */
        void chassis_arcade_standard(e_type t);

        /**
         * Sets the chassis to controller joysticks, using flipped arcade control.
         * \param t
         *        enum e_type, k_single or k_split control
        */
        void chassis_arcade_flipped(e_type t);

        /**
         * The position of the right motor
        */
        int right_sensor();

        /**
         * The velocity of the right motor
        */
        int right_velocity();

        /**
         * The position of the left motor
        */

        int left_sensor();

        /**
         * The velocity of the left motor
        */
        int left_velocity();

        /**
         * Tares all the chassis motors (run before auton)
        */
        void reset_drive_sensor();

        /**
         * Resets the imu so that where the drive is pointing is zero in set_drive_pid(turn)
        */
        void tare_gyro();

        /**
         * Resets the imu so that where the drive is pointing is zero in set_drive_pid(turn)
         * Reccomended to run before auton
        */
        float get_gyro();

        /**
         * Calibrates the IMU, reccomended to run in initialize()
        */
        bool imu_calibrate();

        /**
         * Changes the way the drive behaves when it is not under active user control
         * \param input
         *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
        */
        void set_drive_brake(pros::motor_brake_mode_e_t input);

        /**
         * Changes max speed during a drive motion.
         * \param speed
         *        new clipped speed
        */
        void set_max_speed(int speed);

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
         * Changes max speed during a drive motion.
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
        //void wait_drive(double l_target, double r_target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, int delay_time = 10, bool wait_until = false);
        //void wait_turn(double target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, int delay_time = 10, bool wait_until = false);
        //void wait_swing(double target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, int delay_time = 10, bool wait_until = false);

        /**
         * Lock the code in a while loop until this position has passed.
         * \param input
         *        when driving, this is inches.  when turning, this is degrees.
        */
        void wait_until(double target);
        void set_curve_default(int left, int right);
        void init_curve_sd();

        void set_active_brake(double kp);

         struct exit_condition_{
          int small_exit_time;
          int small_error;
          int big_exit_time;
          int big_error;
          int velocity_exit_time;
        };

        exit_condition_ turn_exit;
        exit_condition_ swing_exit;
        exit_condition_ drive_exit;

        void set_exit_condition(exit_condition_ &type, int p_small_exit_time, int p_small_error, int p_big_exit_time, int p_big_error, int p_velocity_exit_time);

        void toggle_controller_curve_modifier(bool input);

    private:  // !Auton

    double active_brake_kp = 0;


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



      double TICK_PER_REV;
      double CIRCUMFERENCE;
      double TICK_PER_INCH;
      int max_speed;


      bool drive_exit_condition(double l_target, double r_target);
      bool turn_exit_condition(double target);
      bool swing_exit_condition(double target);

       void drive_pid_task();
       void swing_pid_task();
       void turn_pid_task();



      float unction(int x);


      double SLEW_DISTANCE [2];
      double SLEW_MIN_POWER[2];

      double l_start = 0;
      double r_start = 0;


      bool disable_controller = true; // If false, allows controller to modify CURVE_SCALE.
      bool is_tank;

      double left_curve_function(double x);
      double right_curve_function(double x);

       void save_l_curve_sd();
      void save_r_curve_sd();
     void modify_curve_with_controller();
     void button_press(button_ *input_name, int button, std::function<void()> changeCurve, std::function<void()> save);

        double left_curve_scale;
        double right_curve_scale;

        void l_decrease();
        void l_increase();
        void r_decrease();
        void r_increase();



      button_ l_increase_;
      button_ l_decrease_;
      button_ r_increase_;
      button_ r_decrease_;



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

        bool heading_on = true;

};
