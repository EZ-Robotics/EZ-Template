/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include "main.h"
#include "EZ-Template/Helper.hpp"
#include "EZ-Template/PID.hpp"
class Drive {
    public:
        enum e_type{ k_single=0, k_split=1 };
        std::vector<pros::Motor> LeftMotors;
        std::vector<pros::Motor> RightMotors;
        pros::Imu gyro;

        PID headingPID;
        PID turnPID;
        PID drivePID;
        PID swingPID;
        PID activeBrakePID;

        pros::Task drive_pid;
        pros::Task turn_pid;
        pros::Task swing_pid;

        /**
        * Give Sensor and Motor Ports (give a negative port if motor is reversed)
        */
        Drive(int leftMotorPorts[], int rightMotorPorts[], int imuPort, double wheelDiameter = 3.25, double motorCartridge = 600, double ratio = 1.66666666667);
        ~Drive();

        /**
        * Set Either the headingPID, turnPID, drivePID, activeBrakePID, or swingPID(IF NOT DONE PID WILL DEFAULT TO 0!)
        */
        void SetPIDConstants(PID, double kP, double kI, double kD, double startI);

        /**
         * Sets the chassis to voltage
         * \param input_l
         *        voltage for left side, -127 to 127
         * \param input_r
         *        voltage for right side, -127 to 127
        */
        void set_tank(int input_l, int input_r);
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
         * \param type
         *        type of drive motion.  drive, turn, l_swing, r_swing
         * \param target
         *        target value (inches for drive, degrees for turn)
         * \param speed
         *        0 to 127, max speed during motion
         * \param slew_on
         *        ramp up from slew_min to speed over slew_distance.  only use when you're going over about 14"
         * \param toggle_heading
         *        toggle for heading correction
        */
        void set_drive_pid(int type, float target, int speed, bool slew_on = false, bool toggle_heading = false);

        /**
         * Lock the code in a while loop until the robot has settled.
        */
        void wait_drive();

        /**
         * Lock the code in a while loop until this position has passed.
         * \param input
         *        when driving, this is inches.  when turning, this is degrees.
        */
        void wait_until(int input);
    private:
      void drive_pid_task(void*);
      float WHEEL_DIA; // Have the robot go 8ft forward and adjust this value until the robot actually goes 8ft
      float CART_RPM;   // Output RPM of the cart
      float RATIO; // External drive ratio (MUST BE DECIMAL)

      void chassis_tank();



      float left_curve_function(int x);
      float right_curve_function(int x);



        pros::Controller master;
        //! Params
        const int DELAY_TIME = 10;

        const bool  DISABLE_CONTROLLER = false; // If false, allows controller to modify CURVE_SCALE.
                                                // if true, locks STARTING_LEFT_CURVE_SCALE and STARTING_RIGHT_CURVE_SCALE to whatever it's set to.

        ///
        // Input Curve
        ///

        // Set the starting


        // Arcade uses two sticks to control, and you need control over the curve on each stick.
        // these buttons only do anything when DISABLE_CONTROLLER is FALSE
        #define DECREASE_L_CURVE pros::E_CONTROLLER_DIGITAL_LEFT  // decrease left joystick curve
        #define INCREASE_L_CURVE pros::E_CONTROLLER_DIGITAL_RIGHT // increase left joystick curve
        #define DECREASE_R_CURVE pros::E_CONTROLLER_DIGITAL_Y     // decrease right joystick curve (disabled when TANK_CONTROL = false)
        #define INCREASE_R_CURVE pros::E_CONTROLLER_DIGITAL_A     // increase right joystick curve (disabled when TANK_CONTROL = false)

        const double STARTING_LEFT_CURVE_SCALE  = 0;     // Starting value for curve (if 0, linear graph)
        const double STARTING_RIGHT_CURVE_SCALE = 0;     // Starting value for curve (if 0, linear graph) (disabled when TANK_CONTROL = false)
        const double CURVE_MODIFY_INTERVAL      = 0.1;   // When you modify the scaler with the controller, it will increase/decrease by this interval



        ///
        // Active Brake Constants
        //  -when both sticks are let go, run a p loop on the drive to make sure opponents can't push you
        //  -if you don't like active brake, set ACTIVE_BRAKE_KP to 0
        ///
        const float ACTIVE_BRAKE_KP = 0; // Constant for activebrake (increase this to make it more aggressive, 0.1 is recommended)




        // !Auton
        void drive_exit_condition(float l_target, float r_target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, bool wait_until = false);

        void turn_exit_condition(float target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, bool wait_until = false);

        /**
         * Sets minimum slew speed constants.
         * \param fw
         *        minimum power for forward drive pd
         * \param bw
         *        minimum power for backwards drive pd
        */
        void set_slew_min_power(int fw, int bw);

        /**
         * Sets minimum slew distance constants.
         * \param fw
         *        minimum distance for forward drive pd
         * \param bw
         *        minimum distance for backwards drive pd
        */
        void set_slew_distance (int fw, int bw);

        /**
         * Sets kp and kd for forward drive pd.
         * \param kp
         *        multipler for p
         * \param kd
         *        multipler for d
        */
        // these will get deleted when we get rid of setup.hpp
        void reset_slew_min_power();
        void reset_slew_distance();
        void reset_fw_drive_constants();
        void reset_bw_drive_constants();
        void reset_turn_constants();
        void reset_turn_i_constants();
        void reset_swing_constants();

        //IDK WHAT TO DO WITH THIS YET

/*



        ///
        // PID Default Constants
        //  -all constants have independent forward (FW) and backward (BW) constants
        ///

        // Slew
        const int FW_SLEW_MIN_POWER = 80; // Starting speed when slew is enabled
        const int BW_SLEW_MIN_POWER = 80;

        const int FW_SLEW_DISTANCE  = 7; // After robot has traveled this far, the robot will go max speed
        const int BW_SLEW_DISTANCE  = 7;

        // Drive
        const float FW_DRIVE_KP = 0.45;
        const float FW_DRIVE_KD = 5;

        const float BW_DRIVE_KP = 0.45;
        const float BW_DRIVE_KD = 5;

        // Heading
        const float HEADING_KP = 11;
        const float HEADING_KD = 20;

        // Gyro
        const float GYRO_KP = 5;
        const float GYRO_KI = 0.003;
        const float GYRO_KD = 35;
        const float START_I = 15; // Start I when error is this
        const int CLIPPED_TURN_I_SPEED = 30; // When I engages, this becomes max power

        // Swing
        const float SWING_KP = 12;
        const float SWING_KD = 35;

        // Minimum speed for driving and error to stop within
        // if speed goes below min_speed, robot travels at min_speed until it gets within min_error, where motors go 0
        const int MIN_SPEED = 0;
        const int MIN_ERROR = 0;*/

};
