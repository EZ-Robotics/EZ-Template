/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

enum e_type{ k_single=0, k_split=1 };

class Drive {
    public:
        std::vector<int> LL_MOTOR_PORTS = {};
        std::vector<int> RR_MOTOR_PORTS = {};
    private:
        // IMU Port
        int IMU_PORT;

        pros::Imu gyro;

        pros::Controller master;
        //! Params
        const int DELAY_TIME = 10;

        ///
        // Port Setup
        ///
        //const int MOTORS_PER_SIDE = 3; // Motors per side of drive

        // Make the port negative if it's reversed
        //const int  L_CHASSIS_PORTS[MOTORS_PER_SIDE] = {-4, 3, -5}; // Ports, the first number will be used for sensing
        //const int  R_CHASSIS_PORTS[MOTORS_PER_SIDE] = {7, -8, 6}; // Ports, the first number will be used for sensing



        ///
        // Wheel Size
        //  -for tick to inch conversion
        ///
        // Remember that 4" wheels are actually 4.125"!
        // If you tune an inch conversion not listed below, tell us you have it with a pull request!
        const float WHEEL_DIA = 3.25; // Have the robot go 8ft forward and adjust this value until the robot actually goes 8ft
        const float CART_RPM  = 600;   // Output RPM of the cart
        const float RATIO     = 1.66666666667; // External drive ratio (MUST BE DECIMAL)
        // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
        // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.



        ///
        // Input Curve Setup
        //  -adds an input curve to the joysticks, here https://www.desmos.com/calculator/rcfjjg83zx
        ///

        const bool  CURVE_TYPE         = true;  // true is red, false is blue in the demos link above
        const bool  DISABLE_CONTROLLER = false; // If false, allows controller to modify CURVE_SCALE.
                                                // if true, locks STARTING_LEFT_CURVE_SCALE and STARTING_RIGHT_CURVE_SCALE to whatever it's set to.

        ///
        // Input Curve
        ///

        // Set the starting
        float LEFT_CURVE_SCALE  = 0;
        float RIGHT_CURVE_SCALE = 0;

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
        const int THRESH = 5; // Joystick threshold to trigger activebrake



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
        const int MIN_ERROR = 0;
    public:
        // ! Joystick
        /**
         * Sets the chassis to controller joysticks, using tank control.
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
         * The left sensored motor.  Use this for telementry.
        */
        pros::Motor l_motor;
        
        /**
         * The right sensored motor.  Use this for telementry.
        */
        pros::Motor r_motor;

        /**
        *
        */
       float left_curve_function(int x);
       float right_curve_function(int x);

        // ! Util
        /**
         * Declares and prepares chassis motors for pid and joystick control (run in initialize())
         * \param l
         *        left chassis ports
         * \param r
         *        right chassis ports
        */
        void chassis_motor_init(std::list<int> l, std::list<int> r);

        /**
         * DEV
         * Sets the left side of the drive
         * \param left
         *        voltage for left side, -127 to 127
        */
        void set_left_chassis(int left);

        /**
         * DEV
         * Sets the right side of the drive
         * \param right
         *        voltage for right side, -127 to 127
        */
        void set_right_chassis(int right);

        /**
         * Sets the chassis to voltage
         * \param input_l
         *        voltage for left side, -127 to 127
         * \param input_r
         *        voltage for right side, -127 to 127
        */
        void set_tank(int input_l, int input_r);

        /**
         * Changes the way the drive behaves when it is not under active user control
         * \param input
         *        the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'
        */
        void set_drive_brake(pros::motor_brake_mode_e_t input);

        /**
         * DEV
         * The position of the right motor
        */
        int right_sensor();

        /**
         * DEV
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
         * Returns 1 if input is positive and -1 if input is negative
        */
        int sgn(int input);

        /**
         * Returns true if the input is < 0
        */
        bool is_reversed(int input);

        /**
         * Returns input restricted to min-max threshold
        */
        double clip_num(double input, double max, double min);

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
        void set_fw_drive_constants(float kp, float kd);

        /**
         * Sets kp and kd for backwards drive pd.
         * \param kp
         *        multipler for p
         * \param kd
         *        multipler for d
        */
        void set_bw_drive_constants(float kp, float kd);

        /**
         * Sets kp and kd for heading control.  This keeps the robot facing an angle while driving.
         * \param kp
         *        multipler for p
         * \param kd
         *        multipler for d
        */
        void set_heading_constants(float kp, float kd);

        /**
         * Sets kp, ki and kd for turning.
         * \param kp
         *        multipler for p
         * \param ki
         *        multipler for i
         * \param kd
         *        multipler for d
        */
        void set_turn_constants(float kp, float ki, float kd);

        /**
         * Set i constants
         * \param starting
         *        enable i when error is within this
         * \param clipping
         *        when within starting, clip speed to this
        */
        void set_turn_i_constants(float starting, int clipping);

        /**
         * Sets kp and kd for swing turns.
         * \param kp
         *        multipler for p
         * \param kd
         *        multipler for d
        */
        void set_swing_constants(float kp, float kd);

        // these will get deleted when we get rid of setup.hpp
        void reset_slew_min_power();
        void reset_slew_distance();
        void reset_fw_drive_constants();
        void reset_bw_drive_constants();
        void reset_turn_constants();
        void reset_turn_i_constants();
        void reset_swing_constants();


        /**
         * DEV
         * Drive pid task
        */
        void drive_pid_task(void*);
        pros::Task drive_pid;

        /**
         * DEV
         * Variables that should be an enum.
        */
        const int drive = 0;
        const int turn = 1;
        const int l_swing = 2;
        const int r_swing = 3;

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
};