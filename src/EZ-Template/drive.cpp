/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include <list>
class Drive {
  public:
    std::vector<int> LL_MOTOR_PORTS;
    std::vector<int> RR_MOTOR_PORTS;
    // IMU Port
    int IMU_PORT;

    pros::Imu gyro;

    pros::Controller master;
    //! Params
    const bool IS_SD_CARD = pros::usd::is_installed();
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

    // !Util
    bool is_reversed(int input) {
      if (input < 0)
        return true;
      return false;
    }


    // Set drive
    void
    set_left_chassis(int l) {
      for (int i : LL_MOTOR_PORTS) {
        pros::c::motor_move_voltage(abs(i), sgn(i) * l * (12000.0/127.0));
      }
    }

    void
    set_right_chassis(int r) {
      for (int i : RR_MOTOR_PORTS) {
        pros::c::motor_move_voltage(abs(i), sgn(i) * r * (12000.0/127.0));
      }
    }

    void
    set_tank(int l, int r) {
      set_left_chassis(l);
      set_right_chassis(r);
    }

    // Brake modes
    void
    set_drive_brake(pros::motor_brake_mode_e_t input) {
      for (int i : LL_MOTOR_PORTS) {
        pros::c::motor_set_brake_mode(abs(i), input);
      }
      for (int i : RR_MOTOR_PORTS) {
        pros::c::motor_set_brake_mode(abs(i), input);
      }
    }


    // Motor telemetry
    void
    reset_drive_sensor() {
      pros::c::motor_tare_position(abs(LL_MOTOR_PORTS.front()));
      pros::c::motor_tare_position(abs(RR_MOTOR_PORTS.front()));
    }

    int right_sensor()   { return pros::c::motor_get_position(abs(RR_MOTOR_PORTS.front())); }
    int right_velocity() { return pros::c::motor_get_actual_velocity(abs(RR_MOTOR_PORTS.front())); }

    int left_sensor()    { return pros::c::motor_get_position(abs(LL_MOTOR_PORTS.front())); }
    int left_velocity()  { return pros::c::motor_get_actual_velocity(abs(LL_MOTOR_PORTS.front())); }


    void  tare_gyro() { gyro.set_rotation(0); }
    float get_gyro()  { return gyro.get_rotation(); }


    bool
    imu_calibrate() {
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


    int
    sgn (int input) {
      if (input > 0)
        return 1;
      else if (input < 0)
        return -1;
      return 0;
    }

    double
    clip_num(double input, double max, double min) {
      if (input > max)
        return max;
      else if (input < min)
        return min;
      return input;
    }

    //!Joystick

    bool is_tank;

    ///
    // Increase / Decrease Input Curve
    ///

    // Set the starting
    float LEFT_CURVE_SCALE  = STARTING_LEFT_CURVE_SCALE;
    float RIGHT_CURVE_SCALE = STARTING_RIGHT_CURVE_SCALE;
    void
    init_curve_sd() {
      // If no SD card, return
      if (!IS_SD_CARD) return;

      // Set Starting Curve to SD Card
      // Left Curve
      FILE* l_usd_file_read = fopen("/usd/left_curve.txt", "r");
      char l_buf[5];
      fread(l_buf, 1, 5, l_usd_file_read);
      LEFT_CURVE_SCALE = std::stof(l_buf);
      fclose(l_usd_file_read);

      // Right Curve
      FILE* r_usd_file_read = fopen("/usd/right_curve.txt", "r");
      char r_buf[5];
      fread(r_buf, 1, 5, r_usd_file_read);
      RIGHT_CURVE_SCALE = std::stof(r_buf);
      fclose(r_usd_file_read);
    }

    // Save New left Curve
    void
    save_l_curve_sd() {
      // If no SD card, return
      if (!IS_SD_CARD) return;

      FILE* usd_file_write = fopen("/usd/left_curve.txt", "w");
      std::string in_str = std::to_string(LEFT_CURVE_SCALE);
      char const *in_c = in_str.c_str();
      fputs(in_c, usd_file_write);
      fclose(usd_file_write);
    }
    void
    save_r_curve_sd() {
      // If no SD card, return
      if (!IS_SD_CARD) return;

      FILE* usd_file_write = fopen("/usd/right_curve.txt", "w");
      std::string in_str = std::to_string(RIGHT_CURVE_SCALE);
      char const *in_c = in_str.c_str();
      fputs(in_c, usd_file_write);
      fclose(usd_file_write);
    }

    // Math to increase / decrease left and right curve, capping at 0
    void l_increase() {
      LEFT_CURVE_SCALE += CURVE_MODIFY_INTERVAL;
    }
    void l_decrease() {
      LEFT_CURVE_SCALE -= CURVE_MODIFY_INTERVAL;
      LEFT_CURVE_SCALE =  LEFT_CURVE_SCALE<0 ? 0 : LEFT_CURVE_SCALE;
    }
    void r_increase() {
      RIGHT_CURVE_SCALE += CURVE_MODIFY_INTERVAL;
    }
    void r_decrease() {
      RIGHT_CURVE_SCALE -= CURVE_MODIFY_INTERVAL;
      RIGHT_CURVE_SCALE =  RIGHT_CURVE_SCALE<0 ? 0 : RIGHT_CURVE_SCALE;
    }

    // Hold button constants
    const int WAIT = 500; // Time button needs to be held before increasing
    const int INCREASE_INTERVAL = 100; // After buttin is held for WAIT, curve scaler will increase every this amount of time

    // Struct for pointer values
    typedef struct {
      bool lock = false;
      bool release_reset = false;
      int release_timer = 0;
      int hold_timer = 0;
      int increase = INCREASE_INTERVAL;
    } button_;
    
        // Button logic
    // When tapped, run increase/decrease function once
    // When held, run increase/decrease function every INCREASE_INTERCAL time
    void
    button_press(button_ *input_name, int button, void (*f)(), void (*save)()) {
      if (button && !input_name->lock) {
        f();
        input_name->lock = true;
        input_name->release_reset = true;
      }
      else if (button && input_name->lock) {
        input_name->hold_timer+=DELAY_TIME;
        if (input_name->hold_timer > WAIT) {
          input_name->increase+=DELAY_TIME;
          if (input_name->increase > INCREASE_INTERVAL) {
            f();
            input_name->increase = 0;
          }
        }
      }
      else if (!button) {
        input_name->lock = false;
        input_name->hold_timer = 0;

        if (input_name->release_reset) {
          input_name->release_timer+=DELAY_TIME;
          if (input_name->release_timer > WAIT/2.0) {
            save();
            input_name->release_timer = 0;
            input_name->release_reset = false;
          }
        }
      }
    }

    // Creating variables for each button
    button_ l_increase_;
    button_ l_decrease_;
    button_ r_increase_;
    button_ r_decrease_;

    void
    modify_curve_with_controller() {
      button_press(&l_increase_, master.get_digital(INCREASE_L_CURVE), l_increase, save_l_curve_sd);
      button_press(&l_decrease_, master.get_digital(DECREASE_L_CURVE), l_decrease, save_l_curve_sd);
      if (!is_tank) {
        button_press(&r_increase_, master.get_digital(INCREASE_R_CURVE), r_increase, save_r_curve_sd);
        button_press(&r_decrease_, master.get_digital(DECREASE_R_CURVE), r_decrease, save_r_curve_sd);
      }

      auto sf = std::to_string(RIGHT_CURVE_SCALE);
      auto st = std::to_string(LEFT_CURVE_SCALE);
      if (!is_tank)
        master.set_text(2, 0, st+"   "+sf);
      else
        master.set_text(2, 0, st);
    }

    float
    right_curve_function(int x) {
      if (RIGHT_CURVE_SCALE != 0) {
        if (CURVE_TYPE)
          return (powf(2.718, -(RIGHT_CURVE_SCALE/10)) + powf(2.718, (abs(x)-127)/10) * (1-powf(2.718, -(RIGHT_CURVE_SCALE/10))))*x;
        else
          return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
      }
      return x;
    }

    float
    left_curve_function(int x) {
      if (LEFT_CURVE_SCALE != 0) {
        if (CURVE_TYPE)
          return (powf(2.718, -(LEFT_CURVE_SCALE/10)) + powf(2.718, (abs(x)-127)/10) * (1-powf(2.718, -(LEFT_CURVE_SCALE/10))))*x;
        else
          return powf(2.718, ((abs(x)-127)*LEFT_CURVE_SCALE)/100)*x;
      }
      return x;
    }


    // Tank control
    void chassis_tank() {
      is_tank = true;

      // Toggle for controller curve
      if (!DISABLE_CONTROLLER)
        modify_curve_with_controller();

      // Put the joysticks through the curve function
      int l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
      int r_stick = right_curve_function(master.get_analog(ANALOG_RIGHT_Y));

      // Threshold if joysticks don't come back to perfect 0
      if (abs(l_stick)>THRESH || abs(r_stick)>THRESH) {
        set_tank(l_stick, r_stick);
        reset_drive_sensor();
      }
      // When joys are released, run active brake (P) on drive
      else {
        set_tank((0-left_sensor())*ACTIVE_BRAKE_KP, (0-right_sensor())*ACTIVE_BRAKE_KP);
      }
    }


    // Arcade control standard
    void chassis_arcade_standard(e_type t) {
      is_tank = false;

      // Toggle for controller curve
      if (!DISABLE_CONTROLLER)
        modify_curve_with_controller();

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
      if (abs(l_stick)>THRESH || abs(r_stick)>THRESH) {
        set_tank(l_stick+r_stick, l_stick-r_stick);
        reset_drive_sensor();
      }
      // When joys are released, run active brake (P) on drive
      else {
        set_tank((0-left_sensor())*ACTIVE_BRAKE_KP, (0-right_sensor())*ACTIVE_BRAKE_KP);
      }
    }


    // Arcade control standard
    void chassis_arcade_flipped(e_type t) {
      is_tank = false;

      // Toggle for controller curve
      if (!DISABLE_CONTROLLER)
        modify_curve_with_controller();

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
      if (abs(l_stick)>THRESH || abs(r_stick)>THRESH) {
        set_tank(r_stick+l_stick, r_stick-l_stick);
        reset_drive_sensor();
      }
      // When joys are released, run active brake (P) on drive
      else {
        set_tank((0-left_sensor())*ACTIVE_BRAKE_KP, (0-right_sensor())*ACTIVE_BRAKE_KP);
      }
    }

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
    set_slew_min_power(int fw, int bw) {
      SLEW_MIN_POWER[FORWARD]  = fw;
      SLEW_MIN_POWER[BACKWARD] = bw;
    }

    void
    set_slew_distance(int fw, int bw) {
      SLEW_DISTANCE[FORWARD]  = fw;
      SLEW_DISTANCE[BACKWARD] = bw;
    }

    void
    set_fw_drive_constants(float kp, float kd) {
      drive_constant[FORWARD][0] = kp;
      drive_constant[FORWARD][1] = kd;
    }

    void
    set_bw_drive_constants(float kp, float kd) {
      drive_constant[BACKWARD][0] = kp;
      drive_constant[BACKWARD][1] = kd;
    }

    void
    set_heading_constants(float kp, float kd) {
      heading_kp = kp;
      heading_kd = kd;
    }

    void
    set_turn_constants(float kp, float ki, float kd) {
      gyro_kp = kp;
      gyro_ki = ki;
      gyro_kd = kd;
    }

    void
    set_turn_i_constants(float starting, int clipping) {
      start_i = starting;
      clipped_turn_i_speed = clipping;
    }

    void
    set_swing_constants(float kp, float kd) {
      swing_kp = kp;
      swing_kd = kd;
    }

    ///
    // Reset to default constants
    ///
    void
    reset_slew_min_power() {
      SLEW_MIN_POWER[FORWARD]  = FW_SLEW_MIN_POWER;
      SLEW_MIN_POWER[BACKWARD] = FW_SLEW_MIN_POWER;
    }

    void
    reset_slew_distance() {
      SLEW_DISTANCE[FORWARD]  = FW_SLEW_DISTANCE;
      SLEW_DISTANCE[BACKWARD] = FW_SLEW_DISTANCE;
    }

    void
    reset_fw_drive_constants() {
      fw_drive_kp = FW_DRIVE_KP;
      fw_drive_kd = FW_DRIVE_KD;
    }

    void
    reset_bw_drive_constants() {
      bw_drive_kp = BW_DRIVE_KP;
      bw_drive_kd = BW_DRIVE_KD;
    }

    void
    reset_heading_constants() {
      heading_kp = HEADING_KP;
      heading_kd = HEADING_KD;
    }

    void
    reset_turn_constants() {
      gyro_kp = GYRO_KP;
      gyro_ki = GYRO_KI;
      gyro_kd = GYRO_KD;
    }

    void
    reset_turn_i_constants() {
      start_i = START_I;
      clipped_turn_i_speed = CLIPPED_TURN_I_SPEED;
    }

    void
    reset_swing_constants() {
      swing_kp = SWING_KP;
      swing_kd = SWING_KD;
    }

    const float TICK_PER_REV  = (50*(3600/CART_RPM)) * RATIO; // with no cart, the encoder reads 50 counts per rotation
    const float CIRCUMFERENCE = WHEEL_DIA*M_PI;
    const float TICK_PER_INCH = (TICK_PER_REV/CIRCUMFERENCE);

    // "Enumerator" for drive type
    int active_drive_type = drive;

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
    drive_pid_task(void*) {
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
    // the task that will hold this function
    pros::Task drive_pid;

    void
    set_max_speed(int speed) {
      max_speed = speed;
    }

    void
    set_drive_pid(int type, float target, int speed, bool slew_on, bool toggle_heading) {
      // Global setup
      active_drive_type = type;
      max_speed = speed;
      slew = slew_on;
      stop = slew;
      heading_on = toggle_heading;

      // If drive or line, set targets to drive
      if (type==drive) {
        printf("Drive Started... Target Value: %f\n", target);
        l_start = left_sensor();
        r_start = right_sensor();

        if (target<l_start && target<r_start) {
          direction = BACKWARD;
        } else {
          direction = FORWARD;
        }

        l_target_encoder = l_start + (target*TICK_PER_INCH);
        r_target_encoder = r_start + (target*TICK_PER_INCH);

        l_sign = sgn(l_target_encoder-left_sensor());
        r_sign = sgn(r_target_encoder-right_sensor());

        l_x_intercept = l_start + (SLEW_DISTANCE[direction]*TICK_PER_INCH);
        r_x_intercept = r_start + (SLEW_DISTANCE[direction]*TICK_PER_INCH);

        l_y_intercept = max_speed * l_sign;
        r_y_intercept = max_speed * r_sign;

        l_slope = (SLEW_MIN_POWER[direction]-max_speed) / ((l_start+(SLEW_DISTANCE[direction]*TICK_PER_INCH))-0);
        r_slope = (SLEW_MIN_POWER[direction]-max_speed) / ((l_start+(SLEW_DISTANCE[direction]*TICK_PER_INCH))-0);
      }

      // If turn, set targets to angle
      else if (type == turn) {
        printf("Turn Started... Target Value: %f\n", target);
        gyro_target = target;
        gyro_sign = sgn(target - get_gyro());
      }

      // If l_turn, set targets to angle
      else if (type == l_swing || type == r_swing) {
        printf("Swing Started... Target Value: %f\n", target);
        gyro_target = target;
        swing_sign = sgn(target-get_gyro());
      }
    }

    bool
    drive_exit_condition(float l_target, float r_target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, bool wait_until = false) {
      static int i = 0, j = 0, k = 0, g = 0;

      // If the robot gets within the target, make sure it's there for small_timeout amount of time
      if (fabs(l_target-left_sensor())<start_small_counter_within && fabs(r_target-right_sensor())<start_small_counter_within) {
        j+=DELAY_TIME;
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
        i+=DELAY_TIME;
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
        k+=DELAY_TIME;
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
    turn_exit_condition(float target, int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout, bool wait_until = false) {
      static int i, j, k;

      // If the robot gets within the target, make sure it's there for small_timeout amount of time
      if (fabs(target-get_gyro())<start_small_counter_within) {
        j+=DELAY_TIME;
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
        i+=DELAY_TIME;
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
        k+=DELAY_TIME;
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
    wait_drive() {
      pros::delay(DELAY_TIME);

      // Parameters for exit condition function:
      // #3 - time the robot has to be within #2 of target
      // #4 - threshold for timer to start
      // #5 - time for if position is never reached
      // #6 - position for robot to be within to never reach target
      // #7 - velocity timeout

      if (active_drive_type==drive) {
        while (drive_exit_condition(l_target_encoder, r_target_encoder, 80, 50, 300, 150, 500)) {
          pros::delay(DELAY_TIME);
        }
      }
      else if (active_drive_type==turn) {
        while (turn_exit_condition(gyro_target, 100, 3, 500, 7, 500)) {
          pros::delay(DELAY_TIME);
        }
      }
      else if (active_drive_type==l_swing || active_drive_type==r_swing) {
        while (turn_exit_condition(gyro_target, 100, 3, 500, 7, 500)) {
          pros::delay(DELAY_TIME);
        }
        //while (turn_exit_condition(100, 3, 500, 7) && fabs(lf.get_power()-rf.get_power())<20) {
        //  pros::delay(delay_time);
        //}
      }
    }

    // Function to wait until a certain position is reached
    void
    wait_until(int input) {
      // If robot is driving...
      if (active_drive_type == drive) {
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
          else if (!drive_exit_condition(l_tar, r_tar, 80, 50, 300, 150, 500, true)) {
            run = false;
          }

          pros::delay(DELAY_TIME);
        }
      }

      // If robot is turning...
      else if (active_drive_type==turn || active_drive_type==l_swing || active_drive_type==r_swing) {
        // Calculate error between current and target (target needs to be an inbetween position)
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
          else if (!turn_exit_condition(input, 100, 3, 500, 7, 500, true)) {
            run = false;
          }

          pros::delay(DELAY_TIME);
        }
      }
    }
  
  Drive(std::vector<int> pLL_MOTOR_PORTS, std::vector<int> pRR_MOTOR_PORTS, int pIMU_PORT, pros::controller_id_e_t pController) {
    // initializes motor reversing
    LL_MOTOR_PORTS.assign(pLL_MOTOR_PORTS.begin(), pLL_MOTOR_PORTS.end());
    RR_MOTOR_PORTS.assign(pRR_MOTOR_PORTS.begin(), pRR_MOTOR_PORTS.end());
    // declares imu
    pros::IMU gyro(pIMU_PORT);
    // gets drive_pid_task going
    pros::Task drive_pid(drive_pid_task, nullptr, "drive_pid");

  }
};