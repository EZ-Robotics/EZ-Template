#include "main.h"
#include <string>

// Slew
// Variables that are arrays mean the first variable is for forward and the second is for backward
const int SLEW_MIN_POWER[2] = {80, 80}; // Starting speed for the slew
const int SLEW_DISTANCE [2] = {7, 7};  // Distance the robot slews at before reaching max speed

// Tick to Inch Conversion
const float WHEEL_SIZE    = 2.79124162145; 	// Have the robot go 6ft forward and adjust this value until the robot actually goes 6ft
const float TICK_PER_REV  = 300 * (5/3); // yes 300 is wrong but as long as the robot goes the correct distance it doesnt matter so dont bother me about it :)
const float CIRCUMFERENCE = WHEEL_SIZE*M_PI;
const float TICK_PER_INCH = (TICK_PER_REV/CIRCUMFERENCE);

const bool DEBUG = false;

// Forward Drive Constants
const float fw_drive_kp = 0.45;
const float fw_drive_kd = 5;

// Backward Drive Constants
const float bw_drive_kp = 0.375;
const float bw_drive_kd = 4;

// Minimum speed for driving and error to stop within
// if spede goes below min_speed, robot travels at min_speed until it gets within min_error, where motors go 0
const int min_speed = 0;
const int min_error = 0;

// Heading Constants (uses imu to keep robot driving straight)
const float heading_kp = 11;
const float heading_kd = 20;

// Turn Constants
const float gyro_kp = 5;
const float gyro_ki = 0.003;
const float gyro_kd = 35;
const int   start_i = 15; // Start I when error is this
const int clipped_turn_i_speed = 30; // When I engages, this becomes max power

// Swing Constants
const float swing_kp = 12;
const float swing_kd = 35;
const float swing_thresh = 5; // Threshold that the opposite side stops moving
const float swing_max_power = 110;

bool use_adjusted_constants = false;
float adjusted_swing_kp = 0, adjusted_swing_kd = 0;
void
adjusted_swing_constants(float kp, float kd) {
	use_adjusted_constants = true;
	adjusted_swing_kp = kp;
	adjusted_swing_kd = kd;
}
void
default_swing_constants() {
	use_adjusted_constants = false;
}

float swing_kp_return() { return use_adjusted_constants?adjusted_swing_kp:swing_kp; }
float swing_kd_return() { return use_adjusted_constants?adjusted_swing_kd:swing_kd; }


const float drive_constant[2][2] = {
	{fw_drive_kp, fw_drive_kd}, // Foward KP, KD
	{bw_drive_kp, bw_drive_kd}  // Backward KP, KD
};
int direction;



// "Enumerator" for drive type
int active_drive_type = drive;

// Drive PID with active straight code
// - it makes sure the angle of the robot is what it should be all the way through the movements,
// - turning if needed to keep it going straight
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
		// Different kP and kD are used for turning and heading
		if (active_drive_type==drive)
			gyro_output = (gyro_error*heading_kp) + (gyro_der*heading_kd);
		else if (active_drive_type==turn)
			gyro_output = (gyro_error*gyro_kp) + (gyro_integral*gyro_ki) + (gyro_der*gyro_kd);
		else if (active_drive_type==l_swing || active_drive_type==r_swing)
			gyro_output = (gyro_error*swing_kp_return()) + (gyro_der*swing_kd_return());

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
			gyro_output = clip_num(gyro_output, swing_max_power, -swing_max_power);
		}

		// Set drive based on m
		if (active_drive_type == drive) {
			if (heading_on) {
				l_output = left_output;
				r_output = right_output;
			} else {
				l_output = left_output  + gyro_output;
				r_output = right_output - gyro_output;
			}

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
		else if (active_drive_type == turn) {
			l_output =  gyro_output;
			r_output = -gyro_output;
		}
		else if (active_drive_type == l_swing) {
			l_output = gyro_output;
			if (!stop) {
				if (fabs(gyro_error) > swing_thresh)
					r_output = max_speed*swing_sign;
				else
					r_output = 0;
			}
			else {
				r_output = max_speed;
			}
		}
		else if (active_drive_type == r_swing) {
			r_output = -gyro_output;
			if (!stop) {
				if (fabs(gyro_error) > swing_thresh)
					l_output = max_speed*swing_sign;
				else
					l_output = 0;
			}
			else {
				l_output = max_speed;
			}
		}

		if (pros::millis()<2000) {
			set_tank(0, 0);
		} else {
			set_tank(l_output, r_output);
		}

		if (DEBUG) {
			if (active_drive_type == drive)  printf("\nle: %f   re: %f   l_der %f", left_error, right_error, l_der*drive_constant[direction][1]);
			if (active_drive_type==turn || active_drive_type==l_swing || active_drive_type==r_swing)   printf("\noutput: %f   error: %f   p: %f   i: %f   d: %f", gyro_output, gyro_error, gyro_error*gyro_kp, gyro_integral*gyro_ki, gyro_der*gyro_kd);
		}

    last_time       = pros::millis();
    last_l_error    = left_error;
    last_r_error    = right_error;
    last_gyro_error = gyro_error;

    pros::delay(10);
  }
}
pros::Task drive_pid(drive_pid_task, nullptr, "drive_pid");

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
		l_start = left_sensor();
		r_start = right_sensor();

		if (target<l_start && target<r_start) {
			direction = 1;
		} else {
			direction = 0;
		}

		l_target_encoder = l_start + (target*TICK_PER_INCH);
	  r_target_encoder = r_start + (target*TICK_PER_INCH);
		//printf("\nL %i, R %i, %f, %i", l_target_encoder, r_target_encoder, TICK_PER_INCH, l_start);

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
		gyro_target = target;
		gyro_sign = sgn(target - get_gyro());
		//printf("\nTURNING   Target: %ideg with", target);
	}

	// If l_turn, set targets to angle
	else if (type == l_swing || type == r_swing) {
		gyro_target = target;
		swing_sign = sgn(target - get_gyro());
	}
}

bool
drive_exit_condition(int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout) {
	static int i = 0, j = 0, k = 0, g = 0;
	static int delay_time = 10;

	// If the robot gets within the target, make sure it's there for small_timeout amount of time
	if (fabs(l_target_encoder-left_sensor())<start_small_counter_within && fabs(r_target_encoder-right_sensor())<start_small_counter_within) {
		j++;
		//printf("\nJ: %i", j/10);

		if (j>small_timeout/10) {
			printf("\nJ DRIVE BROKEN");
			return false;
		}
	}
	else {
		j = 0;
	}

	// If the robot is close to the target, start a timer.  If the robot doesn't get closer within
	// a certain amount of time, exit and continue.
	if (fabs(l_target_encoder-left_sensor())<start_big_counter_within && fabs(r_target_encoder-right_sensor())<start_big_counter_within) {
		i++;
		//printf("\nI: %i", i/10);

		if (i>big_timeout/10) {
			printf("\nI DRIVE BROKEN");
			return false;
		}
	}
	else {
		i = 0;
	}

	if (right_velocity()==0 && left_velocity()==0) {
		k+=delay_time;
		//printf("\nI: %i", i/10);

		if (k>velocity_timeout) {
			printf("\nI DRIVE BROKEN");
			return false;
		}
	}
	else {
		k = 0;
	}

	return true;
}

bool
turn_exit_condition(int small_timeout, int start_small_counter_within, int big_timeout, int start_big_counter_within, int velocity_timeout) {
	static int i, j, k;
	static int delay_time = 10;

	// If the robot gets within the target, make sure it's there for small_timeout amount of time
	if (fabs(gyro_target-get_gyro())<start_small_counter_within) {
		j++;
		//printf("\nJ: %i", j/10);

		if (j>small_timeout/10) {
			printf("\nJ TURN BROKEN");
			return false;
		}
	}
	else {
		j = 0;
	}
	// If the robot is close to the target, start a timer.  If the robot doesn't get closer within
	// a certain amount of time, exit and continue.
	if (fabs(gyro_target-get_gyro())<start_big_counter_within) {
		i++;
		//printf("\nI: %i", i/10);

		if (i>big_timeout/10) {
			printf("\nI TURN BROKEN");
			return false;
		}
	}
	else {
		i = 0;
	}

	if (right_velocity()==0 && left_velocity()==0) {
		k++;
		//printf("\nI: %i", i/10);

		if (k>velocity_timeout/10) {
			printf("\nI DRIVE BROKEN");
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
wait_drive(bool goal_yes, int delay_after) {
	int delay_time = 10;

	pros::delay(delay_time);
	// Parameters for exit condition function:
	// #1 - time the robot has to be within #2 of target
	// #2 - threshold for timer to start
	// #3 - time for if position is never reached
	// #4 - position for robot to be within to never reach target
	// #5 - velocity timeout
	if (active_drive_type==drive) {
		while (drive_exit_condition(80, 50, 300, 150, 500)) {
			pros::delay(delay_time);
		}
	}
	else if (active_drive_type==turn) {
		while (turn_exit_condition(100, 3, 500, 7, 500)) {
			pros::delay(delay_time);
		}
	}
	else if (active_drive_type==l_swing || active_drive_type==r_swing) {
		while (turn_exit_condition(100, 3, 500, 7, 500)) {
			pros::delay(delay_time);
		}
		//while (turn_exit_condition(100, 3, 500, 7) && fabs(lf.get_power()-rf.get_power())<20) {
		//	pros::delay(delay_time);
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
			if (sgn(l_error)!=l_sgn && sgn(r_error)!=r_sgn)
				run = false;

			pros::delay(10);
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
			if (sgn(g_error)!=g_sgn)
				run = false;

			pros::delay(10);
		}
	}
}
