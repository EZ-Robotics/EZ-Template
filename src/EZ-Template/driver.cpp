#include "EZ-Template/driver.hpp"
bool is_tank = false;
float LEFT_CURVE_SCALE  = 0;
float RIGHT_CURVE_SCALE = 0;
const int THRESH = 5; // Joystick threshold to trigger activebrake
///
// Input Curve Setup
//  -adds an input curve to the joysticks, here https://www.desmos.com/calculator/rcfjjg83zx
///
const bool  CURVE_TYPE  = true;  // true is red, false is blue in the demos link above
float right_curve_function(int x) {
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

set_tank(int l, int r) {
  set_left_chassis(l);
  set_right_chassis(r);
}

// Tank control
void chassis_tank() {
  is_tank = true;

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
