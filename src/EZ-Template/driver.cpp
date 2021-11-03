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



// Arcade control standard
