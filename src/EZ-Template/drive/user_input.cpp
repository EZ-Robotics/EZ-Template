/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"


// Set curve defaults
void Drive::set_curve_default(int left, int right)
{
  left_curve_scale = left;
  right_curve_scale = right;
}

// Initialize curve SD card
void Drive::init_curve_sd() {
  // If no SD card, return
  if (!ez::util::IS_SD_CARD) return;

  // Set Starting Curve to SD Card
  // Left Curve
  FILE* l_usd_file_read = fopen("/usd/left_curve.txt", "r");
  char l_buf[5];
  fread(l_buf, 1, 5, l_usd_file_read);
  left_curve_scale = std::stof(l_buf);
  fclose(l_usd_file_read);

  // Right Curve
  FILE* r_usd_file_read = fopen("/usd/right_curve.txt", "r");
  char r_buf[5];
  fread(r_buf, 1, 5, r_usd_file_read);
  right_curve_scale = std::stof(r_buf);
  fclose(r_usd_file_read);
}

// Save new left curve to SD card
void Drive::save_l_curve_sd() {
  // If no SD card, return
  if (!ez::util::IS_SD_CARD) return;

  FILE* usd_file_write = fopen("/usd/left_curve.txt", "w");
  std::string in_str = std::to_string(left_curve_scale);
  char const *in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}

// Save new right curve to SD card
void Drive::save_r_curve_sd() {
  // If no SD card, return
  if (!ez::util::IS_SD_CARD) return;

  FILE* usd_file_write = fopen("/usd/right_curve.txt", "w");
  std::string in_str = std::to_string(right_curve_scale);
  char const *in_c = in_str.c_str();
  fputs(in_c, usd_file_write);
  fclose(usd_file_write);
}

void Drive::left_curve_modify_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
  l_increase_.button = increase;
  l_decrease_.button = decrease;
}
void Drive::right_curve_modify_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
  r_increase_.button = increase;
  r_decrease_.button = decrease;
}

// Increase / decrease left and right curves
void Drive::l_increase() {
  left_curve_scale += 0.1;
}
 void Drive::l_decrease() {
  left_curve_scale -= 0.1;
  left_curve_scale =  left_curve_scale<0 ? 0 : left_curve_scale;
}
void Drive::r_increase() {
  right_curve_scale += 0.1;
}
void Drive::r_decrease() {
  right_curve_scale -= 0.1;
  right_curve_scale =  right_curve_scale<0 ? 0 : right_curve_scale;
}

// Button press logic for increase/decrease curves
void Drive::button_press(button_ *input_name, int button, std::function<void()> changeCurve, std::function<void()> save) {
  // If button is pressed, increase the curve and set toggles.
  if (button && !input_name->lock) {
    changeCurve();
    input_name->lock = true;
    input_name->release_reset = true;
  }

  // If the button is still held, check if it's held for 500ms.
  // Then, increase the curve every 100ms by 0.1
  else if (button && input_name->lock) {
    input_name->hold_timer+=ez::util::DELAY_TIME;
    if (input_name->hold_timer > 500.0) {
      input_name->increase_timer+=ez::util::DELAY_TIME;
      if (input_name->increase_timer > 100.0) {
        changeCurve();
        input_name->increase_timer = 0;
      }
    }
  }

  // When button is released for 250ms, save the new curve value to the SD card
  else if (!button) {
    input_name->lock = false;
    input_name->hold_timer = 0;

    if (input_name->release_reset) {
      input_name->release_timer+=ez::util::DELAY_TIME;
      if (input_name->release_timer > 250.0) {
        save();
        input_name->release_timer = 0;
        input_name->release_reset = false;
      }
    }
  }
}

// Toggle modifying curves with controller
void Drive::toggle_controller_curve_modifier(bool toggle) {
  disable_controller = toggle;
}

// Modify curves with button presses and display them to contrller
void Drive::modify_curve_with_controller() {
  if (!disable_controller) return; // True enables, false disables.
/*
  button_press(&l_increase_, master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT), ([this]{ this->l_increase(); }), ([this]{ this->save_l_curve_sd(); }));
  button_press(&l_decrease_, master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT), ([this]{ this->l_decrease(); }), ([this]{ this->save_l_curve_sd(); }));
  if (!is_tank) {
    button_press(&r_increase_, master.get_digital(pros::E_CONTROLLER_DIGITAL_A), ([this]{ this->r_increase(); }), ([this]{ this->save_r_curve_sd(); }));
    button_press(&r_decrease_, master.get_digital(pros::E_CONTROLLER_DIGITAL_Y), ([this]{ this->r_decrease(); }), ([this]{ this->save_r_curve_sd(); }));
  }
*/
  button_press(&l_increase_, master.get_digital(l_increase_.button), ([this]{ this->l_increase(); }), ([this]{ this->save_l_curve_sd(); }));
  button_press(&l_decrease_, master.get_digital(l_decrease_.button), ([this]{ this->l_decrease(); }), ([this]{ this->save_l_curve_sd(); }));
  if (!is_tank) {
    button_press(&r_increase_, master.get_digital(r_increase_.button), ([this]{ this->r_increase(); }), ([this]{ this->save_r_curve_sd(); }));
    button_press(&r_decrease_, master.get_digital(r_decrease_.button), ([this]{ this->r_decrease(); }), ([this]{ this->save_r_curve_sd(); }));
  }

  auto sr = std::to_string(right_curve_scale);
  auto sl = std::to_string(left_curve_scale);
  if (!is_tank)
    master.set_text(2, 0, sl+"   "+sr);
  else
    master.set_text(2, 0, sl);
}

// Left curve function
double Drive::left_curve_function(double x) {
  if (left_curve_scale != 0) {
    //if (CURVE_TYPE)
      return (powf(2.718, -(left_curve_scale/10)) + powf(2.718, (fabs(x)-127)/10) * (1-powf(2.718, -(left_curve_scale/10))))*x;
    //else
      //return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

// Right curve fnuction
double Drive::right_curve_function(double x) {
  if (right_curve_scale != 0) {
    //if (CURVE_TYPE)
      return (powf(2.718, -(right_curve_scale/10)) + powf(2.718, (fabs(x)-127)/10) * (1-powf(2.718, -(right_curve_scale/10))))*x;
    //else
      //return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

// Set active brake constant
void Drive::set_active_brake(double kp) {
  active_brake_kp = kp;
}

// Tank control
void Drive::chassis_tank()
{
  is_tank = true;

  // Toggle for controller curve
  modify_curve_with_controller();

  // Put the joysticks through the curve function
  int l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
  int r_stick = left_curve_function(master.get_analog(ANALOG_RIGHT_Y));

  // Threshold if joysticks don't come back to perfect 0
  if (abs(l_stick)>5 || abs(r_stick)>5) {
    set_tank(l_stick, r_stick);
    reset_drive_sensor();
  }
  // When joys are released, run active brake (P) on drive
  else {
    set_tank((0-left_sensor())*active_brake_kp, (0-right_sensor())*active_brake_kp);
  }
}

// Arcade standard
void Drive::chassis_arcade_standard(e_type t) {
  is_tank = false;

  // Toggle for controller curve
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
  if (abs(l_stick)>5 || abs(r_stick)>5) {
    set_tank(l_stick+r_stick, l_stick-r_stick);
    reset_drive_sensor();
  }
  // When joys are released, run active brake (P) on drive
  else {
    set_tank((0-left_sensor())*active_brake_kp, (0-right_sensor())*active_brake_kp);
  }
}

// Arcade control flipped
void Drive::chassis_arcade_flipped(e_type t) {
  is_tank = false;

  // Toggle for controller curve
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
  if (abs(l_stick)>5 || abs(r_stick)>5) {
    set_tank(r_stick+l_stick, r_stick-l_stick);
    reset_drive_sensor();
  }
  // When joys are released, run active brake (P) on drive
  else {
    set_tank((0-left_sensor())*active_brake_kp, (0-right_sensor())*active_brake_kp);
  }
}
