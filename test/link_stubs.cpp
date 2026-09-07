// Definitions for symbols declared in headers whose implementation file is
// excluded from the test binary (see test/Makefile), but that a file we DO
// compile still calls. Without these, linking fails with an undefined
// reference.
//
// ez::screen_line_set/screen_line_clear/screen_lines_clear are declared in
// display.hpp and defined in display.cpp (excluded -- it's the screen code
// being replaced, per repo rules). util.cpp's screen_print() and
// auton_selector.cpp's selected_auton_print() both call them.
#include <string>

#include "EZ-Template/api.hpp"

namespace ez {
void screen_line_set(int line, std::string text) {}
void screen_line_clear(int line) {}
void screen_lines_clear() {}

namespace as {
// Declared in sdcard.hpp, defined in sdcard.cpp (excluded). Called from
// Drive::drive_defaults_set() with (nullptr, nullptr), which the real
// implementation just turns into an early return, so a no-op is equivalent.
void limit_switch_lcd_initialize(pros::adi::DigitalIn* right_limit, pros::adi::DigitalIn* left_limit) {}
}  // namespace as
}  // namespace ez

// Every Drive constructor calls Drive::drive_defaults_set(), which reaches
// into these ten setters -- six declared in drive.hpp and defined in
// pid_tuner.cpp, four defined in user_input.cpp (both excluded, see
// test/Makefile). Real bodies just assign a private member or touch the SD
// card / controller display, none of which any test needs, so these are
// no-ops rather than copies of the real assignments.
void ez::Drive::opcontrol_curve_sd_initialize() {}
void ez::Drive::opcontrol_curve_buttons_toggle(bool toggle) {}
void ez::Drive::opcontrol_curve_buttons_left_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {}
void ez::Drive::opcontrol_curve_buttons_right_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {}
void ez::Drive::pid_tuner_button_increment_set(pros::controller_digital_e_t increase) {}
void ez::Drive::pid_tuner_button_decrement_set(pros::controller_digital_e_t decrease) {}
void ez::Drive::pid_tuner_button_up_set(pros::controller_digital_e_t pageUp) {}
void ez::Drive::pid_tuner_button_down_set(pros::controller_digital_e_t pageDown) {}
void ez::Drive::pid_tuner_button_left_set(pros::controller_digital_e_t pageLeft) {}
void ez::Drive::pid_tuner_button_right_set(pros::controller_digital_e_t pageRight) {}
