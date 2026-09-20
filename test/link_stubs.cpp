// Definitions for symbols declared in headers whose implementation file is
// excluded from the test binary (see test/Makefile), but that a file we DO
// compile still calls. Without these, linking fails with an undefined
// reference.
//
// ez::screen_line_set/screen_line_clear/screen_lines_clear are declared in
// display.hpp and defined in display.cpp (excluded -- it's the screen code
// being replaced, per repo rules). util.cpp's screen_print() and
// auton_selector.cpp's selected_auton_print() both call them. They write to
// the fake screen in fake_screen.hpp, so a test can read back what was printed.
#include <string>

#include "EZ-Template/api.hpp"
#include "fake_screen.hpp"

namespace ez {
// Like the real ones, lines outside the screen are ignored.
void screen_line_set(int line, std::string text) {
  if (line < 0 || line >= test_stub::SCREEN_LINE_COUNT) return;
  test_stub::screen_lines()[line] = text;
}
void screen_line_clear(int line) { screen_line_set(line, ""); }
void screen_lines_clear() { test_stub::screen_reset(); }

namespace as {
// Declared in sdcard.hpp, defined in sdcard.cpp (excluded). Called from
// Drive::drive_defaults_set() with (nullptr, nullptr), which the real
// implementation just turns into an early return, so a no-op is equivalent.
void limit_switch_lcd_initialize(pros::adi::DigitalIn* right_limit, pros::adi::DigitalIn* left_limit) {}

// Declared in sdcard.hpp, defined in sdcard.cpp (excluded). autons_add()
// reads it to keep auton_count in sync with blank pages the screen task
// has already registered.
int amount_of_blank_pages = 0;
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
