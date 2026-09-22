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
#include "screen_capture.hpp"

namespace ez {
namespace test {
std::string screen_lines[8];
}  // namespace test

void screen_line_set(int line, std::string text) {
  if (line < 0 || line >= 8) return;
  test::screen_lines[line] = text;
}
void screen_line_clear(int line) {
  if (line < 0 || line >= 8) return;
  test::screen_lines[line].clear();
}
void screen_lines_clear() {
  for (auto& line : test::screen_lines) line.clear();
}

namespace as {
// Declared in sdcard.hpp, defined in sdcard.cpp (excluded). Drive::drive_defaults_set()
// reads the two pointers and, when neither switch is set up, calls
// limit_switch_lcd_initialize(nullptr, nullptr). The real one disables the limit
// switches and no test looks at that, so a no-op is equivalent.
pros::adi::DigitalIn* limit_switch_left = nullptr;
pros::adi::DigitalIn* limit_switch_right = nullptr;
void limit_switch_lcd_initialize(pros::adi::DigitalIn* right_limit, pros::adi::DigitalIn* left_limit) {}

// Declared in sdcard.hpp, defined in sdcard.cpp (excluded). autons_add()
// reads it to keep auton_count in sync with blank pages the screen task
// has already registered.
int amount_of_blank_pages = 0;
}  // namespace as
}  // namespace ez

// Every Drive constructor calls Drive::drive_defaults_set(), which reaches
// into these six setters, declared in drive.hpp and defined in pid_tuner.cpp
// (excluded, see test/Makefile). Real bodies just assign a private member,
// which no test needs, so these are no-ops rather than copies of the real
// assignments.
void ez::Drive::pid_tuner_button_increment_set(pros::controller_digital_e_t increase) {}
void ez::Drive::pid_tuner_button_decrement_set(pros::controller_digital_e_t decrease) {}
void ez::Drive::pid_tuner_button_up_set(pros::controller_digital_e_t pageUp) {}
void ez::Drive::pid_tuner_button_down_set(pros::controller_digital_e_t pageDown) {}
void ez::Drive::pid_tuner_button_left_set(pros::controller_digital_e_t pageLeft) {}
void ez::Drive::pid_tuner_button_right_set(pros::controller_digital_e_t pageRight) {}
