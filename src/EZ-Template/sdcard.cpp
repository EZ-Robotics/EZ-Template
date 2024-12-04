/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "sdcard.hpp"

#include <filesystem>

#include "auton_selector.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "util.hpp"

namespace ez::as {
AutonSelector auton_selector{};

void auto_sd_update() {
  // If no SD card, return
  if (!ez::util::SD_CARD_ACTIVE) return;

  FILE* usd_file_write = fopen("/usd/auto.txt", "w");
  std::string cp_str = std::to_string(auton_selector.auton_page_current);
  char const* cp_c = cp_str.c_str();
  fputs(cp_c, usd_file_write);
  fclose(usd_file_write);
}

void auton_selector_initialize() {
  // If no SD card, return
  if (!ez::util::SD_CARD_ACTIVE) return;

  FILE* as_usd_file_read;
  // If file exists...
  if ((as_usd_file_read = fopen("/usd/auto.txt", "r"))) {
    char a_buf[10];
    fread(a_buf, 1, 10, as_usd_file_read);
    ez::as::auton_selector.auton_page_current = std::stof(a_buf);
    fclose(as_usd_file_read);
  }
  // If file doesn't exist, create file
  else {
    auto_sd_update();  // Writing to a file that doesn't exist creates the file
    printf("Created auto.txt\n");
  }

  if (ez::as::auton_selector.auton_page_current > ez::as::auton_selector.auton_count - 1 || ez::as::auton_selector.auton_page_current < 0) {
    ez::as::auton_selector.auton_page_current = 0;
    ez::as::auto_sd_update();
  }
}

void print_page() {
  if (((auton_selector.auton_count - 1 - amount_of_blank_pages) - auton_selector.auton_page_current) >= 0) {
    auto_sd_update();
    auton_selector.selected_auton_print();
  } else {
    for (int i = 0; i < 8; i++)
      pros::lcd::clear_line(i);
    screen_print("Page " + std::to_string(auton_selector.auton_page_current + 1) + " - Blank page " + std::to_string(page_blank_current() + 1));
  }
  if (page_blank_current() < 0)
    auton_selector.last_auton_page_current = auton_selector.auton_page_current;
}

void page_up() {
  if (util::sgn(page_blank_current()) == -1) auton_selector.last_auton_page_current = auton_selector.auton_page_current;
  if (auton_selector.auton_page_current == auton_selector.auton_count - 1)
    auton_selector.auton_page_current = 0;
  else
    auton_selector.auton_page_current++;
  print_page();
}

void page_down() {
  if (util::sgn(page_blank_current()) == -1) auton_selector.last_auton_page_current = auton_selector.auton_page_current;
  if (auton_selector.auton_page_current == 0)
    auton_selector.auton_page_current = auton_selector.auton_count - 1;
  else
    auton_selector.auton_page_current--;
  print_page();
}

int page_blank_current() {
  return (auton_selector.auton_count - amount_of_blank_pages - auton_selector.auton_page_current) * -1;
}

int amount_of_blank_pages = 0;
bool page_blank_is_on(int page) {
  if (page + 1 > amount_of_blank_pages) {
    auton_selector.auton_count -= amount_of_blank_pages;
    amount_of_blank_pages = page + 1;
    auton_selector.auton_count += amount_of_blank_pages;
  }
  if (page_blank_current() == page)
    return true;
  return false;
}

void page_blank_remove(int page) {
  if (amount_of_blank_pages >= page + 1) {
    auton_selector.auton_count -= amount_of_blank_pages;
    amount_of_blank_pages -= page + 1;
    auton_selector.auton_count += amount_of_blank_pages;
  }
  auton_selector.auton_page_current = auton_selector.last_auton_page_current;
  print_page();
}

void page_blank_remove_all() {
  page_blank_remove(amount_of_blank_pages - 1);
}

int page_blank_amount() {
  return amount_of_blank_pages;
}

void initialize() {
  // Initialize auto selector and LLEMU
  pros::lcd::initialize();
  ez::as::auton_selector_initialize();

  // Callbacks for auto selector
  print_page();
  pros::lcd::register_btn0_cb(ez::as::page_down);
  pros::lcd::register_btn2_cb(ez::as::page_up);

  auton_selector_running = true;
}

void shutdown() {
  pros::lcd::shutdown();
  pros::lcd::register_btn0_cb(nullptr);
  pros::lcd::register_btn2_cb(nullptr);

  auton_selector_running = false;
}

bool enabled() { return auton_selector_running; }

bool turn_off = false;

// Using a button to control the lcd
pros::adi::DigitalIn* limit_switch_left = nullptr;
pros::adi::DigitalIn* limit_switch_right = nullptr;
pros::Task limit_switch_task(ez::as::limitSwitchTask);
void limit_switch_lcd_initialize(pros::adi::DigitalIn* right_limit, pros::adi::DigitalIn* left_limit) {
  if (!left_limit && !right_limit) {
    delete limit_switch_left;
    delete limit_switch_right;
    if (pros::millis() <= 100)
      turn_off = true;
    return;
  }
  turn_off = false;
  limit_switch_right = right_limit;
  limit_switch_left = left_limit;
  limit_switch_task.resume();
}

void limitSwitchTask() {
  while (true) {
    if (limit_switch_right && limit_switch_right->get_new_press())
      ez::as::page_up();
    else if (limit_switch_left && limit_switch_left->get_new_press())
      ez::as::page_down();

    if (pros::millis() >= 500 && turn_off)
      limit_switch_task.suspend();

    pros::delay(50);
  }
}
}  // namespace ez::as