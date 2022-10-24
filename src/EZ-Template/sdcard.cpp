/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <filesystem>

#include "main.h"

namespace ez::as {
AutonSelector auton_selector{};

void auton_sd_update() {
  // If no SD card, return
  if (!ez::util::SD_CARD_ACTIVE) return;

  FILE* usd_file_write = fopen("/usd/auto.txt", "w");
  std::string cp_str = std::to_string(auton_selector.auton_page_current);
  char const* cp_c = cp_str.c_str();
  fputs(cp_c, usd_file_write);
  fclose(usd_file_write);
}

void auton_selector_init() {
  // If no SD card, return
  if (!ez::util::SD_CARD_ACTIVE) return;

  FILE* as_usd_file_read;
  // If file exists...
  if ((as_usd_file_read = fopen("/usd/auto.txt", "r"))) {
    char l_buf[5];
    fread(l_buf, 1, 5, as_usd_file_read);
    ez::as::auton_selector.auton_page_current = std::stof(l_buf);
    fclose(as_usd_file_read);
  }
  // If file doesn't exist, create file
  else {
    auton_sd_update();  // Writing to a file that doesn't exist creates the file
    printf("Created auto.txt\n");
  }

  if (ez::as::auton_selector.auton_page_current > ez::as::auton_selector.auton_count - 1 || ez::as::auton_selector.auton_page_current < 0) {
    ez::as::auton_selector.auton_page_current = 0;
    ez::as::auton_sd_update();
  }
}

void page_up() {
  if (auton_selector.auton_page_current == auton_selector.auton_count - 1)
    auton_selector.auton_page_current = 0;
  else
    auton_selector.auton_page_current++;
  auton_sd_update();
  auton_selector.selected_auton_print();
}

void page_down() {
  if (auton_selector.auton_page_current == 0)
    auton_selector.auton_page_current = auton_selector.auton_count - 1;
  else
    auton_selector.auton_page_current--;
  auton_sd_update();
  auton_selector.selected_auton_print();
}

void initialize() {
  // Initialize auto selector and LLEMU
  pros::lcd::initialize();
  ez::as::auton_selector_init();

  // Callbacks for auto selector
  ez::as::auton_selector.selected_auton_print();
  pros::lcd::register_btn0_cb(ez::as::page_down);
  pros::lcd::register_btn2_cb(ez::as::page_up);
}

void shutdown() {
  pros::lcd::shutdown();
}

bool turn_off = false;

// Using a button to control the lcd
pros::ADIDigitalIn* limit_switch_left = nullptr;
pros::ADIDigitalIn* limit_switch_right = nullptr;
pros::Task limit_switch_task(limit_switch_task);
void limit_switch_lcd_initialize(pros::ADIDigitalIn* right_limit, pros::ADIDigitalIn* left_limit) {
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

void limit_switch_task() {
  while (true) {
    if (limit_switch_right && limit_switch_right->get_new_press())
      page_up();
    else if (limit_switch_left && limit_switch_left->get_new_press())
      page_down();

    if (pros::millis() >= 500 && turn_off)
      limit_switch_task.suspend();

    pros::delay(50);
  }
}
}  // namespace ez::as
