/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

namespace ez {
namespace as {
AutonSelector auton_selector{};

void update_auto_sd() {
  // If no SD card, return
  if (!ez::util::IS_SD_CARD) return;

  FILE* usd_file_write = fopen("/usd/auto.txt", "w");
  std::string cp_str = std::to_string(auton_selector.current_auton_page);
  char const* cp_c = cp_str.c_str();
  fputs(cp_c, usd_file_write);
  fclose(usd_file_write);
}

void init_auton_selector() {
  // If no SD card, return
  if (!ez::util::IS_SD_CARD) return;

  FILE* as_usd_file_read = fopen("/usd/auto.txt", "r");
  char l_buf[5];
  fread(l_buf, 1, 5, as_usd_file_read);
  ez::as::auton_selector.current_auton_page = std::stof(l_buf);
  fclose(as_usd_file_read);

  if (ez::as::auton_selector.current_auton_page > ez::as::auton_selector.auton_count - 1 || ez::as::auton_selector.current_auton_page < 0) {
    ez::as::auton_selector.current_auton_page = 0;
    ez::as::update_auto_sd();
  }
}

void page_up() {
  if (auton_selector.current_auton_page == auton_selector.auton_count - 1)
    auton_selector.current_auton_page = 0;
  else
    auton_selector.current_auton_page++;
  update_auto_sd();
  auton_selector.print_selected_auton();
}

void page_down() {
  if (auton_selector.current_auton_page == 0)
    auton_selector.current_auton_page = auton_selector.auton_count - 1;
  else
    auton_selector.current_auton_page--;
  update_auto_sd();
  auton_selector.print_selected_auton();
}

void initialize() {
  // Initialize auto selector and LLEMU
  pros::lcd::initialize();
  ez::as::init_auton_selector();

  // Callbacks for auto selector
  ez::as::auton_selector.print_selected_auton();
  pros::lcd::register_btn0_cb(ez::as::page_down);
  pros::lcd::register_btn2_cb(ez::as::page_up);
}

void shutdown() {
  pros::lcd::shutdown();
}
}  // namespace as
}  // namespace ez
