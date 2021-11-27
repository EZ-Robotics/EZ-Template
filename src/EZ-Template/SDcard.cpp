/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

namespace ez {
namespace as {
AutonSelector autoSelector{};

void update_auto_sd() {
  // If no SD card, return
  if (!ez::util::IS_SD_CARD) return;

  FILE* usd_file_write = fopen("/usd/auto.txt", "w");
  std::string cp_str = std::to_string(autoSelector.current_auton_page);
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
  ez::as::autoSelector.current_auton_page = std::stof(l_buf);
  fclose(as_usd_file_read);

  if (ez::as::autoSelector.current_auton_page > ez::as::autoSelector.auton_count - 1 || ez::as::autoSelector.current_auton_page < 0) {
    ez::as::autoSelector.current_auton_page = 0;
    ez::as::update_auto_sd();
  }
}

void page_up() {
  if (autoSelector.current_auton_page == autoSelector.auton_count - 1)
    autoSelector.current_auton_page = 0;
  else
    autoSelector.current_auton_page++;
  update_auto_sd();
  autoSelector.print_selected_auton();
}

void page_down() {
  if (autoSelector.current_auton_page == 0)
    autoSelector.current_auton_page = autoSelector.auton_count - 1;
  else
    autoSelector.current_auton_page--;
  update_auto_sd();
  autoSelector.print_selected_auton();
}
}  // namespace as
}  // namespace ez
