/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"

AutonSelector::AutonSelector() {
  auton_count = 0;
  auton_page_current = 0;
  Autons = {};
}

AutonSelector::AutonSelector(std::vector<Auton> autons) {
  auton_count = autons.size();
  auton_page_current = 0;
  Autons = {};
  Autons.assign(autons.begin(), autons.end());
}

void AutonSelector::selected_auton_print() {
  if (auton_count == 0) return;
  for (int i = 0; i < 8; i++)
    pros::lcd::clear_line(i);
  ez::screen_print("Page " + std::to_string(auton_page_current + 1) + "\n" + Autons[auton_page_current].Name);
}

void AutonSelector::selected_auton_call() {
  if (auton_count == 0) return;
  Autons[last_auton_page_current].auton_call();
}

void AutonSelector::autons_add(std::vector<Auton> autons) {
  auton_count += autons.size();
  auton_page_current = 0;
  Autons.assign(autons.begin(), autons.end());
}
