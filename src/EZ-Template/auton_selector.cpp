/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"

ez::AutonSelector::AutonSelector() {
  auton_count = 0;
  auton_page_current = 0;
  last_auton_page_current = 0;
  Autons = {};
}

ez::AutonSelector::AutonSelector(std::vector<Auton> autons) {
  auton_count = autons.size();
  auton_page_current = 0;
  last_auton_page_current = 0;
  Autons = {};
  Autons.assign(autons.begin(), autons.end());
}

void ez::AutonSelector::selected_auton_print() {
  if (auton_count == 0) return;
  for (int i = 0; i < 8; i++)
    ez::screen_line_clear(i);
  ez::screen_print("Page " + std::to_string(auton_page_current + 1) + "\n" + Autons[auton_page_current].Name);
}

void ez::AutonSelector::selected_auton_call() {
  // Blank pages count toward auton_count, so an empty list has to be checked directly
  if (Autons.empty()) return;
  int last_auton = (int)Autons.size() - 1;

  // Run the page that is selected.  Anything can set auton_page_current (a custom selector, a
  // potentiometer, code), but last_auton_page_current is only updated by the brain screen buttons.
  // A blank page has no auton, so there fall back to the last auton page the buttons landed on.
  int page = auton_page_current;
  if (page < 0 || page > last_auton) page = last_auton_page_current;
  if (page < 0) page = 0;
  if (page > last_auton) page = last_auton;
  Autons[page].auton_call();
}

void ez::AutonSelector::autons_add(std::vector<Auton> autons) {
  Autons.insert(Autons.end(), autons.begin(), autons.end());
  auton_count = Autons.size() + ez::as::amount_of_blank_pages;
  auton_page_current = 0;
}
