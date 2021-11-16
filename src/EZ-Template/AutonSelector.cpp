/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"


AutonSelector::AutonSelector() {
  AutonCount = 0;
  CurrentAutonPage = 0;
  Autons = {};
}

AutonSelector::AutonSelector(std::vector<Auton> autons) {
  AutonCount = autons.size();
  CurrentAutonPage = 0;
  Autons = {};
  Autons.assign(autons.begin(), autons.end());
}

void AutonSelector::PrintSelectedAuto() {
  if (AutonCount == 0) return;
  pros::lcd::clear_line(0);
  pros::lcd::clear_line(1);
  ez::print_to_screen("Page " + std::to_string(CurrentAutonPage+1) + "\n" + Autons[CurrentAutonPage].Name);
}

void AutonSelector::CallSelectedAuto() {
  if (AutonCount == 0) return;
  Autons[CurrentAutonPage].AutonCall();
}

void AutonSelector::AddAutons(std::vector<Auton> autons) {
  AutonCount += autons.size();
  CurrentAutonPage = 0;
  Autons.assign(autons.begin(), autons.end());
}
