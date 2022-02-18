/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/piston_group.hpp"

// Constructor for multiple pistons
PistonGroup::PistonGroup(std::vector<int> input_ports, bool default_state) {
  for (auto i : input_ports) {
    Piston temp(i, default_state);
    pistons.push_back(temp);
  }
}

void PistonGroup::set(bool input) {
  for (auto i : pistons) {
    i.set(input);
  }
}

// Get the current state
bool PistonGroup::get() { return pistons[0].get(); }

// Toggle for user control
void PistonGroup::button_toggle(int toggle) {
  for (auto i : pistons) {
    i.button_toggle(toggle);
  }
}

// Two button control for piston
void PistonGroup::button(int active, int deactive) {
  for (auto i : pistons) {
    i.button(active, deactive);
  }
}