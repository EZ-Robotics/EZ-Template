/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/piston_group.hpp"

// Constructor for one piston
PistonGroup::PistonGroup(std::vector<int> input_ports, bool default_state) {
  for (auto i : input_ports) {
    pros::ADIDigitalOut temp(i, default_state);
    pistons.push_back(temp);
  }
  reversed = default_state;
}

// Constructor for one piston in expander
PistonGroup::PistonGroup(std::vector<int> input_ports, int expander_smart_port, bool default_state) {
  for (auto i : input_ports) {
    pros::ADIDigitalOut temp({expander_smart_port, i}, default_state);
    pistons.push_back(temp);
  }
  reversed = default_state;
}

// Set piston
void PistonGroup::set(bool input) {
  for (auto i : pistons) {
    i.set_value(reversed ? !input : input);
    current = input;
  }
}

// Get the current state
bool PistonGroup::get() { return current; }

// Toggle for user control
void PistonGroup::button_toggle(bool toggle) {
  if (toggle && !last_press) {
    set(!get());
  }
  last_press = toggle;
}

// Two button control for piston
void PistonGroup::buttons(bool active, bool deactive) {
  if (active && !get())
    set(true);
  else if (deactive && get())
    set(false);
}
