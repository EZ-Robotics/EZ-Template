/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/piston.hpp"

using namespace ez;

// Constructor for one piston
Piston::Piston(int input_port, bool default_state)
    : piston(input_port, default_state) {
  reversed = default_state;
}

// Constructor for one piston plugged into expander
Piston::Piston(int input_port, int expander_smart_port, bool default_state)
    : piston({expander_smart_port, input_port}, default_state) {
  reversed = default_state;
}

// Set piston
void Piston::set(bool input) {
  piston.set_value(reversed ? !input : input);
  current = input;
}

// Get the current state
bool Piston::get() { return current; }

// Toggle for user control
void Piston::button_toggle(int toggle) {
  if (toggle && !last_press) {
    set(!get());
  }
  last_press = toggle;
}

// Two button control for piston
void Piston::buttons(int active, int deactive) {
  if (active && !get())
    set(true);
  else if (deactive && get())
    set(false);
}