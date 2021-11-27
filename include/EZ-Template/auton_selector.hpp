/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include <tuple>

#include "EZ-Template/auton.hpp"

using namespace std;
class AutonSelector {
 public:
  std::vector<Auton> Autons;
  int current_auton_page;
  int auton_count;
  AutonSelector();
  AutonSelector(std::vector<Auton> autons);
  void call_selected_auton();
  void print_selected_auton();
  void add_autons(std::vector<Auton> autons);
};
