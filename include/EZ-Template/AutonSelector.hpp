/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
//#include <list>
#include "main.h"
#include "Auton.hpp"
#include <tuple>

using namespace std;
class AutonSelector
{
public:
  std::vector<Auton> Autons;
  int CurrentAutonPage;
  int AutonCount;
  AutonSelector();
  AutonSelector(std::vector<tuple<std::string, std::function<void()>>>);
  void CallSelectedAuto();
  void PrintSelectedAuto();
};
