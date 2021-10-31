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
