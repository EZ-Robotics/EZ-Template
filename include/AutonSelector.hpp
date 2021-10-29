#pragma once
#include <list>
#include "Auton.hpp"
#include <tuple>

using namespace std;
class AutonSelector
{
public:
  std::list<Auton> Autons;
  int AutonIndex;
  AutonSelector();
  AutonSelector(std::list<tuple<string, std::function<void()>>>);

};
AutonSelector::AutonSelector()
{
  AutonIndex = 0;
  Autons = {};
}
AutonSelector::AutonSelector(std::list<tuple<string, std::function<void()>>> autons)
{
  AutonIndex = 0;
  //Autons();
  for(auto i : autons)
  {
    Auton temp(get<0>(i), get<1>(i));
    Autons.push_back(temp);
  }
}
