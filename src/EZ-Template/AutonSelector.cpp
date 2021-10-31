#include "EZ-Template/AutonSelector.hpp"
AutonSelector::AutonSelector()
{
  AutonCount = 0;
  CurrentAutonPage = 0;
  Autons = {};
}

AutonSelector::AutonSelector(std::vector<tuple<std::string, std::function<void()>>> autons)
{
  AutonCount = autons.size();
  CurrentAutonPage = 0;
  //Autons();
  for(auto i : autons)
  {
    Auton temp(get<0>(i), get<1>(i));
    Autons.push_back(temp);
  }
}
void AutonSelector::PrintSelectedAuto()
{
  pros::lcd::clear_line(0);
  pros::lcd::set_text(0, "Page "+std::to_string(CurrentAutonPage+1));
  pros::lcd::set_text(1, Autons[CurrentAutonPage].Name);
}
void AutonSelector::CallSelectedAuto()
{
  Autons[AutonCount].CallAuton();
}
