#include "Auton.hpp"
Auton::Auton(std::string name, std::function<void()> callback)
{
  Name = name;
  AutonCall = callback;
}

void Auton::CallAuton()
{

  AutonCall();
}
