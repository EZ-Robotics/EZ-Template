#pragma once
#include <iostream>
#include <functional>
class Auton
{
public:
  Auton(std::string, std::function<void()>);
  std::string Name;
  void CallAuton();
private:
  std::function<void()> AutonCall;
};

Auton::Auton(std::string name, std::function<void()> callback)
{
  Name = name;
  AutonCall = callback;
}

void Auton::CallAuton()
{
  AutonCall();
}
