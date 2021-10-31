#pragma once
#include <iostream>
#include <functional>
//hpp file
class Auton
{
public:
  Auton(std::string, std::function<void()>);
  std::string Name;
  void CallAuton();
private:
  std::function<void()> AutonCall;
};


//cpp file
Auton::Auton(std::string name, std::function<void()> callback)
{
  Name = name;
  AutonCall = callback;
}

void Auton::CallAuton()
{

  AutonCall();
}
