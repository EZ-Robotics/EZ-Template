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
