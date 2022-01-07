#include "pre_auto_selector.hpp"

namespace ez
{
  pros::ADIDigitalIn* left_limit_switch = nullptr;
  pros::ADIDigitalIn* right_limit_switch = nullptr;
  void limit_switch_lcd_initialize(int left_limit_port, int right_limit_port)
  {
    if(left_limit_port == -1 || right_limit_port == -1)
    {
      limit_switch_task.suspend();
      return;
    }
    left_limit_switch = new pros::ADIDigitalIn(left_limit_port);
    right_limit_switch = new pros::ADIDigitalIn(right_limit_port);
    limit_switch_task.resume();
  }

  void limitSwitchTask()
  {
    while(true)
    {
      if(left_limit_switch || right_limit_switch)
      {
        if(left_limit_switch->get_new_press())
        {
          ez::as::page_down();
        }
        else if(right_limit_switch->get_new_press())
        {
          ez::as::page_up();
        }
      }
      pros::delay(20);
    }
  }
  pros::Task limit_switch_task(limitSwitchTask, nullptr);
}
