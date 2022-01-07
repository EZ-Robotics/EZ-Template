#pragma once
#include "main.h"

namespace ez
{
extern  pros::ADIDigitalIn* left_limit_switch;
extern pros::ADIDigitalIn* right_limit_switch;
/**
* Initialize two limitswithces to change pages on the lcd
*
* @param left_limit_port
*        port for the left limit switch
* @param right_limit_port
*        port for the right limit switch
*/
void limit_switch_lcd_initialize(int left_limit_port, int right_limit_port);

/**
* pre_auto_task
*/
extern pros::Task limit_switch_task;
}
