#pragma once
#include "main.h"
/**
 * Sets the chassis to controller joysticks, using tank control.
*/
void chassis_tank();

/**
 * Sets the chassis to controller joysticks, using standard arcade control.
 * \param t
 *        enum e_type, k_single or k_split control
*/
void chassis_arcade_standard(e_type t);
void set_tank(int l, int r);
/**
 * Sets the chassis to controller joysticks, using flipped arcade control.
 * \param t
 *        enum e_type, k_single or k_split control
*/
void chassis_arcade_flipped(e_type t);
float left_curve_function(int x);
float right_curve_function(int x);
