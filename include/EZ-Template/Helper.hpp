#pragma once

/**
 * Returns 1 if input is positive and -1 if input is negative
*/
static int sgn(double input);
/**
 * Returns true if the input is < 0
*/
static bool isReversed(double input);
/**
 * Returns input restricted to min-max threshold
*/
static double clip_num(double input, double min, double max);
/**
 * DriverType
*/
enum e_type{ k_single=0, k_split=1 };
