#include <math.h>

#ifndef PATHFINDER_MATH_UTIL_H_DEF
#define PATHFINDER_MATH_UTIL_H_DEF

#ifdef __cplusplus
extern "C"
{
#endif

#include "okapi/pathfinder/include/pathfinder/lib.h"

#define PI 3.14159265358979323846
#define TAU PI*2

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

CAPI double bound_radians(double angle);

CAPI double r2d(double angleInRads);

CAPI double d2r(double angleInDegrees);

#ifdef __cplusplus
}
#endif

#endif