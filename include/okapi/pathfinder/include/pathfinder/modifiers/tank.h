#ifndef PATHFINDER_MOD_TANK_H_DEF
#define PATHFINDER_MOD_TANK_H_DEF

#ifdef __cplusplus
extern "C"
{
#endif

#include "okapi/pathfinder/include/pathfinder/lib.h"
#include "okapi/pathfinder/include/pathfinder/structs.h"

CAPI void pathfinder_modify_tank(Segment *original, int length, Segment *left, Segment *right, double wheelbase_width);

#ifdef __cplusplus
}
#endif

#endif