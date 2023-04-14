#ifndef PATHFINDER_MOD_SWERVE_H_DEF
#define PATHFINDER_MOD_SWERVE_H_DEF

#ifdef __cplusplus
extern "C"
{
#endif

#include "okapi/pathfinder/include/pathfinder/lib.h"
#include "okapi/pathfinder/include/pathfinder/structs.h"

CAPI typedef enum {
    SWERVE_DEFAULT
} SWERVE_MODE;

CAPI void pathfinder_modify_swerve(Segment *original, int length, Segment *front_left, Segment *front_right,
        Segment *back_left, Segment *back_right, double wheelbase_width, double wheelbase_depth, SWERVE_MODE mode);

#ifdef __cplusplus
}
#endif

#endif