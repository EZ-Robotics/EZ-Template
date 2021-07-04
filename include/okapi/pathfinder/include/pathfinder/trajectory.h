#ifndef PATHFINDER_TRAJECTORY_H_DEF
#define PATHFINDER_TRAJECTORY_H_DEF

#ifdef __cplusplus
extern "C"
{
#endif

#include "okapi/pathfinder/include/pathfinder/lib.h"
#include "okapi/pathfinder/include/pathfinder/structs.h"

CAPI int pathfinder_prepare(Waypoint *path, int path_length, void (*fit)(Waypoint,Waypoint,Spline*), int sample_count, double dt,
        double max_velocity, double max_acceleration, double max_jerk, TrajectoryCandidate *cand);
CAPI int pathfinder_generate(TrajectoryCandidate *c, Segment *segments);

CAPI void pf_trajectory_copy(Segment *src, Segment *dest, int length);

CAPI TrajectoryInfo pf_trajectory_prepare(TrajectoryConfig c);
CAPI int pf_trajectory_create(TrajectoryInfo info, TrajectoryConfig c, Segment *seg);
CAPI int pf_trajectory_fromSecondOrderFilter(int filter_1_l, int filter_2_l, 
        double dt, double u, double v, double impulse, int len, Segment *t);

#ifdef __cplusplus
}
#endif

#endif