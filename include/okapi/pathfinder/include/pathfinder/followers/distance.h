#ifndef PATHFINDER_FOL_DISTANCE_H_DEF
#define PATHFINDER_FOL_DISTANCE_H_DEF

#ifdef __cplusplus
extern "C"
{
#endif

#include "okapi/pathfinder/include/pathfinder/lib.h"
#include "okapi/pathfinder/include/pathfinder/structs.h"

CAPI typedef struct {
    double kp, ki, kd, kv, ka;
} FollowerConfig;

CAPI typedef struct {
    double last_error, heading, output;
    int segment, finished;
} DistanceFollower;

CAPI double pathfinder_follow_distance(FollowerConfig c, DistanceFollower *follower, Segment *trajectory, int trajectory_length, double distance);

CAPI double pathfinder_follow_distance2(FollowerConfig c, DistanceFollower *follower, Segment segment, int trajectory_length, double distance);

#ifdef __cplusplus
}
#endif

#endif