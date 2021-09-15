#ifndef PATHFINDER_STRUCT_H_DEF
#define PATHFINDER_STRUCT_H_DEF

#include "okapi/pathfinder/include/pathfinder/lib.h"

CAPI typedef struct {
    double x, y, angle;
} Waypoint;

CAPI typedef struct {
    double a, b, c, d, e;
    double x_offset, y_offset, angle_offset, knot_distance, arc_length;
} Spline;

CAPI typedef struct {
    double x, y;
} Coord;

CAPI typedef struct {
    double dt, x, y, position, velocity, acceleration, jerk, heading;
} Segment;

CAPI typedef struct {
    double dt, max_v, max_a, max_j, src_v, src_theta, dest_pos, dest_v, dest_theta;
    int sample_count;
} TrajectoryConfig;

CAPI typedef struct {
    int filter1, filter2, length;
    double dt, u, v, impulse;
} TrajectoryInfo;

CAPI typedef struct {
    Spline *saptr;
    double *laptr;
    double totalLength;
    int length;
    int path_length;
    TrajectoryInfo info;
    TrajectoryConfig config;
} TrajectoryCandidate;

#endif