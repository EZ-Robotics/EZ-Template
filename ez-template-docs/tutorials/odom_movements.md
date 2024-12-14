---
layout: default
title: Odom Movements
description: make robor go to coordinates by itself
---




## Moving to a Point
`pid_odom_set()` has many ways of calling it with many default values you can configure and override in specific motions.

### Moving to Point
At minimum, you need to give the robot a point to go to, a direction to get there, and a speed limit (0-127).  You can do this with and without okapi units.  
```cpp
// Drive forward to (0, 36) forward
chassis.pid_odom_set({{0_in, 36_in}, fwd, 110});
chassis.pid_wait();

// Drive back to (0, 0) forward
chassis.pid_odom_set({{0_in, 0_in}, fwd, 110});
chassis.pid_wait();

// Drive forward to (0, 36) backward
chassis.pid_odom_set({{0, 36}, rev, 110});
chassis.pid_wait();

// Drive back to (0, 0) backward
chassis.pid_odom_set({{0, 0}, rev, 110});
chassis.pid_wait();
```

### Moving to Pose
You can specify an ending angle for the robot to reach.  If no angle is specified, the robot will try to get there in the fastest way possible.  

Points that have angle specified use an algorithm called Boomerang, and they will be a little slower.  
```cpp
// Drive forward to (0, 36) forward, end at 45 degrees
chassis.pid_odom_set({{0_in, 36_in, 45_deg}, fwd, 110});
chassis.pid_wait();

// Drive back to (0, 0) backward, end at 0 degrees
chassis.pid_odom_set({{0, 0, 0}, rev, 110});
chassis.pid_wait();
```

### Turn Paths
You can set the robot to always turn a specific direction.  
```cpp
chassis.pid_odom_behavior_set(ez::shortest);

// This go to (0, 24) ending at 45 deg, the fastest way
chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, 110});
chassis.pid_wait();
```

But you can override these defaults in each motion.  
```cpp
chassis.pid_odom_behavior_set(ez::shortest);

// This go to (0, 24) ending at 45 deg, the longest way
chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, 110, ez::longest});
chassis.pid_wait();
```

You can also pick between turning clockwise and counterclockwise.  
```cpp
chassis.pid_odom_behavior_set(ez::shortest);

// This go to (24, 24) but it'll turn clockwise
chassis.pid_odom_set({{24_in, 24_in}, fwd, 110, ez::cw});
chassis.pid_wait();

chassis.pid_turn_set(0_deg, 90);
chassis.pid_wait();

// This go to (48, 48) but it'll turn counter clockwise
chassis.pid_odom_set({{48_in, 48_in}, fwd, 110, ez::ccw});
chassis.pid_wait();
```

### Slew
Slew ramps up the speed limit of the robot to give smoother accelerations.  
```cpp
chassis.slew_drive_set(true);  // Enables global slew
chassis.slew_drive_constants_set(5_in, 50);

// Over the first 5 inches, the robot will ramp the speed limit from 50 to 110 d
chassis.pid_odom_set({{0_in, 36_in}, fwd, 110});
chassis.pid_wait();
```

You can change the behavior of slew for each motion.  In the example below, slew is disabled globally but is enabled in one of the motions.   
```cpp
chassis.slew_drive_set(false);  // Disables global slew
chassis.slew_drive_constants_set(5_in, 50);

// Over the first 5 inches, the robot will ramp the speed limit from 50 to 110 
chassis.pid_odom_set({{0_in, 36_in}, fwd, 110}, true);  // Slew will be enabled for this motion
chassis.pid_wait();

// This will not use slew to travel backwards to (0, 0)
chassis.pid_odom_set({{0_in, 0_in}, rev, 110});
chassis.pid_wait();
```


### Turn Paths and Slew
You can use slew in conjunction with different turn behavior. 
```cpp
chassis.pid_odom_behavior_set(ez::shortest);
chassis.slew_drive_set(false);  // Enables global slew
chassis.slew_drive_constants_set(5_in, 50);


// This will go to (0, 24) ending at 45 deg, the longest way, with slew
chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, 110, ez::longest}, true);
chassis.pid_wait();

// This will go to (0, 0) the shortest way, without slew
chassis.pid_odom_set({{0_in, 0_in}, rev, 110);
chassis.pid_wait();
```


## Multiple Points
You can string multiple points together and the robot will go through all of them.  
```cpp
// Drive to (0, 30) and pass through (6, 10) and (0, 20) on the way, with slew
chassis.pid_odom_set({{{6_in, 10_in}, fwd, 110},
                      {{0_in, 20_in}, fwd, 110},
                      {{0_in, 30_in}, fwd, 110}},
                      true);
chassis.pid_wait();
```

```cpp
// Drive to (0, 24) ending at 45deg, then go to (24, 24)
chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, 110},
                      {{24_in, 24_in}, fwd, 110}},
                      true);
chassis.pid_wait();
```

Beyond this, all of the syntax for moving to single points applies to moving through multiple points.  Each point has the same syntax, and a final parameter for if slew is enabled or not.  


## "pid_drive_set()"
You can take all the `pid_drive_set()` syntax and replace it for `pid_odom_set()`.  This will run odometry but it'll be going forward/backwards only.  
```cpp
chassis.slew_drive_set(false);  // Disables global slew
chassis.slew_drive_constants_set(5_in, 50);

// Over the first 5 inches, the robot will ramp the speed limit from 50 to 110, with odometry
chassis.pid_odom_set(24_in, 110, true);  // Slew will be enabled for this motion
chassis.pid_wait();

// This will not use slew to travel backwards 24in, with odometry
chassis.pid_odom_set(-24_in, 110);  
chassis.pid_wait();
```