---
layout: default
title: Odom Movements
description: make robor go to coordinates by itself
---

## How does it work?

### Coordinate System
By default the robot starts at (0, 0) and facing 0 degrees.  This means:  
- right is positive X
- forward is positive Y
- clockwise is positive Theta

<!---
IMAGE

diagram of coordinate system with bot in center, arrows showing +Y, +X, and a rotation for +T?
-->

You can change where you start by messing with this line of code in your autonomous routines.  
```cpp
chassis.odom_xyt_set(0_in, 0_in, 0_deg);
```

### Point to Point
This is the backbone of all EZ-Template odometry motions.  

<!---
IMAGE

this point to point section feels like it needs an image, honestly I'm not sure what, but anything to break up this absolute wall of text
-->

Two PID loops run simultaneously.  One for forward/backwards movements (we call this xyPID) and one for turning (angularPID).  The constants used for xyPID are your normal drive constants, in all of our testing we found that there was no need for odometry motions to have unique constants.  Unique constants for angularPID was found to be necessary, and you can use another unique set of constants for Boomerang motions (Boomerang is explained further below).  

A short explanation of how moving to a point works:
- angularPID is calculated using the arc tangent between the target point and our current point, this gives us an angle that we can use as our target angle  
- xyPID is calculated distance between our current position and the target point, this gives us the result of `target - current` in PID

EZ-Template's implementation is more complicated than this:
- angularPID
  - angle doesn't scale based on distance.  Imagine we're at (0.5, 11.5) trying to go to (0, 12).  The robot will try to face -45 degrees, but if during this the robot overshoots and goes to (-0.5, 12.5) the robot will want to turn to 135 degrees.
  - this causes extremely jittery motions that will never settle 
- xyPID
  - distance formula doesn't have a sign, it's an absolute value
  - PID needs to know when we've overshot our target so it can make the robot come backwards, and distance formula alone doesn't give us that
  - the robot will go forward towards the target, slow down as it reaches the target, and then continue forward forever

The solutions are:
- angularPID
  -  instead of doing this calculation on the target point, we create a new target point to face that is projected out from the target point
  -  if we're going to (12, 12) from (0, 0) we would try to face (20, 20) 
-  xyPID
   -  instead of the robot traveling from (0, 0) to (12, 12) and us figuring it out from that, we can translate and rotate everything and pretend the robot is traveling to (0, 0) from (0, distance formula)
   -  first we translate our target point to be (0, 0), then we rotate the robot around the origin so it's x value is 0
   -  we use the Y axis of the robot in this translation + rotation instead of distance formula 
   -  this tells us when we overshoot by giving Y values

EZ-Template also prioritizes turning over driving, this is nice because it forces the robot to face a direction while driving there.  Something unique about EZ-Template is the ability to tune how much priority turning takes over driving, which is extremely useful for robots that only have omni wheels.  

### Pure Pursuit
At its core, pure pursuit is just continuously calling Point to Point and is updating the target point.  

How does it decide the target point?  

The robot is following a "carrot on a stick", like in the gif below.  The new target is the next point along the path that the "carrot" intersects with.  This allows you to give the robot as many points as you'd like and the robot will follow through all of them.  
![](@site/static/img/pure_pursuit.gif)

### Boomerang
Boomerang tries to get the robot to get to a point at a specific angle.  

It does this by creating a new target point for the robot to follow that will (ideally) end the robot at your desired angle when it reaches the target point.  There is a [desmos page](https://www.desmos.com/calculator/sptjw5szex) that showcases how Boomerang works well.  

**Boomerang has limitations.**  The big limitation is distance to the target point.  The closer you are, the worse boomerang will perform.  You cannot go from (0, 0) to (0, 2) and be at a specific angle, the robot doesn't have enough time to build up that gradual angle change.  

Boomerang motions can also be a bit slower than Pure Pursuit motions.  Boomerang is changing the target much more rapidly (literally every tick), but the values change less and less the closer you get to the target position.  Your robot will slow down due to PID and Boomerang.  


## Turn to Point
At minimum, you need to give the robot a point to face, if it's facing it forward or backward, and a speed limit.  You can do this with and without okapi units.  
```cpp
// Turn to face 12, 12 forward
chassis.pid_turn_set({24, 24}, fwd,  90);
chassis.pid_wait();

// Turn to 0, 24 forward
chassis.pid_turn_set({0_in, 24_in}, fwd, 90);
chassis.pid_wait();

// Turn to face 12, 12 backward
chassis.pid_turn_set({24, 24}, rev,  90);
chassis.pid_wait();

// Turn to 0, 24 backward
chassis.pid_turn_set({0_in, 24_in}, rev, 90);
chassis.pid_wait();
```

Beyond this, all of the syntax for absolute turns applies to turning to a point, just give a point and a direction before speed.  You can find that syntax [here](/tutorials/turn_movements).

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