---
layout: default
title: Drive Movements
description: make robor go fwd and rev by itself
---

`pid_drive_set()` has many ways of calling it with many default values you can configure and override in specific motions.  

## How does it work?
`pid_drive_set()` moves the drive forward and backwards **relatively**.  This means the robot will move relative to where it currently is.  

Relative motions allow for error accumulation.  If you tell the robot to go 24" and it will go anywhere from 22" - 26", the rest of your run will be off by this amount.  

To help fight general error accumulation, `pid_drive_set()` will maintain an **absolute heading**.  What does this mean?  The robot runs turn PID and drive PID at the same time.  The target for the turn PID is the last turn the robot made.  To start the robot faces 0 degrees, so this is the target it will use for driving.  After the robot turns 90 degrees, 90 becomes the new target for heading.  This allows drive motions to help fight some of the error accumulation that gets built up from turns.  

Maintaining an angle while driving is called "heading" in EZ-Template.  This has a unique set of PID constants that you can tune independently from turns.  

EZ-Template will also run independent PID loops on the left and right side of your drive.  Because of this, EZ-Template will use the motor encoders if you only have one vertical tracking wheel.  The tracking wheel will be used to get your XY position and for all odometry motions, but the motor encoders will still be used for this function unless you have two vertical tracking wheels.  

## Simplest Drive
At minimum, you need to give the robot a distance to travel and a speed limit (0-127).  You can do this with and without okapi units.  
```cpp
// Drive forward 24in
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

// Drive back 24in
chassis.pid_drive_set(-24_in, 110);
chassis.pid_wait();
```


## Slew
Slew ramps up the speed limit of the robot to give smoother accelerations.  
```cpp
chassis.slew_drive_set(true);  // Enables global slew
chassis.slew_drive_constants_set(5_in, 50);

// Over the first 5 inches, the robot will ramp the speed limit from 50 to 110 
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();
```

You can change the behavior of slew for each motion.  In the example below, slew is disabled globally but is enabled in one of the motions.   
```cpp
chassis.slew_drive_set(false);  // Disables global slew
chassis.slew_drive_constants_set(5_in, 50);

// Over the first 5 inches, the robot will ramp the speed limit from 50 to 110 
chassis.pid_drive_set(24_in, 110, true);  // Slew will be enabled for this motion
chassis.pid_wait();

// This will not use slew to travel backwards 24in
chassis.pid_drive_set(-24_in, 110);  
chassis.pid_wait();
```




