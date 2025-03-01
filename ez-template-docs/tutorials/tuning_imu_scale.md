---
layout: default
title: IMU Scaling
description: mess with what the imu thinks a degree is
---

:::note

This is not required for consistent autonomous routines, it only makes the correlation between what the robot thinks it's doing and what it's actually doing match more closely.  

:::

## What is it?
IMU scaling is a number that the IMU value gets multiplied by.  If this is set to 2, then when you turn 90°, the robot will turn ~45°, but it will think it has traveled 90°.  This can be useful in autonomous routines that require your robot to turn fully throughout a run, to guarantee -360° is the same place as 360°. 

## Tuning 
If we tell the robot to turn 360°, the robot will look like it's close enough.  We can see the error accumulation multiplied if we turn 10 times to 3600°, and this will happen with the line of code below being run in autonomous.  
```cpp
chassis.pid_turn_set(3600_deg, 40, ez::raw);
chassis.pid_wait();
```

Place your robot carefully, ideally align the chassis with tiles on the field so you know the robot is correctly facing forward.  Run the autonomous routine and see how far off it is. 

If the robot undershoots, you'll want to increase the imu scaler.  If the robot overshoots, you'll want to decrease the IMU scaler.  This number will be very sensitive, I suggest modifying it in increments of `0.05` to start.  You'll do this by adding this line of code to `default_constants()` in `src/autons.cpp`.  
```cpp
chassis.drive_imu_scaler_set(1.05);
```

## You're Done!
That's it!  Your IMU is now tuned!  
