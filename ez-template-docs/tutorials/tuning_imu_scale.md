---
layout: default
title: IMU Scaling
description: mess with what the imu thinks a degree is
---

:::note

This is not required for consistent autonomous routines, it only makes the correlation between what the robot thinks it's doing and what it's actually doing match more closely.  

:::

## What is it?
No imu reports exactly 1 degree of rotation for every degree the robot actually turns.  Rather than hand-tune a scaling factor by trial and error, you calibrate it directly: physically turn the robot 3600 degrees (10 full rotations) and tell EZ-Template what the imu reported for that turn.  Internally, the imu's readings are corrected so that value maps back to a true 3600.  

## Tuning 
Place your robot carefully, ideally aligned with tiles on the field so you know it's facing forward, and make sure nothing is calling `drive_imu_scaler_set()` yet, since a freshly constructed drive's imu is unscaled.  

Physically turn the robot exactly 3600 degrees (10 full rotations), either by hand or with a raw, unscaled auton.  
```cpp
chassis.pid_turn_set(3600_deg, 40, ez::raw);
chassis.pid_wait();
```

Read what the imu reported for that turn.  
```cpp
printf("%.2f\n", chassis.drive_imu_get());
```

Whatever that number is, pass it directly into `drive_imu_scaler_set()`.  You'll do this by adding this line of code to `default_constants()` in `src/autons.cpp`.  
```cpp
chassis.drive_imu_scaler_set(3625.42);  // Whatever your imu printed above
```

No trial and error needed, this is a one-shot calibration.  For drives built with the redundant IMU constructor, repeat this per imu and pass all the values to `drive_imus_scalers_set()` at once, in the same order as the IMU ports passed to the constructor.  

## You're Done!
That's it!  Your IMU is now tuned!  
