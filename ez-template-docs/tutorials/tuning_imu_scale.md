---
layout: default
title: IMU Scaling
description: mess with what the imu thinks a degree is
---

:::note

This is not required for consistent autonomous routines, it only makes the correlation between what the robot thinks it's doing and what it's actually doing match more closely.  

:::

:::caution Coming from 3.2.x?

In 3.2.x, `drive_imu_scaler_set()` took a multiplier like `1.007`.  In 4.0 it was renamed to `drive_imu_scaler_3600_set()` and takes what the imu reads after a 3600 degree turn, so a call from 3.2.x will not compile.  Remove the old line and follow the steps below to calibrate again.  

`drive_imu_scaler_get()`, `drive_imus_scalers_set()` and `drive_imus_scalers_get()` were renamed the same way, with `_3600` added.  

:::

## What is it?
No imu reports exactly 1 degree of rotation for every degree the robot actually turns.  Rather than hand-tune a scaling factor by trial and error, you calibrate it directly: physically turn the robot 3600 degrees (10 full rotations) and tell EZ-Template what the imu reported for that turn.  Internally, the imu's readings are corrected so that value maps back to a true 3600.  

## Tuning 
Place your robot carefully, ideally aligned with tiles on the field so you know it's facing forward, and make sure nothing is calling `drive_imu_scaler_3600_set()` yet, since a freshly constructed drive's imu is unscaled.  

Physically turn the robot exactly 3600 degrees (10 full rotations) by hand.  Don't use an autonomous routine for this.  `pid_turn_set()` stops when the imu reads its target, so the imu would report 3600 no matter how far the robot really turned.  Line the robot up with a tile edge at the start and after every rotation so you know each one was a full turn.  

While you turn it, show what the imu reports on a blank page.  Add this to `ez_screen_task()` in `main.cpp`, under `// Add your own blank pages here!`, and upload it.  In the example project blank page 0 shows odometry and blank page 1 shows motor temperatures, so this is page 2.  If you've added pages of your own, use the next number that's free.  Then go left on the autonomous selector until you're on the page.  [Blank Pages](blank_pages.md) explains how they work.  
```cpp
else if (ez::as::page_blank_is_on(2)) {
  ez::screen_print("IMU: " + ez::util::to_string_with_precision(chassis.drive_imu_get()), 1);  // What the imu has measured so far, in degrees
}
```

The imu reads 0 once it finishes calibrating, so wait for the loading bar to finish before you start turning the robot.  Once you've finished the 10th rotation, write down the number on the screen.  

Whatever that number is, pass it directly into `drive_imu_scaler_3600_set()`.  You'll do this by adding this line of code to `default_constants()` in `src/autons.cpp`.  
```cpp
chassis.drive_imu_scaler_3600_set(3625.42);  // Whatever your imu printed above
```

No trial and error needed, this is a one-shot calibration.  For drives built with the redundant IMU constructor, repeat this per imu and pass all the values to `drive_imus_scalers_3600_set()` at once, in the same order as the IMU ports passed to the constructor.  

## You're Done!
That's it!  Your IMU is now tuned!  
