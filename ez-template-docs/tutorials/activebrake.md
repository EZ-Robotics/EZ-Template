---
title: Active Brake
description: Powerful, tunable brake for driver control
---

If you put the motors on brake type hold, a robot can still push the robot a bit, and when you let go of the joysticks the robot just locks in place. Active brake runs a P loop on the drive when you let go of the joysticks. By adjusting the kP, you adjust how hard the robot fights back. If you make it smaller, there will be a larger dead zone and you'll coast a little bit. Active brake vs brake type is personal preference.  

This is nice to use while the robot's brake mode is set to `coast` so you have full control over how much the robot will fight back when pushed.  

## Enabling  
To adjust the kP, in `src/main.cpp` change `chassis.opcontrol_drive_activebrake_set(0)` to whatever you like! We suggest around `2.0`.

## Disabling  
To disable active brake, in `src/main.cpp` make sure the kP is 0 with `chassis.opcontrol_drive_activebrake_set(0)`.