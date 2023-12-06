---
title: Active Brake
description: How the motors will brake and stop unwanted motion.
---

# **Active Brake**

## Introduction  
If you put the motors on brake type hold, a robot can still push the robot a bit, and when you let go of the joysticks the robot just locks in place. Active brake runs a P loop on the drive when you let go of the joysticks. By adjusting the kP, you adjust how hard the robot fights back. If you make it smaller, there will be a larger dead zone and you'll coast a little bit. Active brake vs brake type is personal preference.

## Enabling  
To adjust the kP, in `src/main.cpp` change `chassis.set_active_brake(0)` to whatever you like! We suggest around `0.1`.

## Disabling  
To disable active brake, in `src/main.cpp` make sure the kP is 0 with `chassis.set_active_brake(0)`.