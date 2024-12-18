---
layout: default
title: Mirroring Autons
description: it's just like looking in a mirror
---

## Why?
Some games require unique autons for red and blue.  Turning Point, Tower Takeover, and now High Stakes.  

You can have a completely unique auton for red and blue, but when they're going to be doing almost exactly the same thing, this feels inefficient and easy to make mistakes.  

EZ-Template allows you to flip the X, Y, and Rotation axis so you can keep one base auton and have it work for red and blue.  

## Flipping and Reading Flips
These are the functions you can use to flip axis.  By default, the `_get()` returns false.  
```cpp
odom_y_flip();
odom_x_flip();
odom_theta_flip();
odom_y_direction_get();  // True = down is positive Y, False = up is positive Y
odom_x_direction_get();  // True = left is positive X, False = right is positive X
odom_theta_direction_get(); // True = positive is counterclockwise, False = positive is counterclockwise
```

:::note

Flipping works without odom too!  You'll only need to use `odom_theta_flip()` and `odom_theta_direction_get()`

:::

## Example
You'll need three functions per auton that you want flipped:
- a base function
- one for red
- one for blue

You'll make this base auton work for one side.  If it's red (like the example below), your red function will just call the base function.  Your blue function will flip what's required and then it will call the base auton.  

Robots aren't perfect and your autonomous routine will most likely not work perfectly.  But because we can read if any axis is flipped, we can have unique values for red vs blue.  This is shown in the example below.  
```cpp
void base() {
  ez::united_pose start_intaking_here;

  if (chassis.odom_x_direction_get() == false)
    start_intaking_here = {12_in, 24_in};  // If red
  else
    start_intaking_here = {15_in, 24_in};  // If blue

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {start_intaking_here, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 48_in, 45_deg}, fwd, 110},
                       true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, 110, true);
  chassis.pid_wait();
}

void red() {
  base();
}

void blue() {
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  base();
}
```