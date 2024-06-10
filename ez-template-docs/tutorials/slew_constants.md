---
title: Slew Constants
description: Ramping up speed so your robot doesn't tip
---

:::note

Most robots will work perfectly fine without slew.  You can visit this page if you start to see the undesired behavior described below.  

:::

## What is it?
Slew is when your motors start at a lower speed and build up to a faster speed.  This can be useful if your robot is prone to tipping, can wheelie easily, or is prone to wheel slip at the start of motions.   

## Tuning Slew 
There are 2 parameters to tune with slew in EZ-Template; the distance to slew for and starting speed.  The robot will accelerate very quickly with a low distance to slew for, regardless of the starting speed.  

I will tune slew by running normal motions but having the robot go slower.  The goal is to find the fastest the robot can go without any of the undesirable behaviors described above.  To enable slew you need the `true` parameter at the end of your `x_pid_set()`.  
```cpp
void tuning_slew() {
  chassis.drive_pid_set(24_in, 80, true);
}
```

After you've found this number, you can ensure this line is in your `default_constants()` function in `src/autons.cpp`.  This should be near the top.  You can set the second number, speed, to the number you've found in the test above.  
```cpp
void default_constants() {
  // . . .
  chassis.slew_drive_constants_set(7_in, 80);
  // . . .
}
```

Now you can lower the distance until the robot is able to successfully reach `DRIVE_SPEED` without undesirable behavior!  
```cpp
void tuning_slew() {
  chassis.drive_pid_set(24_in, DRIVE_SPEED, true);
}
```

## Specifics for Each Motion

### Driving
Sometimes robots aren't symmetrical and can be tippier in one direction than the other.  This can be solved by choosing one of the following functions to place in `default_constants()`.  
```cpp
chassis.slew_drive_constants_set(7_in, 80);  // This sets forwards and backwards constants
chassis.slew_drive_constants_forward_set(7_in, 80);
chassis.slew_drive_constants_backward_set(7_in, 80);
```
### Turning
This is the simplest one.  Turning should be symmetrical on your robot, so this is the only function to tune with.  
```cpp
chassis.slew_turn_constants_set(5_deg, 50);
```

### Swinging
Swinging is the most complex.  Because of the introduction of arcs, EZ-Template now supports either inches or degrees for tuning slew.  You can also have independent forward and backward constants.  it doesn't matter if you decide to use inches or degrees for tuning, you can change it later.  

Using degrees, these are your options for setting slew constants. 
```cpp
chassis.slew_swing_constants_set(5_deg, 50);  // This sets forwards and backwards constants
chassis.slew_swing_constants_forward_set(5_deg, 50);
chassis.slew_swing_constants_backward_set(5_deg, 50);
```

Using inches, these are your options for setting slew constants. 
```cpp
chassis.slew_drive_constants_set(7_in, 80);  // This sets forwards and backwards constants
chassis.slew_drive_constants_forward_set(7_in, 80);
chassis.slew_drive_constants_backward_set(7_in, 80);
```
