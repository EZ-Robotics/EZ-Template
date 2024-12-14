---
layout: default
title: Wheel Diameter
description: mess with what the robot thinks an inch is
---

:::note

This is not required for consistent autonomous routines, it only makes the correlation between what the robot thinks it's doing and what it's actually doing match more closely.  

:::

## What is it?
Tuning wheel diameter will change how far the robot goes when you input 1".  The larger this number is, the less 1" will be.  This can be useful in making very precise autonomous routines easier to get working first try by guaranteeing 1" is truly 1".  

## Tuning 
:::note

Blank pages are a feature of v3.2.0.  Please upgrade if you're on an older version!

:::
We'll print our drive sensors to a new blank page on the brain.  The code below will go in the screen task that starting shipping with the 3.2.0 example project.  
```cpp
else if (ez::as::page_blank_is_on(1)) {
  ez::screen_print("Left: " + util::to_string_with_precision(chassis.drive_sensor_left()) +
                   "\nRight: " + util::to_string_with_precision(chassis.drive_sensor_right()), 1);
}
```

If you're using tracking wheels, you can replace `chassis.drive_sensor_left()` and `chassis.drive_sensor_right()` with the following:
```cpp
chassis.odom_tracker_left->get();
chassis.odom_tracker_right->get();
chassis.odom_tracker_back->get();
chassis.odom_tracker_front->get();
```

To ensure our results can be trusted, let's give ourselves the ability to reset the drive sensors to 0.  In `while(true) { }` in opcontrol, we can add the chunk of code below.  
```cpp
if (master.get_digital(DIGITAL_L1)) {
  chassis.drive_sensor_reset();
}
```

Place your robot carefully, ideally against a wall to ensure consistency.  After resetting the drive sensors by pressing L1, we'll push the robot a known distance.  Some number will be in our terminal now, probably close to the known distance but not quite exact.  Keep track of what these numbers are and this test can be repeated a couple of times.  That'll leave us with this:
```
Left: 24.29   Right: 24.22
Left: 24.38   Right: 24.40
Left: 24.20   Right: 24.18
```

All of these numbers are close showing that I was consistently pushing the robot the known distance.  Averaging all of these numbers gives us `x`.  Using the equation below we can find our new wheel size.  
```
(known distance / averaged sensor values) * previous wheel size
```

For the example data above, the average sensor value is 24.2783.  The robot was pushed 24", and the robot currently thinks it's using 2.75" wheels.  Feeding these numbers into the equation above gives us this.  
```
(24 / 24.2783) * 2.75
```

## You're Done!
Take this new number, in my case 2.7185, and replace it with your wheel size in your drive constructor.  That's it!  