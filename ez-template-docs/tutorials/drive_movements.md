---
layout: default
title: Drive Movements
description: make robor go fwd and rev by itself
---

`pid_drive_set()` has many ways of calling it with many default values you can configure and override in specific motions.  



### Simplest Drive
At minimum, you need to give the robot a distance to travel and a speed limit (0-127).  You can do this with and without okapi units.  
```cpp
// Drive forward 24in
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

// Drive back 24in
chassis.pid_drive_set(-24_in, 110);
chassis.pid_wait();
```


### Slew
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




