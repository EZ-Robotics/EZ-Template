---
title: Tuning Constants
description: Tuning the PID system to accurately rotate the requested amount.
---


# **Tuning Constants**



## Setup
For more information on how PID controllers work, [reference this document made by George Gillard](http://georgegillard.com/documents/2-introduction-to-pid-controllers). 

### Step 1 - Uncommenting Default Constants  
In `src/main.cpp`, uncomment `default_constants()` in `void initialize()`.  
```cpp
void initialize() {
    . . .

    default_constants();

    . . .
}
```


### Step 2 - Running the Autonomous  
You can run one of the example autonomous routines to tune your constants.  When you're tuning drive constants, run `drive_example()`.  When you're tuning turn constants, run `turn_example()`.

Look at our [Using Auton Selector tutorial](https://ez-robotics.github.io/EZ-Template/docs/Tutorials/using_auton_selector.html) for how to select autonomous routines!


### Step 3 - Modifying Constants  
In `src/autons.cpp`, there is a function called `default_constants()`.  This function is where all of your modified PID constants can be set.  
```cpp
void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}
```


### Step 4 - Different Constants
When your robot has huge weight shifts (grabbing a mobile goal, raising a lift, etc), you might want to have different constants for those states.  You can have multiple functions with constants for different states, and change constants during your autonomous routine.
```cpp
void grab_mogo() {
  chassis.set_drive_pid(40, 110, true);
  chassis.wait_drive();

  one_mogo_constants();
  chassis.set_drive_pid(-40, 110, true);
  chassis.wait_drive();
}
```

---


## Tuning PID

### Step 1 - kP
You'll start with kP.  Set it to some number, and run your example autonomous.  The robot will either undershoot the target (kP too low), or the robot is oscillate around the target (kP too high).  

To tune a PD controller, you want the kP to oscillate a little bit, usually a bounce or two.   

### Step 2 - kD
After finding a kP that oscillates a little, we can tune kD.  Increase kD until the oscillation goes away.  This movement should look more "snappy" then just a P loop.

### Step 3 - Repeat
Repeat Steps 1 and Steps 2 until kD cannot remove oscillation from the movement.  Then go back to the last values that worked. 

### Step 4 - kI
Sometimes you need a little extra power to get your robot all the way there.  Integral can be a dangerous variable because it grows exponentially.  The fourth parameter is what the error needs to be for I to start.  For turns, we found around 15 degrees is good. 

Increase kI until any minor disturbances are accounted for.  You might need to adjust kD while tuning kI.  

---


## Tuning Forward/Backward Constants  


### Step 1 - Forward/Backwards kP and kD
Using the steps above, modify your kP, kD, and if you chose to, kI.  

If you see the robot acting differently going forwards and backwards, you can have different constants for each.  This is usually only needed when something is off balance on your robot mechanically.
```cpp
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
```


### Step 2 - Heading Correction 
Heading correction tries to keep your robot facing an angle while driving forward.  The constants for this are generally higher then other PID controllers because you're only correcting for a few degrees of error.  

The same steps above can be used to tune the heading constants.  Increase kP until there's a little oscillation, increase kD until it goes away, repeat. 

```cpp
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
```


### Step 3 - Slew
`slew_min_power()` is the starting power the robot will use when slewing.  `slew_distance()` is the distance the robot will increase from min power to the max power you set in your autonomous function.  

Minimum power should be as high as it can be without causing the robot to wheelie or cause wheel slip.  

Slew distance should be as small as it can be without causing the same. 

```cpp
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
```


---


## Tuning Turns and Swings  


### Step 1 - Constants 
Using the same steps as above, set kP to some number and modify it until there is slight oscillation, one or two bounces. 

Increase kD until the oscillation is gone.  

Repeat until kD cannot fix the oscillation.  
```cpp
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
```


### Step 2 - kI
What about when PD isn't enough?  

Sometimes you need a little extra power to get your robot all the way there.  Integral can be a dangerous variable because it grows exponentially.  The fourth parameter is what the error needs to be for I to start.  For turns, we found around 15 degrees is good. 

Increase kI until any minor disturbances are accounted for.  You might need to adjust kD while tuning kI.   

```cpp
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
```