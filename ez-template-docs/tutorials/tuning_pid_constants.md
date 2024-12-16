---
layout: default
title: PID Constants
description: tune pid constants so the robot does what you want it to do
---

For more information on how PID controllers work, [reference this document made by George Gillard](http://georgegillard.com/documents/2-introduction-to-pid-controllers). 


## Setup

### Uncommenting Default Constants  
In `src/main.cpp`, make sure `default_constants()` is uncommented in `void initialize()`.  
```cpp
void initialize() {
  // . . .
  default_constants();
  // . . .
}
```


### Running the Autonomous  
You can run one of the example autonomous routines to tune your constants.  When you're tuning drive constants, run `drive_example()`.  When you're tuning turn constants, run `turn_example()`.

Look at our [Using Auton Selector tutorial](/tutorials/using_ez) for how to select autonomous routines!


### Modifying Constants  
In `src/autons.cpp`, there is a function called `default_constants()`.  This function is where all of your modified PID constants can be set.  
```cpp
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions
  // . . .
}
```


### Different Constants
When your robot has huge weight shifts (grabbing a mobile goal, raising a lift, etc), you might want to have different constants for those states.  You can have multiple functions with constants for different states and change constants during your autonomous routine.
```cpp
void grab_mogo() {
  default_constants();  // Use default constants
  chassis.pid_drive_set(40_in, 110, true);
  chassis.pid_wait();

  one_mogo_constants();  // Switch over to constants tuned while holding a mogo
  chassis.pid_drive_set(-40_in, 110, true);
  chassis.pid_wait();
}
```



### Using the PID Tuner
EZ-Template's PID Tuner allows users to tune PID faster.  The alternative to using this is to reupload your code every time you want to try new constants. 

To enable it, we can use `chassis.pid_tuner_toggle();`.  In the example project, it is only allowed to run when we're not connected to a competition switch or tournament.  This is done to avoid accidentally leaving this code in your code while at a competition and running your autonomous during the driver-controlled period.  

Once the PID Tuner is enabled we can navigate the menus on the brain screen with the arrow keys on the controller.  To increase or decrease the constant we can use A and Y on the controller.  

To select your autonomous routine, navigate the autonomous selector until you find the one you want to run.  Then once you switch to the PID Tuner, the last auton page you were on will run when you press DOWN and B on your controller.  

---

## Tuning PID
This video is a good visual representation of what each constant does. 
<iframe width="560" height="315" src="https://www.youtube.com/embed/qKy98Cbcltw?si=tC9FrdKh6DR1Imrp" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

### Step 1 - kP
You'll start with kP.  Set it to some number, and run your example autonomous.  The robot will either undershoot the target (kP too low), or the robot is oscillate around the target or cause undesired behavior (kP too high).  

To start tuning kD, you'll need kP to cause the robot to have undesired behavior.  This can be oscillation (the robot "bouncing" around the target), wheelies, wheel slipping.  kP should be the smallest it can be to start to cause undesired behavior.  

### Step 2 - kD
After finding a kP that causes undesired behavior, we can tune kD.  Increase kD until the oscillation goes away.  This movement should look more "snappy" than just a P loop.

### Step 3 - Repeat
Repeat Steps 1 and Steps 2 until kD cannot remove oscillation from the movement.  Then go back to the last values that worked. 

### Step 4 - kI
Sometimes you need a little extra power to get your robot all the way there.  Integral can be a dangerous variable because it grows exponentially.  The fourth parameter is what the error needs to be for I to start.  For turns, we found around 15 degrees is good. 

Increase kI until any minor disturbances are accounted for.  You might need to adjust kD while tuning kI.  

---


## Tuning Forward/Backward Constants  


### Step 1 - Forward/Backwards Constants
Using the steps above, modify your kP, kD, and if you choose to, kI.  These constants are used for normal driving motions and for odometry motions.  

Sometimes instead of seeing the robot oscillate around your target, you'll see the robot wheelie really hard or wheel slip.  These are **undesired behaviors** and it means your kP is too high.  

If you see the robot acting differently going forward and backward, you can have different constants for each.  This is usually only needed when something is off balance on your robot mechanically.
```cpp
chassis.pid_drive_constants_forward_set(20.0, 0.0, 100.0);
chassis.pid_drive_constants_backward_set(20.0, 0.0, 100.0);
```

If you don't have problems with differences between forward and backward, you can use the following instead.  
```cpp
chassis.pid_drive_constants_set(20.0, 0.0, 100.0);  // Sets forward and backward
```

### Step 2 - Heading Correction 
Heading correction tries to keep your robot facing an angle while driving forward.  The constants for this are generally higher than other PID controllers because you're only correcting for a few degrees of error.  

The same steps above can be used to tune the heading constants.  Increase kP until there's a little oscillation, increase kD until it goes away, repeat. 

```cpp
chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
```

You should tune this while having the robot perform the start of an autonomous routine.  Heading correction is meant to correct for errors in your turns and swings.  If you tune this and only have the robot start from a standstill, it won't be correcting for anything.  The extra error needs to be introduced for heading tuning to be useful.  


---


## Tuning Turns  


### Step 1 - Constants 
Using the same steps as above, set kP to some number and modify it until there is slight oscillation, one or two bounces. 

Increase kD until the oscillation is gone.  

Repeat until kD cannot fix the oscillation.  
```cpp
chassis.pid_turn_constants_set(3.0, 0.0, 20.0);
```


### Step 2 - kI
What about when PD isn't enough?  

Sometimes you need a little extra power to get your robot all the way there.  Integral can be a dangerous variable because it grows exponentially.  The fourth parameter is what the error needs to be for I to start.  For turns, we found around 15 degrees is good. 

Increase kI until any minor disturbances are accounted for.  You might need to adjust kD while tuning kI.   

```cpp
chassis.pid_turn_constants_set(3, 0.003, 20, 15);
```


---


## Tuning Swing Turns   

### Forward and Backward PID Constants
Just like with driving, you can have independent forward and backward constants for your swing turns. 
```cpp
chassis.pid_swing_constants_forward_set(6.0, 0.0, 65.0);
chassis.pid_swing_constants_forward_set(6.0, 0.0, 65.0);
```

And if this isn't a problem for your robot, you can just use this function. 
```cpp
chassis.pid_swing_constants_set(6.0, 0.0, 65.0);  // Sets forward and backward
```

## Odometry
:::warning

Make sure you have drive PID constants tuned before starting to tune the below constants!

:::

The process for tuning PID for odom_angular and odom_boomerang is the same as everything above, except there is an extra parameter that changes the priority of turning vs driving during odometry motions.  

This number can be closer to 1.0 if you have traction wheels or if you have a horizontal tracker.  If you have omnis this number should be lower, and if you have omnis without a horizontal tracker this number should be even lower.  

Using the default constants, have the robot travel to (0, 24) then (24, 24) with pure pursuit.  Adjust this number until it looks like the robot is sticking to the path.  You might need to come in and change this number more later, but doing this to start will give you a good starting point.   
```cpp
// The amount that turns are prioritized over driving in odom motions
// - if you have tracking wheels, you can run this higher.  1.0 is the max
chassis.odom_turn_bias_set(0.9);
```


### Angular
Run a test program where the robot travels to (0, 24) then (24, 24) with pure pursuit.  

Increase kP until the robot starts to oscilate around the target heading a little, increase kD until this goes away.  If needed you can add kI too.  

```cpp
chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);
```

### Boomerang
Run a test program where the robot travels to (24, 24, 90) with boomerang.  

Increase kP until the robot starts to oscilate around the target heading a little, increase kD until this goes away.  If needed you can add kI too.  

```cpp
chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);
```

---


## General Tips

### Quickly Find Your Maximum
When you're tuning PID, don't increase kP by 0.1 every time you run.  If 1.0 isn't high enough, try 10.0.  If that's still not high enough, go to 20.0.  In this process, you'll quickly find a minimum and maximum value that it can be within.  

### Change it Up
Your goal is to make PID constants that generally work on all of your drive motions.  If you're tuning constants to a single motion, it's less likely to work on everything you make.  

Using the example autons to tune is a good start.  But once you're in the ballpark of where you want to be, start slowly making an autonomous routine and you can tune it more as you go.  Once that first autonomous routine is done, your constants will be good.