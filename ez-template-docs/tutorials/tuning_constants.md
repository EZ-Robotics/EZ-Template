---
title: Tuning Constants
description: Tuning the PID system to accurately rotate the desired amount
---


# **Tuning Constants**
For more information on how PID controllers work, [reference this document made by George Gillard](http://georgegillard.com/documents/2-introduction-to-pid-controllers). 


## Setup

### Uncommenting Default Constants  
In `src/main.cpp`, make sure `default_constants()` is uncommented in `void initialize()`.  
```cpp
void initialize() {
    . . .
    default_constants();
    . . .
}
```


### Running the Autonomous  
You can run one of the example autonomous routines to tune your constants.  When you're tuning drive constants, run `drive_example()`.  When you're tuning turn constants, run `turn_example()`.

Look at our [Using Auton Selector tutorial](https://ez-robotics.github.io/EZ-Template/tutorials/using_auton_selector) for how to select autonomous routines!


### Modifying Constants  
In `src/autons.cpp`, there is a function called `default_constants()`.  This function is where all of your modified PID constants can be set.  
```cpp
void default_constants() {
  // PID Constants
  chassis.pid_drive_constants_set(10, 0, 100); // Sets forward and backward
  chassis.pid_heading_constants_set(3, 0, 20);
  chassis.pid_turn_constants_set(3, 0, 20);
  chassis.pid_swing_constants_set(5, 0, 30); // Sets forward and backward

  // Swing Constants
  chassis.slew_drive_constants_set(7_in, 80); // Sets forward and backward
  chassis.slew_turn_constants_set(5_deg, 50);
  chassis.slew_swing_constants_set(5_deg, 50); // Sets forward and backward
}
```


### Different Constants
When your robot has huge weight shifts (grabbing a mobile goal, raising a lift, etc), you might want to have different constants for those states.  You can have multiple functions with constants for different states and change constants during your autonomous routine.
```cpp
void grab_mogo() {
  chassis.pid_drive_set(40_in, 110, true);
  chassis.pid_wait();

  one_mogo_constants();
  chassis.pid_drive_set(-40_in, 110, true);
  chassis.pid_wait();
}
```



### Using the PID Tuner
EZ-Template's PID Tuner allows users to tune PID faster.  The alternative to using this is to reupload your code every time you want to try new constants. 

To enable it, we can use `chassis.pid_tuner_toggle();`.  In the code below it is only allowed to run when we're not connected to a competition switch or tournament.  This is done to avoid accidentally leaving this code in your code while at a competition and running your autonomous during the driver controlled period.  

Once the PID Tuner is enabled we can navigate the menus on the brain screen with the arrow keys on the controller.  To increase or decrease the constant we can use A and Y on the controller.  

To select your autonomous routine, navigate the autonomous selector until you find the one you want to run.  Then once you switch to the PID Tuner, the last auton page you were on will run when you press B on your controller.  
```cpp
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  
  while (true) {
    
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) { 
      // Enable / Disable PID Tuner
      if (master.get_digital_new_press(DIGITAL_X)) 
        chassis.pid_tuner_toggle();
        
      // Trigger the selected autonomous routine
      if (master.get_digital_new_press(DIGITAL_B)) 
        autonomous();

      chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate
    } 

    chassis.opcontrol_tank(); // Tank control

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
---

## Tuning PID

This video is a good visual representation of what each constant does. 
<iframe width="560" height="315" src="https://www.youtube.com/embed/qKy98Cbcltw?si=tC9FrdKh6DR1Imrp" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

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


### Step 1 - Forward/Backwards Constants
Using the steps above, modify your kP, kD, and if you chose to, kI.  

If you see the robot acting differently going forwards and backwards, you can have different constants for each.  This is usually only needed when something is off balance on your robot mechanically.
```cpp
  chassis.pid_drive_constants_forward_set(10, 0, 100);
  chassis.pid_drive_constants_backward_set(10, 0, 100);
```

If you don't have problems with differences between forward and backwards, you can use the following instead.  
```cpp
  chassis.pid_drive_constants_set(10, 0, 100); // Sets forward and backward
```

### Step 2 - Heading Correction 
Heading correction tries to keep your robot facing an angle while driving forward.  The constants for this are generally higher then other PID controllers because you're only correcting for a few degrees of error.  

The same steps above can be used to tune the heading constants.  Increase kP until there's a little oscillation, increase kD until it goes away, repeat. 

```cpp
  chassis.pid_heading_constants_set(3, 0, 20);
```


### Step 3 - Slew
Slew ramps the speed of the robot up from the start of the motion to avoid wheelies and wheel slip.  
The first parameter is the distance the robot will increase from min power to the max power you set in your autonomous function.  
The second parameter is the starting power the robot will use when slewing.    

Minimum power should be as high as it can be without causing the robot to wheelie or cause wheel slip.  Slew distance should be as small as it can be without causing the same. 

```cpp
  chassis.slew_drive_constants_set(7_in, 80); // This sets forwards and backwards constants
```

If your robot behaves differently forward and backwards, you can set different slew rates for both. 
```cpp
  chassis.slew_drive_constants_forward_set(7_in, 80);
  chassis.slew_drive_constants_backward_set(7_in, 80);
```


---


## Tuning Turns  


### Step 1 - Constants 
Using the same steps as above, set kP to some number and modify it until there is slight oscillation, one or two bounces. 

Increase kD until the oscillation is gone.  

Repeat until kD cannot fix the oscillation.  
```cpp
  chassis.pid_turn_constants_set(3, 0, 20);
```


### Step 2 - kI
What about when PD isn't enough?  

Sometimes you need a little extra power to get your robot all the way there.  Integral can be a dangerous variable because it grows exponentially.  The fourth parameter is what the error needs to be for I to start.  For turns, we found around 15 degrees is good. 

Increase kI until any minor disturbances are accounted for.  You might need to adjust kD while tuning kI.   

```cpp
  chassis.pid_turn_constants_set(3, 0.003, 20, 15);
```


### Step 3 - Slew
Just like with drive motions, you can use slew with turns.  The input distance is an angle instead of a length unit, but it works in the same way.  

The first parameter is the distance the robot will increase from min power to the max power you set in your autonomous function.  
The second parameter is the starting power the robot will use when slewing.    
```cpp
  chassis.slew_turn_constants_set(5_deg, 50);
```

---


## Swings  

### Forwards and Backwards PID Constants
Just like with driving, you can have independent forward and backward constants for your swing turns. 
```cpp
  chassis.pid_swing_constants_forward_set(5, 0, 30);
  chassis.pid_swing_constants_forward_set(5, 0, 30);
```

And if this isn't a problem for your robot, you can just use this function. 
```cpp
  chassis.pid_swing_constants_set(5, 0, 30); // Sets forward and backward
```

### Slew with Angle as Target
Swing turns are a middle ground between driving and turns.  Because of this, you have two options for how you want to use slew.  This option uses angle just like turns do.    
```cpp
  chassis.slew_swing_constants_set(5_deg, 50); // Sets forward and backward
```

If your robot is different forwards and backwards, these are the functions you'd want to use.  
```cpp
  chassis.slew_swing_constants_forward_set(5_deg, 50);
  chassis.slew_swing_constants_backward_set(5_deg, 50);
```

### Slew with Length as Target
You can use length instead of angle to accomplish the same thing with swing turns.  
```cpp
  chassis.slew_swing_constants_set(7_in, 50); // Sets forward and backward
```

And, big surprise, you can have independent forward and backward control over the constants.  
```cpp
  chassis.slew_swing_constants_forward_set(7_in, 50);
  chassis.slew_swing_constants_backward_set(5_in, 50);
```

---

## General Tips

### Find Your Maximum
When you're tuning PID, don't increase kP by 0.1 every time you run.  If 1.0 isn't high enough, try 10.0.  If that's still not high enough, go to 20.0.  In this process you'll quickly find a minimum and maximum value that it can be within.  

### Change it Up
Your goal is to make PID constants that generally work on all of your drive motions.  If you're tuning constants to a single motion, it's less likely to work on everything you make.  

Using the example autons to tune is a good start.  But once you're in the ballpark of where you want to be, start slowly making an autonomous routine and you can tune it more as you go.  Once that first autonomous routine is done, your constants will be good.