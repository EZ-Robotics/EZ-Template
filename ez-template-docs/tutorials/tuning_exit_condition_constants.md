---
layout: default
title: Exit Condition Constants
description: tune constants so the robot actually knows when it's there
---

# Tuning Exit Conditions

:::note 

For more information on how to use exit conditions, [see this page](/tutorials/exiting_movements).  

:::

## Tuning
Ultimately you're tuning for 2 functions; timers and when to start timing for `pid_wait()`, and how much to add to target for `pid_wait_quick_chain()`.  

### Tuning pid_wait()
The values to tune are in `default_constants()` in your `src/autons.cpp`.  The parameters are:
* The first number is the amount of time needed to exit when within the second number of your target  
* The third number is the amount of time needed to edit when within the fourth number of your target   
* The fifth number is the amount of time to edit when the velocity of the robot is 0  
* The sixth number is the amount of time to exit when you're pulling too many amps from your motors  
```cpp
void default_constants() {
  // . . .
  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  // . . .
}
```

I like to use the `drive_and_turn()` example autonomous routine when tuning PID.  You get turns that happen back to back and some drive motions that lead into them.  

The default constants are already pretty aggressive, you shouldn't have to tune them much.  You can adjust the window that the timers start within and you can adjust the amount the timer has to reach before exiting.  If the window is too large, you'll exit too early.  If the time to wait is too low, you'll exit too early.  This can all be adjusted retroactively while making real autonomous routines 

You should make these numbers as low as you can without causing inconsistencies.  With `pid_wait()` you generally want the robot to be pretty close to stopped before it moves on to the next motion.  

### Tuning pid_wait_quick_chain()
You can tune the amount added to `target` during chained motions.  

**Larger constants** will carry more momentum into the next motion, making your autonomous faster but potentially hurting consistency.  **Smaller constants** will carry less momentum into the next motion, this will still be significantly faster then any other method of exiting but could be slightly less consistent than `pid_wait()`.  

If I were making autonomous routines, I would try to use a smaller constant for normal motions and only push a larger constant when an autonomous routine would greatly benefit from it.  


#### Driving
You have independent control over forward and backward constants for this.  If your robot doesn't have problems with tipping or wheel slip, you should be good with the normal `pid_drive_chain_constant_set()`.  
```cpp
void default_constants() {
  // . . .
  chassis.pid_drive_chain_constant_set(3_in);  // This sets forwards and backwards constants
  chassis.pid_drive_chain_forward_constant_set(3_in);
  chassis.pid_drive_chain_backward_constant_set(3_in);
  // . . .
}
```

#### Turning
This is the simple one.  We assume your robot is symmetrical in turning and you only get 1 function to tune.  
```cpp
void default_constants() {
  // . . .
  chassis.pid_turn_chain_constant_set(3_deg);  
  // . . .
}
```

#### Swinging
You have independent control over forward and backward constants for this.  If your robot doesn't have problems with tipping or wheel slip, you should be good with the normal `pid_swing_chain_constant_set()`.  
```cpp
void default_constants() {
  // . . .
  chassis.pid_swing_chain_constant_set(5_deg);  // This sets forwards and backwards constants
  chassis.pid_swing_chain_forward_constant_set(5_deg);
  chassis.pid_swing_chain_backward_constant_set(5_deg);
  // . . .
}
```