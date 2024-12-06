---
layout: default
title: Turning
description:  Turning in place
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Overview
:::note

This overview section is not intended to be a full tutorial.  Check the Tutorial section for our tutorials!

:::
`pid_turn_set()` has many ways of calling it with many default values you can configure and override in specific motions.  This is a quick overview of how you can use them.  


### Default Turning

At minimum, you need to give the robot an angle to go to and a speed limit.  You can do this with and without okapi units.  
```cpp
// Turn to 45deg
chassis.pid_turn_set(45_deg, 90);
chassis.pid_wait();

// Turn to 0deg
chassis.pid_turn_set(0, 90);
chassis.pid_wait();
```
### Turn Paths
You can set the robot to always take a specific path.  
```cpp
chassis.pid_turn_behavior_set(ez::shortest);

// This will turn 1 degree to the right
chassis.pid_turn_set(361_deg, 90);
chassis.pid_wait();
```

But you can override these defaults in each motion.  
```cpp
chassis.pid_turn_behavior_set(ez::shortest);

// This will turn 361 degrees to the left
chassis.pid_turn_set(361_deg, 90, ez::longest);
chassis.pid_wait();
```

You can also pick between turning clockwise and counterclockwise.  
```cpp
chassis.pid_turn_behavior_set(ez::shortest);

// This will turn to 180deg counter clockwise
chassis.pid_turn_set(180_deg, 90, ez::ccw);
chassis.pid_wait();

// This will turn to 0 deg clockwise
chassis.pid_turn_set(180_deg, 90, ez::cw);
chassis.pid_wait();
```

### Slew
Slew ramps up the speed limit of the robot to give smoother accelerations.  
```cpp
chassis.slew_turn_set(true);  // Enables global slew
chassis.slew_turn_constants_set(5_deg, 50);

// Over the first 5 degrees, the robot will ramp the speed limit from 50 to 110 
chassis.pid_turn_set(90_deg, 110);
chassis.pid_wait();
```

You can change the behavior of slew for each motion.  In the example below, slew is disabled globally but is enabled in one of the motions.   
```cpp
chassis.slew_turn_set(false);  // Disables global slew
chassis.slew_turn_constants_set(5_deg, 50);

// Over the first 5 degrees, the robot will ramp the speed limit from 50 to 110 
chassis.pid_turn_set(90_deg, 110, true);  // Slew will be enabled for this motion
chassis.pid_wait();

// This will not use slew to return to 0 degrees
chassis.pid_turn_set(0_deg, 110, true);  
chassis.pid_wait();
```

### Turn Paths and Slew

You can use slew in conjunction with different turn behavior.  
```cpp
chassis.pid_turn_behavior_set(ez::shortest);
chassis.slew_turn_set(false);  // Enables global slew
chassis.slew_turn_constants_set(5_deg, 50);

// This will turn to 270 deg clockwise while slewing
chassis.pid_turn_set(270_deg, 90, ez::cw, true);
chassis.pid_wait();

// This will take the shortest path to 0 degrees without slew
chassis.pid_turn_set(3600_deg, 90);
chassis.pid_wait();
```

---


























## Functions with Okapi Units


### pid_turn_set()
Sets the drive to turn using PID.  

`p_target` is an okapi angle unit.  
`speed` is 0 to 127.  
<Tabs
  groupId="pid_turn_0a9sk1k2jhset_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  chassis.pid_turn_set(24_deg, 110);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(okapi::QAngle p_target, int speed);
```

</TabItem>
</Tabs>






### pid_turn_set()
Sets the drive to turn using PID.  

`p_target` is an okapi angle unit.  
`speed` is 0 to 127.  
`behavior` the default behavior for turns.  This can be `ez::shortest`, `ez::longest`, `ez::cw`, `ez::ccw`, or `ez::raw`.   
<Tabs
  groupId="pid_turn_s098sfa8et_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  chassis.pid_turn_set(22.5_deg, 110, ez::longest);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior);
```

</TabItem>
</Tabs>







### pid_turn_set()
Sets the drive to turn using PID.  

`p_target` is an okapi angle unit.  
`speed` is 0 to 127.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!
<Tabs
  groupId="pid_tur1234jkh134n_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  chassis.pid_turn_set(24_deg, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(okapi::QAngle p_target, int speed, bool slew_on);
```

</TabItem>
</Tabs>





### pid_turn_set()
Sets the drive to turn using PID.  

`p_target` is an okapi angle unit.  
`speed` is 0 to 127.  
`behavior` the default behavior for turns.  This can be `ez::shortest`, `ez::longest`, `ez::cw`, `ez::ccw`, or `ez::raw`.   
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!    
<Tabs
  groupId="pid_turn_set_okapasdasdasi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  chassis.pid_turn_set(22.5_deg, 110, ez::longest, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);
```

</TabItem>
</Tabs>






































### pid_turn_relative_set()
Sets the drive to turn using PID.  Target is relative here, the robot will add the target to your current angle.  

`p_target` is an okapi angle unit.  
`speed` is 0 to 127.  
<Tabs
  groupId="pid_turn_re098asd12l_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  // Turns to 90 deg
  chassis.pid_turn_relative_set(90_deg, 110);
  chassis.pid_wait();

  // Turns backwards by 45 degrees
  chassis.pid_turn_relative_set(-45_deg, 110);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_relative_set(okapi::QAngle p_target, int speed);
```

</TabItem>
</Tabs>








### pid_turn_relative_set()
Sets the drive to turn using PID.  Target is relative here, the robot will add the target to your current angle.  

`p_target` is an okapi angle unit.  
`speed` is 0 to 127.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!
<Tabs
  groupId="pid_turn_1lj235drel_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  // Turns to 90 deg
  chassis.pid_turn_relative_set(90_deg, 110, true);
  chassis.pid_wait();

  // Turns backwards by 45 degrees
  chassis.pid_turn_relative_set(-45_deg, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_relative_set(okapi::QAngle p_target, int speed, bool slew_on);
```

</TabItem>
</Tabs>








### pid_turn_relative_set()
Sets the drive to turn using PID.  Target is relative here, the robot will add the target to your current angle.  

`p_target` is an okapi angle unit.  
`speed` is 0 to 127.  
`behavior` the default behavior for turns.  This can be `ez::shortest`, `ez::longest`, `ez::cw`, `ez::ccw`, or `ez::raw`.   
<Tabs
  groupId="pid_tu09aarn_rel_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  // Turns to 90 deg
  chassis.pid_turn_relative_set(90_deg, 110, ez::shortest);
  chassis.pid_wait();

  // Turns the long way around to go -45 from where it started
  chassis.pid_turn_relative_set(-45_deg, 110, ez::longest);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_relative_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior);
```

</TabItem>
</Tabs>


### pid_turn_relative_set()
Sets the drive to turn using PID.  Target is relative here, the robot will add the target to your current angle.  

`p_target` is an okapi angle unit.  
`speed` is 0 to 127.  
`behavior` the default behavior for turns.  This can be `ez::shortest`, `ez::longest`, `ez::cw`, `ez::ccw`, or `ez::raw`.     
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!      
<Tabs
  groupId="pid_turn_rel_set1231235_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  // Turns to 90 deg
  chassis.pid_turn_relative_set(90_deg, 110, ez::shortest, true);
  chassis.pid_wait();

  // Turns the long way around to go -45 from where it started
  chassis.pid_turn_relative_set(-45_deg, 110, ez::longest, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_relative_set(okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);
```

</TabItem>
</Tabs>






























### pid_turn_exit_condition_set()
Sets the exit condition constants for turning. This uses the exit conditions from the PID class.  
 
`p_small_exit_time` time, in okapi units, before exiting `p_small_error`  
`p_small_error` small error threshold in okapi angle unit  
`p_big_exit_time` time, in okapi units, before exiting `p_big_error`  
`p_big_error` big error threshold, in okapi angle unit  
`p_velocity_exit_time` time, in okapi units, for velocity to be 0  
`p_mA_timeout` time, in okapi units, for `is_over_current` to be true   
`use_imu` boolean, true adds the IMU to velocity timeouts, false only uses the PID sensor.  This defaults to `true`     
<Tabs
  groupId="pid_turn_Exit_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_turn_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_turn_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QAngle p_small_error, okapi::QTime p_big_exit_time, okapi::QAngle p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, use_imu = true);
```

</TabItem>
</Tabs>




### pid_turn_chain_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for turns.    
 
`input` okapi angle unit      
<Tabs
  groupId="pid_turn_chain_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_turn_chain_constant_set(3_deg);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_turn_chain_constant_set(okapi::QAngle input);
```
</TabItem>
</Tabs>

















## Functions without Okapi Units
### pid_turn_set()
Sets the drive to turn using PID.  

`target` is in degrees.  
`speed` is 0 to 127.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work! 
<Tabs
  groupId="pid_turn_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  chassis.pid_turn_set(24, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(double target, int speed, bool slew_on = false);
```

</TabItem>
</Tabs>

### pid_turn_relative_set()
Sets the drive to turn using PID.  Target is relative here, the robot will add the target to your current angle.  

`target` is in degrees.  
`speed` is 0 to 127.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!
<Tabs
  groupId="pid_turn_rel_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  // Turns to 90 deg
  chassis.pid_turn_relative_set(90, 110, true);
  chassis.pid_wait();

  // Turns backwards by 45 degrees
  chassis.pid_turn_relative_set(-45, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_relative_set(double target, int speed, bool slew_on = false);
```

</TabItem>
</Tabs>







### pid_turn_exit_condition_set()
Sets the exit condition constants for turning. This uses the exit conditions from the PID class.  

This function can also be used without okapi units.  
`p_small_exit_time` time, in ms, before exiting `p_small_error`  
`p_small_error` small error threshold, assumed degrees  
`p_big_exit_time` time, in ms, before exiting `p_big_error`  
`p_big_error` big error threshold, assumed degrees  
`p_velocity_exit_time` time, in ms, for velocity to be 0  
`p_mA_timeout` time, in ms, for `is_over_current` to be true   
`use_imu` boolean, true adds the IMU to velocity timeouts, false only uses the PID sensor.  This defaults to `true`     
<Tabs
  groupId="pid_turn_exit_set_double"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_turn_exit_condition_set(300, 1, 500, 3, 750, 750);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, use_imu = true);
```

</TabItem>
</Tabs>










### pid_turn_constants_set()
Set PID drive constants for turns.   
 
`p` proportion constant  
`i` integral constant  
`d` derivative constant  
`p_start_i` error needs to be within this for i to start      
<Tabs
  groupId="pid_turn_constants_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_turn_constants_set(3, 0, 20);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_turn_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);
```


</TabItem>
</Tabs>


### pid_turn_chain_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for turns.    
 
`input` double, angle in degrees      
<Tabs
  groupId="pid_turn_chain_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_turn_chain_constant_set(3);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_turn_chain_constant_set(double input);
```
</TabItem>
</Tabs>





### pid_turn_min_set()
Sets the max power of the drive when the robot is within `start_i`.  This only enables when `i` is enabled, and when the movement is greater then `start_i`.        

`min` the minimum speed the robot will turn at when integral is being used
<Tabs
  groupId="examples12"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  chassis.pid_turn_min_set(30);

  chassis.pid_turn_set(45, 110);
  chassis.pid_wait();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_turn_min_set(int min);
```


</TabItem>
</Tabs>


### pid_turn_behavior_set()
Changes the default behavior for turning.   

`behavior` the default behavior for turns.  This can be `ez::shortest`, `ez::longest`, `ez::cw`, `ez::ccw`, or `ez::raw`.     
<Tabs
  groupId="pid_turn_behavior_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  chassis.pid_turn_behavior_set(ez::longest);  // Set the robot to take the longest path there

  // This will make the robot go the long way around to get to 90 degrees
  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_turn_behavior_set(e_angle_behavior behavior);
```
</TabItem>
</Tabs>











## Getter


### pid_turn_min_get()
Returns the minimum power the robot will turn at while integral is enabled.         
<Tabs
  groupId="examples17"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_turn_min_set(30);

  printf("Turn Min: %i", chassis.pid_turn_min_get());
}
```


</TabItem>


<TabItem value="proto">

```cpp
int pid_turn_min_get();
```


</TabItem>
</Tabs>



### pid_turn_chain_constant_get()
Returns a double that's the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for turns.         
<Tabs
  groupId="pid_turn_chain_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_turn_chain_constant_set(3_deg);
  printf("%.2f\n", chassis.pid_turn_chain_constant_get());
}
```
</TabItem>
<TabItem value="proto">

```cpp
double pid_turn_chain_constant_get();
```
</TabItem>
</Tabs>









































### pid_turn_behavior_get()
Returns the default behavior for turning.  This can be `ez::shortest`, `ez::longest`, `ez::cw`, `ez::ccw`, or `ez::raw`.    
<Tabs
  groupId="pid_turn_behavior_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  chassis.pid_turn_behavior_set(ez::longest);  // Set the robot to take the longest path there

  if (chassis.pid_turn_behavior_get() == ez::longest) {
    printf("Current behavior is ez::longest!\n");
  } else {
    printf("Current behavior is not ez::longest!\n");
  }

  // This will make the robot go the long way around to get to 90 degrees
  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
e_angle_behavior pid_turn_behavior_get();
```
</TabItem>
</Tabs>