---
layout: default
title: Turning
description:  Turning in place
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';



























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
Set's constants for turn exit conditions.  
 
`p_small_exit_time` time to exit when within smalL_error, okapi unit     
`p_small_error` small timer will start when error is within this, okapi unit     
`p_big_exit_time` time to exit when within big_error, okapi unit             
`p_big_error` big timer will start when error is within this, okapi unit        
`p_velocity_exit_time` velocity timer will start when velocity is 0, okapi unit   
`p_mA_timeout` mA timer will start when the motors are pulling too much current, okapi unit      
`use_imu` true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't         
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
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.  

This sets turning constants.     
 
`input` okapi angle unit      
<Tabs
  groupId="pid_turn_chain_constant_set_oka"
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












### slew_turn_constants_set()
Sets constants for slew for turns.

Slew ramps up the speed of the robot until the set distance is traveled.

`distance` the distance the robot travels before reaching max speed, an okapi angle unit   
`min_speed` the starting speed for the movement, 0 - 127   
<Tabs
  groupId="slew_turn_constant_set"
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

  chassis.slew_turn_constants_set(5_deg, 50);

  chassis.pid_turn_set(-90_deg, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_turn_constants_set(okapi::QAngle distance, int min_speed);
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
Set's constants for turn exit conditions.  

`p_small_exit_time` time to exit when within smalL_error, in ms  
`p_small_error` small timer will start when error is within this, in degrees
`p_big_exit_time` time to exit when within big_error, in ms
`p_big_error` big timer will start when error is within this, in degrees
`p_velocity_exit_time`  velocity timer will start when velocity is 0, in ms
`p_mA_timeout` mA timer will start when the motors are pulling too much current, in ms   
`use_imu` true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't    
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
 
`p` proportional term   
`i` integral term  
`d` derivative term   
`p_start_i` error threshold to start integral  
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
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.  

This sets turning constants.   
 
`input` angle in degrees      
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




### slew_turn_set()
Sets the default slew for turn motions, can be overwritten in movement functions.        
 
`slew_on` true enables, false disables  
<Tabs
  groupId="slew_turn_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void slew_turn_set(bool slew_on);
```
</TabItem>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  // Set the default slew state to true
  chassis.slew_turn_set(true);

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_turn_set(90_deg, 90, false);
  chassis.pid_wait();

  // This will slew because it's the default state
  chassis.pid_drive_set(0_deg, 90);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>





### pid_turn_min_set()
The minimum power for turns when kI and startI are enabled.        

`min` new clipped speed
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
Sets the default behavior for turns in turning movements.   

`behavior` ez::shortest, ez::longest, ez::left, ez::right, ez::raw        
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



### pid_turn_chain_constant_get()
Returns the amount that the PID will overshoot target by to maintain momentum into the next motion for turning.            
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









### slew_turn_get()
Returns true if slew is enabled for all turn motions, false otherwise.        
<Tabs
  groupId="slew_turn_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
bool slew_turn_get();
```
</TabItem>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  // Set the default slew state to true
  chassis.slew_turn_set(true);
  if (chassis.slew_turn_get())
    printf("Turn Slew is Enabled!\n");

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_turn_set(90_deg, 90, false);
  chassis.pid_wait();

  // This will slew because it's the default state
  chassis.pid_drive_set(0_deg, 90);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>


### pid_turn_min_get()
Returns minimum power for turns when kI and startI are enabled.         
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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

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























### pid_turn_behavior_get()
Returns the turn behavior for turns.   
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















 



