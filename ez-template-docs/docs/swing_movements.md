---
layout: default
title: Swinging
description:  Turns but only using one side of the drive
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


## Functions with Okapi Units

### pid_swing_set()
Sets the robot to turn using only one side of the drive with PID relative to initial heading.  

`type` L_SWING or R_SWING  
`p_target` target value okapi unit  
`speed` 0 to 127, max speed during motion   
<Tabs
  groupId="pid_turn_0a9s09sd09d098123zk1k2jhset_okapi"
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

  chassis.pid_swing_set(ez::LEFT_SWING, 22.5, 110);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed);
```

</TabItem>
</Tabs>






### pid_swing_set()
Sets the robot to turn using only one side of the drive with PID relative to initial heading.  

`type` L_SWING or R_SWING  
`p_target` target value okapi unit  
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
<Tabs
  groupId="pi12kj3hkcd872913z_turn_s098sfa8et_okapi"
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

  chassis.pid_swing_set(ez::LEFT_SWING, 22.5, 110, ez::longest);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior);
```

</TabItem>
</Tabs>







### pid_swing_set()
Sets the robot to turn using only one side of the drive with PID relative to initial heading.  

`type` L_SWING or R_SWING  
`p_target` target value okapi unit  
`speed` 0 to 127, max speed during motion   
`slew_on` ramp up from a lower speed to your target speed   
<Tabs
  groupId="pid_tur1234jkh134n_908zdset_okapi"
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

  chassis.pid_swing_set(ez::LEFT_SWING, 22.5, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, bool slew_on);
```

</TabItem>
</Tabs>





### pid_swing_set()
Sets the robot to turn using only one side of the drive with PID relative to initial heading.  

`type` L_SWING or R_SWING  
`p_target` target value okapi unit  
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
`slew_on` ramp up from a lower speed to your target speed   
<Tabs
  groupId="pid_tkjh123gurn_set_okapa09123zsdasdasi"
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

  chassis.pid_swing_set(ez::LEFT_SWING, 22.5, 110, ez::longest, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);
```

</TabItem>
</Tabs>






































### pid_swing_relative_set()
Sets the robot to turn using only one side of the drive with PID relative to the current heading.  

`type` L_SWING or R_SWING  
`p_target` target value okapi unit  
`speed` 0 to 127, max speed during motion   
<Tabs
  groupId="pid123z4_turn_re098asd12l_set_o098zx12kapi"
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

  // Swing to 90 deg
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 90, 110);
  chassis.pid_wait();

  // Swing the long way around to go -45 from where it started
  chassis.pid_swing_relative_set(ez::RIGHT_SWING, -45, 110);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed);
```

</TabItem>
</Tabs>








### pid_swing_relative_set()
Sets the robot to turn using only one side of the drive with PID relative to the current heading.  

`type` L_SWING or R_SWING  
`p_target` target value okapi unit  
`speed` 0 to 127, max speed during motion   
`slew_on` ramp up from a lower speed to your target speed   
<Tabs
  groupId="pid_turn_1lj7z123235drel_09zx12set_okapi"
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

  // Swing to 90 deg
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 90, 110, true);
  chassis.pid_wait();

  // Swing the long way around to go -45 from where it started
  chassis.pid_swing_relative_set(ez::RIGHT_SWING, -45, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, bool slew_on);
```

</TabItem>
</Tabs>








### pid_swing_relative_set()
Sets the robot to turn using only one side of the drive with PID relative to the current heading.  

`type` L_SWING or R_SWING  
`p_target` target value okapi unit  
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
<Tabs
  groupId="pid_tu09aarn_rel0091z29zx12_set_okapi"
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

  // Swing to 90 deg
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 90, 110, ez::shortest);
  chassis.pid_wait();

  // Swing the long way around to go -45 from where it started
  chassis.pid_swing_relative_set(ez::RIGHT_SWING, -45, 110, ez::longest);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior);
```

</TabItem>
</Tabs>




### pid_swing_relative_set()
Sets the robot to turn using only one side of the drive with PID relative to the current heading.  

`type` L_SWING or R_SWING  
`p_target` target value okapi unit  
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
`slew_on` ramp up from a lower speed to your target speed     
<Tabs
  groupId="pid_t123surn_rel_se182735ct1231235_okapi"
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

  // Swing to 90 deg
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 90, 110, ez::shortest, true);
  chassis.pid_wait();

  // Swing the long way around to go -45 from where it started
  chassis.pid_swing_relative_set(ez::RIGHT_SWING, -45, 110, ez::longest, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, e_angle_behavior behavior, bool slew_on);
```

</TabItem>
</Tabs>















### pid_swing_exit_condition_set()
Set's constants for swing exit conditions.  

`p_small_exit_time` time to exit when within smalL_error, in ms  
`p_small_error` small timer will start when error is within this, in degrees
`p_big_exit_time` time to exit when within big_error, in ms
`p_big_error` big timer will start when error is within this, in degrees
`p_velocity_exit_time`  velocity timer will start when velocity is 0, in ms
`p_mA_timeout` mA timer will start when the motors are pulling too much current, in ms   
`use_imu` true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't    
<Tabs
  groupId="pid_swing_exit_set_double"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_swing_exit_condition_set(300, 1, 500, 3, 750, 750);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_swing_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, use_imu = true);
```

</TabItem>
</Tabs>


### pid_swing_exit_condition_set()
Set's constants for swing exit conditions.  
 
`p_small_exit_time` time to exit when within smalL_error, okapi unit     
`p_small_error` small timer will start when error is within this, okapi unit     
`p_big_exit_time` time to exit when within big_error, okapi unit             
`p_big_error` big timer will start when error is within this, okapi unit        
`p_velocity_exit_time` velocity timer will start when velocity is 0, okapi unit   
`p_mA_timeout` mA timer will start when the motors are pulling too much current, okapi unit      
`use_imu` true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't         
<Tabs
  groupId="pid_swing_Exit_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_swing_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_swing_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QAngle p_small_error, okapi::QTime p_big_exit_time, okapi::QAngle p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, use_imu = true);
```

</TabItem>
</Tabs>





### pid_swing_chain_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.  

This sets forward and backwards swing constants.     
 
`input` okapi angle unit      
<Tabs
  groupId="pid_swing_chain_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_swing_chain_constant_set(5_deg);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_swing_chain_constant_set(okapi::QAngle input);
```
</TabItem>
</Tabs>

### pid_swing_chain_forward_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.   

This only sets forward swing constants.      
 
`input` okapi angle unit      
<Tabs
  groupId="pid_swing_chain_forward_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_swing_chain_forward_constant_set(5_deg);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_swing_chain_forward_constant_set(okapi::QAngle input);
```
</TabItem>
</Tabs>


### pid_swing_chain_backward_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.   

This only sets backward swing constants.     
 
`input` okapi angle unit      
<Tabs
  groupId="pid_swing_chain_forward_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_swing_chain_backward_constant_set(5_deg);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_swing_chain_backward_constant_set(okapi::QAngle input);
```
</TabItem>
</Tabs>








### slew_swing_constants_set()
Sets constants for slew for swing movements.   

Slew ramps up the speed of the robot until the set distance is traveled.   

`distance` the distance the robot travels before reaching max speed, an okapi distance unit  
`min_speed` the starting speed for the movement, 0 - 127  
<Tabs
  groupId="slew_swing_constants_set_okapi_distance"
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

  chassis.slew_swing_constants_set(5_in, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>






### slew_swing_constants_forward_set()
Sets constants for slew for forward swing movements.   

Slew ramps up the speed of the robot until the set distance is traveled.   

`distance` the distance the robot travels before reaching max speed, an okapi distance unit  
`min_speed` the starting speed for the movement, 0 - 127  
<Tabs
  groupId="slew_forward_swing_constants_set_okapi_distance"
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

  chassis.slew_swing_constants_forward_set(5_in, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_forward_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>





### slew_swing_constants_backward_set()
Sets constants for slew for backward swing movements.   

Slew ramps up the speed of the robot until the set distance is traveled.   

`distance` the distance the robot travels before reaching max speed, an okapi distance unit  
`min_speed` the starting speed for the movement, 0 - 127  
<Tabs
  groupId="slew_backward_swing_constants_set_okapi_distance"
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

  chassis.slew_swing_constants_backward_set(5_in, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_backward_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>





### slew_swing_constants_set()
Sets constants for slew for swing movements.   

Slew ramps up the speed of the robot until the set distance is traveled.   

`distance` the distance the robot travels before reaching max speed, an okapi angle unit  
`min_speed` the starting speed for the movement, 0 - 127  
<Tabs
  groupId="slew_swing_constants_set_okapi_angle"
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

  chassis.slew_swing_constants_set(7_deg, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_set(okapi::QAngle distance, int min_speed);
```

</TabItem>
</Tabs>

### slew_swing_constants_forward_set()
Sets constants for slew for forward swing movements.   

Slew ramps up the speed of the robot until the set distance is traveled.   

`distance` the distance the robot travels before reaching max speed, an okapi angle unit  
`min_speed` the starting speed for the movement, 0 - 127  
<Tabs
  groupId="slew_forward_swing_constants_set_okapi_angle"
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

  chassis.slew_swing_constants_forward_set(7_deg, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_forward_set(okapi::QAngle distance, int min_speed);
```

</TabItem>
</Tabs>

### slew_swing_constants_backward_set()
Sets constants for slew for backward swing movements.   

Slew ramps up the speed of the robot until the set distance is traveled.   

`distance` the distance the robot travels before reaching max speed, an okapi angle unit  
`min_speed` the starting speed for the movement, 0 - 127  
<Tabs
  groupId="slew_bakward_swing_constants_set_okapi_angle"
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

  chassis.slew_swing_constants_backward_set(7_deg, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_backward_set(okapi::QAngle distance, int min_speed);
```

</TabItem>
</Tabs>






















## Functions without Okapi Units


### pid_swing_set()
Sets the robot to turn using only one side of the drive with PID relative to initial heading.  

`type` L_SWING or R_SWING  
`target` target value in degrees  
`speed` 0 to 127, max speed during motion   
<Tabs
  groupId="pid_turn_0a9s09sd09dk1k2jhset_okapi"
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

  chassis.pid_swing_set(ez::LEFT_SWING, 22.5, 110);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_set(e_swing type, double target, int speed);
```

</TabItem>
</Tabs>






### pid_swing_set()
Sets the robot to turn using only one side of the drive with PID relative to initial heading.  

`type` L_SWING or R_SWING  
`target` target value in degrees  
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
<Tabs
  groupId="pi12kj3hkcd_turn_s098sfa8et_okapi"
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

  chassis.pid_swing_set(ez::LEFT_SWING, 22.5, 110, ez::longest);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_set(e_swing type, double target, int speed, e_angle_behavior behavior);
```

</TabItem>
</Tabs>







### pid_swing_set()
Sets the robot to turn using only one side of the drive with PID relative to initial heading.  

`type` L_SWING or R_SWING  
`target` target value in degrees  
`speed` 0 to 127, max speed during motion   
`slew_on` ramp up from a lower speed to your target speed   
<Tabs
  groupId="pid_tur1234jkh134n_908zdset_okapi"
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

  chassis.pid_swing_set(ez::LEFT_SWING, 22.5, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_set(e_swing type, double target, int speed, bool slew_on);
```

</TabItem>
</Tabs>





### pid_swing_set()
Sets the robot to turn using only one side of the drive with PID relative to initial heading.  

`type` L_SWING or R_SWING  
`target` target value in degrees  
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
`slew_on` ramp up from a lower speed to your target speed   
<Tabs
  groupId="pid_tkjh123gurn_set_okapasdasdasi"
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

  chassis.pid_swing_set(ez::LEFT_SWING, 22.5, 110, ez::longest, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_set(e_swing type, double target, int speed, e_angle_behavior behavior, bool slew_on);
```

</TabItem>
</Tabs>






































### pid_swing_relative_set()
Sets the robot to turn using only one side of the drive with PID relative to the current heading.  

`type` L_SWING or R_SWING  
`target` target value in degrees  
`speed` 0 to 127, max speed during motion   
<Tabs
  groupId="pid_turn_re098asd12l_set_o098zx12kapi"
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

  // Swing to 90 deg
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 90, 110);
  chassis.pid_wait();

  // Swing the long way around to go -45 from where it started
  chassis.pid_swing_relative_set(ez::RIGHT_SWING, -45, 110);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_relative_set(e_swing type, double target, int speed);
```

</TabItem>
</Tabs>








### pid_swing_relative_set()
Sets the robot to turn using only one side of the drive with PID relative to the current heading.  

`type` L_SWING or R_SWING  
`target` target value in degrees  
`speed` 0 to 127, max speed during motion   
`slew_on` ramp up from a lower speed to your target speed   
<Tabs
  groupId="pid_turn_1lj235drel_09zx12set_okapi"
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

  // Swing to 90 deg
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 90, 110, true);
  chassis.pid_wait();

  // Swing the long way around to go -45 from where it started
  chassis.pid_swing_relative_set(ez::RIGHT_SWING, -45, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_relative_set(e_swing type, double target, int speed, bool slew_on);
```

</TabItem>
</Tabs>








### pid_swing_relative_set()
Sets the robot to turn using only one side of the drive with PID relative to the current heading.  

`type` L_SWING or R_SWING  
`target` target value in degrees  
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
<Tabs
  groupId="pid_tu09aarn_rel09zx12_set_okapi"
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

  // Swing to 90 deg
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 90, 110, ez::shortest);
  chassis.pid_wait();

  // Swing the long way around to go -45 from where it started
  chassis.pid_swing_relative_set(ez::RIGHT_SWING, -45, 110, ez::longest);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_relative_set(e_swing type, double target, int speed, e_angle_behavior behavior);
```

</TabItem>
</Tabs>




### pid_swing_relative_set()
Sets the robot to turn using only one side of the drive with PID relative to the current heading.  

`type` L_SWING or R_SWING  
`target` target value in degrees  
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
`slew_on` ramp up from a lower speed to your target speed     
<Tabs
  groupId="pid_turn_rel_se182735ct1231235_okapi"
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

  // Swing to 90 deg
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 90, 110, ez::shortest, true);
  chassis.pid_wait();

  // Swing the long way around to go -45 from where it started
  chassis.pid_swing_relative_set(ez::RIGHT_SWING, -45, 110, ez::longest, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_swing_relative_set(e_swing type, double target, int speed, e_angle_behavior behavior, bool slew_on);
```

</TabItem>
</Tabs>

























### pid_swing_constants_set()
Set PID drive constants for forward and backward swings.   
 
`p` proportional term   
`i` integral term  
`d` derivative term   
`p_start_i` error threshold to start integral  
<Tabs
  groupId="pid_swing_constants_Set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_swing_constants_set(5, 0, 30);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_swing_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);
```


</TabItem>
</Tabs>




### pid_swing_constants_forward_set()
Set PID drive constants for forward swings.   
 
`p` proportional term   
`i` integral term  
`d` derivative term   
`p_start_i` error threshold to start integral  
<Tabs
  groupId="pid_swing_constants_forward_Set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_swing_constants_forward_set(5, 0, 30);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_swing_constants_forward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);
```


</TabItem>
</Tabs>



### pid_swing_constants_backward_set()
Set PID drive constants for backward swings.   
 
`p` proportional term   
`i` integral term  
`d` derivative term   
`p_start_i` error threshold to start integral  
<Tabs
  groupId="pid_swing_constants_backward_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_swing_constants_backward_set(5, 0, 30);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_swing_constants_backward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);
```


</TabItem>
</Tabs>





### pid_swing_chain_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.  

This sets forward and backwards swing constants.    
 
`input` angle in degrees   
<Tabs
  groupId="pid_swing_chain_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_swing_chain_constant_set(5);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_swing_chain_constant_set(double input);
```
</TabItem>
</Tabs>


### pid_swing_chain_forward_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.   

This only sets forward swing constants.    
 
`input` angle in degrees   
<Tabs
  groupId="pid_swing_chain_forward_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_swing_chain_forward_constant_set(5);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_swing_chain_forward_constant_set(double input);
```
</TabItem>
</Tabs>


### pid_swing_chain_backward_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.   

This only sets backward swing constants.       
 
`input` angle in degrees   
<Tabs
  groupId="pid_swing_chain_backward_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_swing_chain_backward_constant_set(5);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_swing_chain_backward_constant_set(double input);
```
</TabItem>
</Tabs>




### slew_swing_set()
Sets the default slew for swing forward and backward motions, can be overwritten in movement functions.      
 
`slew_on` true enables, false disables  
<Tabs
  groupId="slew_swing_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void slew_swing_set(bool slew_on);
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
  chassis.slew_swing_set(true);

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, false);
  chassis.pid_wait();

  // This will slew because it's the default state
  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>





### slew_swing_forward_set()
Sets the default slew for swing forward motions, can be overwritten in movement functions.      
 
`slew_on` true enables, false disables  
<Tabs
  groupId="slew_swing_forward_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void slew_swing_forward_set(bool slew_on);
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
  chassis.slew_swing_forward_set(true);

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, false);
  chassis.pid_wait();

  // This will slew because that's the default forward state
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 110);
  chassis.pid_wait();

  // This will not slew because we haven't set the default backward state
  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>



### slew_swing_backward_set()
Sets the default slew for swing backward motions, can be overwritten in movement functions.      
 
`slew_on` true enables, false disables  
<Tabs
  groupId="slew_swing_backward_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void slew_swing_backward_set(bool slew_on);
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
  chassis.slew_swing_backward_set(true);

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, 110, false);
  chassis.pid_wait();

  // This will slew because that's the default backward state
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 110);
  chassis.pid_wait();

  // This will not slew because we haven't set the default forward state
  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>











### pid_swing_min_set()
Sets minimum power for swings when kI and startI are enabled.    

`min` new clipped speed
<Tabs
  groupId="examples11"
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

  chassis.pid_swing_min_set(30);

  chassis.pid_swing_set(45, 110);
  chassis.pid_wait();
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_swing_min_set(int min);
```


</TabItem>
</Tabs>




### pid_swing_behavior_set()
Sets the default behavior for turns in swinging movements.   

`behavior` ez::shortest, ez::longest, ez::left, ez::right, ez::raw     
<Tabs
  groupId="pid_swing_behavior_set"
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

  chassis.pid_swing_behavior_set(ez::longest);  // Set the robot to take the longest path there

  // This will make the robot go the long way around to get to 90 degrees
  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_swing_behavior_set(e_angle_behavior behavior);
```
</TabItem>
</Tabs>










## Getter

### slew_swing_forward_get()
Returns true if slew is enabled for all swing forward motions, false otherwise.       
<Tabs
  groupId="slew_swing_forward_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
bool slew_swing_forward_get();
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
  chassis.slew_swing_forward_set(true);
  if (chassis.slew_swing_forward_get()) 
    printf("Slew Forward is Enabled!\n");

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, false);
  chassis.pid_wait();

  // This will slew because that's the default forward state
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 110);
  chassis.pid_wait();

  // This will not slew because we haven't set the default backward state
  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>



### slew_swing_backward_get()
Returns true if slew is enabled for all swing backward motions, false otherwise.       
<Tabs
  groupId="slew_swing_backward_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
bool slew_swing_backward_get();
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
  chassis.slew_swing_backward_set(true);
  if (chassis.slew_swing_backward_get()) 
    printf("Slew Backward is Enabled!\n");

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, 110, false);
  chassis.pid_wait();

  // This will slew because that's the default backward state
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 110);
  chassis.pid_wait();

  // This will not slew because we haven't set the default forward state
  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>



### pid_swing_chain_forward_constant_get()
Returns the amount that the PID will overshoot target by to maintain momentum into the next motion for swinging forward.         
<Tabs
  groupId="pid_swing_chain_forward_constant_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_swing_chain_constant_set(5_deg);
  printf("%.2f\n", chassis.pid_swing_chain_forward_constant_get());
}
```
</TabItem>
<TabItem value="proto">

```cpp
double pid_swing_chain_forward_constant_get();
```
</TabItem>
</Tabs>


### pid_swing_chain_backward_constant_get()
Returns the amount that the PID will overshoot target by to maintain momentum into the next motion for swinging backward.        
<Tabs
  groupId="pid_swing_chain_backward_constant_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_swing_chain_constant_set(5_deg);
  printf("%.2f\n", chassis.pid_swing_chain_backward_constant_get());
}
```
</TabItem>
<TabItem value="proto">

```cpp
double pid_swing_chain_backward_constant_get();
```
</TabItem>
</Tabs>





### pid_swing_min_get()
Returns minimum power for swings when kI and startI are enabled.         
<Tabs
  groupId="examples16"
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

  chassis.pid_swing_min_set(30);

  printf("Swing Min: %i", chassis.pid_swing_min_get());
}
```

</TabItem>


<TabItem value="proto">

```cpp
int pid_swing_min_get();
```


</TabItem>
</Tabs>



























### pid_swing_behavior_get()
Returns the turn behavior for swings.    
<Tabs
  groupId="pid_swing_behavior_get"
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

  chassis.pid_swing_behavior_set(ez::longest);  // Set the robot to take the longest path there

  if (chassis.pid_swing_behavior_get() == ez::longest) {
    printf("Current behavior is ez::longest!\n");
  } else {
    printf("Current behavior is not ez::longest!\n");
  }

  // This will make the robot go the long way around to get to 90 degrees
  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
e_angle_behavior pid_swing_behavior_get();
```
</TabItem>
</Tabs>






















































