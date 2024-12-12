---
layout: default
title: Swinging
description:  Turns but only using one side of the drive
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


## Functions with Okapi Units

### pid_swing_set()
Sets the robot to swing using PID.  The robot will turn using one side of the drive, either the left or right.  The opposite side of the drive can be set to a value for wider or tighter arcs.  

`type` is either `ez::LEFT_SWING` or `ez::RIGHT_SWING`.  
`p_target` is an okapi angle unit.  
`speed` is 0 to 127.        
`opposite_speed` is -127 to 127.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="pid_swing_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_swing_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed = 0, bool slew_on = false);
```

</TabItem>


<TabItem value="example">

```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, 110, 0, true);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 110, 50, true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>







### pid_swing_relative_set()
Sets the robot to swing using PID.  The robot will turn using one side of the drive, either the left or right.  The opposite side of the drive can be set to a value for wider or tighter arcs.  Target is relative here, the robot will add the target to your current angle.  

`type` is either `ez::LEFT_SWING` or `ez::RIGHT_SWING`.  
`p_target` is an okapi angle unit.  
`speed` is 0 to 127.        
`opposite_speed` is -127 to 127.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="pid_swing_rel_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_swing_relative_set(e_swing type, okapi::QAngle p_target, int speed, int opposite_speed = 0, bool slew_on = false);

```

</TabItem>


<TabItem value="example">

```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  // This will turn to 90 degrees
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 90_deg, 110);
  chassis.pid_wait();

  // This will go backwards by 45 degrees
  chassis.pid_swing_relative_set(ez::RIGHT_SWING, -45_deg, 110);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>

### pid_swing_exit_condition_set()
Sets the exit condition constants for swing turns. This uses the exit conditions from the PID class.  

This function can also be used without okapi units.  
`p_small_exit_time` time, in ms, before exiting `p_small_error`  
`p_small_error` small error threshold, assumed degrees  
`p_big_exit_time` time, in ms, before exiting `p_big_error`  
`p_big_error` big error threshold, assumed degrees  
`p_velocity_exit_time` time, in ms, for velocity to be 0  
`p_mA_timeout` time, in ms, for `is_over_current` to be true   
`use_imu` boolean, true adds the IMU to velocity timeouts, false only uses the PID sensor.  This defaults to `true`     
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
Sets the exit condition constants for swing turns. This uses the exit conditions from the PID class.  
 
`p_small_exit_time` time, in okapi units, before exiting `p_small_error`  
`p_small_error` small error threshold in okapi angle unit  
`p_big_exit_time` time, in okapi units, before exiting `p_big_error`  
`p_big_error` big error threshold, in okapi angle unit  
`p_velocity_exit_time` time, in okapi units, for velocity to be 0  
`p_mA_timeout` time, in okapi units, for `is_over_current` to be true       
`use_imu` boolean, true adds the IMU to velocity timeouts, false only uses the PID sensor.  This defaults to `true`     
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
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for swings.    
 
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
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for forward swings.    
 
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
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for backward swings.    
 
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







## Functions without Okapi Units
### pid_swing_set()
Sets the robot to swing using PID.  The robot will turn using one side of the drive, either the left or right.  The opposite side of the drive can be set to a value for wider or tighter arcs.  

`type` is either `ez::LEFT_SWING` or `ez::RIGHT_SWING`.  
`target` is in degrees.  
`speed` is 0 to 127.        
`opposite_speed` is -127 to 127.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="pid_swing_set_double"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_swing_set(e_swing type, double target, int speed, int opposite_speed = 0, bool slew_on = false);
```

</TabItem>


<TabItem value="example">

```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_swing_set(ez::LEFT_SWING, 45, 110, 0, true);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0, 110, 50, true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>


### pid_swing_relative_set()
Sets the robot to swing using PID.  The robot will turn using one side of the drive, either the left or right.  The opposite side of the drive can be set to a value for wider or tighter arcs.  Target is relative here, the robot will add the target to your current angle.  

`type` is either `ez::LEFT_SWING` or `ez::RIGHT_SWING`.  
`p_target` is in degrees.  
`speed` is 0 to 127.        
`opposite_speed` is -127 to 127.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="pid_swing_relative_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_swing_relative_set(e_swing type, double target, int speed, int opposite_speed = 0, bool slew_on = false);
```

</TabItem>


<TabItem value="example">

```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  // This will turn to 90 degrees
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 90, 110);
  chassis.pid_wait();

  // This will go backwards by 45 degrees
  chassis.pid_swing_relative_set(ez::RIGHT_SWING, -45, 110);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>




### pid_swing_constants_set()
Set PID drive constants for forward and backward swings.   
 
`p` proportion constant  
`i` integral constant  
`d` derivative constant  
`p_start_i` error needs to be within this for i to start      
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
 
`p` proportion constant  
`i` integral constant  
`d` derivative constant  
`p_start_i` error needs to be within this for i to start      
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
 
`p` proportion constant  
`i` integral constant  
`d` derivative constant  
`p_start_i` error needs to be within this for i to start      
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
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for swings.    
 
`input` double, angle in degrees   
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
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for forward swings.    
 
`input` double, angle in degrees   
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
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for backward swings.    
 
`input` double, angle in degrees   
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


### pid_swing_min_set()
Sets the max power of the drive when the robot is within `start_i`.  This only enables when `i` is enabled, and when the movement is greater then `start_i`.  

`min` the minimum speed the robot will turn at when integral is being used
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
Changes the default behavior for swinging.   

`behavior` the default behavior for swings.  This can be `ez::shortest`, `ez::longest`, `ez::cw`, `ez::ccw`, or `ez::raw`.     
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


### pid_swing_min_get()
Returns the minimum power the robot will swing at while integral is enabled.         

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





### pid_swing_chain_forward_constant_get()
Returns a double that's the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for forward swings.      
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
Returns a double that's the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for backward swings.      
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





















### pid_swing_behavior_get()
Returns the default behavior for swinging.  This can be `ez::shortest`, `ez::longest`, `ez::cw`, `ez::ccw`, or `ez::raw`.    
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