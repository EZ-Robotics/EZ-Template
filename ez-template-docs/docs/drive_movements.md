---
layout: default
title: Driving
description:  moving forward and backward autonomously
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';



## Functions with Units



### pid_drive_set()
Sets the robot to move forward using PID with units, only using slew if globally enabled.  

`p_target` target, in units 
`speed` 0 to 127, max speed during motion   
<Tabs
  groupId="pid_driv09873214ze_set_okapi"
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

  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_drive_set(ez::QLength p_target, int speed);
```

</TabItem>
</Tabs>







### pid_drive_set()
Sets the robot to move forward using PID with units, using slew if enabled for this motion.  

`p_target` target, in units 
`speed` 0 to 127, max speed during motion  
`slew_on` ramp up from a lower speed to your target speed   
`toggle_heading` toggle for heading correction.  true enables, false disables   
<Tabs
  groupId="pid_dri987475zve_set_okapi"
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

  chassis.pid_drive_set(24_in, 110, true);
  chassis.pid_wait();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_drive_set(ez::QLength p_target, int speed, bool slew_on, bool toggle_heading = true);
```

</TabItem>
</Tabs>

































































### pid_drive_exit_condition_set()
Set's constants for drive exit conditions.     
 
`p_small_exit_time` time to exit when within small_error, in units     
`p_small_error` small timer will start when error is within this, in units     
`p_big_exit_time` time to exit when within big_error, in units             
`p_big_error` big timer will start when error is within this, in units        
`p_velocity_exit_time` time, in units, for velocity to be 0 after the robot has moved (or after 1 second if it never moves)          
`p_mA_timeout` mA timer will start when the first motor on the side(s) being driven is over its current limit, in units     
`use_imu` true adds a second velocity exit timer based on the imu's acceleration (exits if either the main sensor or the imu reports no movement for `p_velocity_exit_time`), false uses only the main sensor         
<Tabs
  groupId="pid_drive_Exit_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_drive_exit_condition_set(300_ms, 1_in, 500_ms, 3_in, 750_ms, 750_ms);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_drive_exit_condition_set(ez::QTime p_small_exit_time, ez::QLength p_small_error, ez::QTime p_big_exit_time, ez::QLength p_big_error, ez::QTime p_velocity_exit_time, ez::QTime p_mA_timeout, bool use_imu = true);
```

</TabItem>
</Tabs>


















### pid_drive_chain_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.   

This sets forward and backwards driving constants.      
 
`input` length unit       
<Tabs
  groupId="pid_drive_chain_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_drive_chain_constant_set(3_in);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_drive_chain_constant_set(ez::QLength input);
```
</TabItem>
</Tabs>


### pid_drive_chain_forward_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.   

This only sets forward driving constants.         
 
`input` length unit    
<Tabs
  groupId="pid_drive_chain_forward_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_drive_chain_forward_constant_set(3_in);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_drive_chain_forward_constant_set(ez::QLength input);
```
</TabItem>
</Tabs>


### pid_drive_chain_backward_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.   

This only sets backward driving constants.         
 
`input` length unit    
<Tabs
  groupId="pid_drive_chain_backward_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_drive_chain_backward_constant_set(3_in);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_drive_chain_backward_constant_set(ez::QLength input);
```
</TabItem>
</Tabs>









### slew_drive_constants_set()
Sets constants for slew for driving.     

Slew ramps up the speed of the robot from `min_speed` to full speed (127) over the set distance.  A motion with a lower max speed is capped at that speed, so it stops ramping sooner.     

`distance` the distance the robot travels to ramp up to full speed (127), a distance unit   
`min_speed` the starting speed for the movement, 0 - 127   
<Tabs
  groupId="slew_backward_constant_set"
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

  chassis.slew_drive_constants_set(5_in, 50);

  chassis.pid_drive_set(12_in, 110, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_drive_constants_set(ez::QLength distance, int min_speed);
```

</TabItem>
</Tabs>




### slew_drive_constants_forward_set()
Sets constants for slew for driving forward.     

Slew ramps up the speed of the robot from `min_speed` to full speed (127) over the set distance.  A motion with a lower max speed is capped at that speed, so it stops ramping sooner.     

`distance` the distance the robot travels to ramp up to full speed (127), a distance unit   
`min_speed` the starting speed for the movement, 0 - 127   
<Tabs
  groupId="slew_forward_constant_set"
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

  chassis.slew_drive_constants_forward_set(5_in, 50);

  chassis.pid_drive_set(12_in, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_drive_constants_forward_set(ez::QLength distance, int min_speed);
```

</TabItem>
</Tabs>




### slew_drive_constants_backward_set()
Sets constants for slew for driving backward.     

Slew ramps up the speed of the robot from `min_speed` to full speed (127) over the set distance.  A motion with a lower max speed is capped at that speed, so it stops ramping sooner.     

`distance` the distance the robot travels to ramp up to full speed (127), a distance unit   
`min_speed` the starting speed for the movement, 0 - 127   
<Tabs
  groupId="slew_backward_constant_set"
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

  chassis.slew_drive_constants_backward_set(5_in, 50);

  chassis.pid_drive_set(-12_in, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_drive_constants_backward_set(ez::QLength distance, int min_speed);
```

</TabItem>
</Tabs>














## Functions without Units



### pid_drive_set()
Sets the robot to move forward using PID without units, only using slew if globally enabled.  

`target` target in inches
`speed` 0 to 127, max speed during motion   
<Tabs
  groupId="pid_d98709321874z5rive_set"
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

  chassis.pid_drive_set(24, 110);
  chassis.pid_wait();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_drive_set(double target, int speed);
```

</TabItem>
</Tabs>





### pid_drive_set()
Sets the robot to move forward using PID without units, using slew if enabled for this motion.  

`target` target in inches
`speed` 0 to 127, max speed during motion  
`slew_on` ramp up from a lower speed to your target speed   
`toggle_heading` toggle for heading correction.  true enables, false disables   
<Tabs
  groupId="987093214zpid_drive_set"
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

  chassis.pid_drive_set(24, 110, true);
  chassis.pid_wait();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_drive_set(double target, int speed, bool slew_on, bool toggle_heading = true);
```

</TabItem>
</Tabs>



















### pid_drive_exit_condition_set()
Set's constants for drive exit conditions.   

`p_small_exit_time` time to exit when within small_error, in ms     
`p_small_error` small timer will start when error is within this, in inches     
`p_big_exit_time` time to exit when within big_error, in ms             
`p_big_error` big timer will start when error is within this, in inches        
`p_velocity_exit_time` velocity timer will start when velocity is 0 after the robot has moved (or after 1 second if it never moves), in ms   
`p_mA_timeout` mA timer will start when the first motor on the side(s) being driven is over its current limit, in ms      
`use_imu` true adds a second velocity exit timer based on the imu's acceleration (exits if either the main sensor or the imu reports no movement for `p_velocity_exit_time`), false uses only the main sensor         
<Tabs
  groupId="pid_drive_exit_set_double"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_drive_exit_condition_set(300, 1, 500, 3, 750, 750);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu = true);
```

</TabItem>
</Tabs>
















### pid_drive_constants_set()
Set PID drive constants for forwards and backwards.   
 
`p` proportional term   
`i` integral term  
`d` derivative term   
`p_start_i` error threshold to start integral    
<Tabs
  groupId="pid_drive_constants_Set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_drive_constants_set(10, 0, 100);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_drive_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);
```


</TabItem>
</Tabs>



### pid_drive_constants_forward_set()
Set PID drive constants for forwards movements.   
 
`p` proportional term   
`i` integral term  
`d` derivative term   
`p_start_i` error threshold to start integral  
<Tabs
  groupId="pid_drive_constants_forward_Set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_drive_constants_forward_set(10, 0, 100);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_drive_constants_forward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);
```


</TabItem>
</Tabs>



### pid_drive_constants_backward_set()
Set PID drive constants for backwards movements.   
 
`p` proportional term   
`i` integral term  
`d` derivative term   
`p_start_i` error threshold to start integral  
<Tabs
  groupId="pid_drive_constants_backward_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_drive_constants_backward_set(10, 0, 100);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_drive_constants_backward_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);
```


</TabItem>
</Tabs>



### pid_heading_constants_set()
Set PID drive constants heading correction during drive motions.   
 
`p` proportion constant  
`i` integral constant  
`d` derivative constant  
`p_start_i` error needs to be within this for i to start      
<Tabs
  groupId="pid_heading_constants_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void initialize() {
  chassis.pid_heading_constants_set(3, 0, 20);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_heading_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);
```


</TabItem>
</Tabs>














### pid_drive_chain_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.   

This sets forward and backwards driving constants.          
 
`input` length in inches   
<Tabs
  groupId="pid_drive_chain_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_drive_chain_constant_set(3);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_drive_chain_constant_set(double input);
```
</TabItem>
</Tabs>


### pid_drive_chain_forward_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.   

This only sets forward driving constants.     
 
`input` length in inches   
<Tabs
  groupId="pid_drive_chain_forward_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_drive_chain_forward_constant_set(3);
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_drive_chain_forward_constant_set(double input);
```
</TabItem>
</Tabs>


### pid_drive_chain_backward_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion.   

This only sets backward driving constants.        
 
`input` length in inches   
<Tabs
  groupId="pid_drive_chain_backward_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_drive_chain_backward_constant_set(double input);
```
</TabItem>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_drive_chain_backward_constant_set(3);
}
```
</TabItem>
</Tabs>




### slew_drive_set()
Sets the default slew for drive forwards and backwards motions, can be overwritten in movement functions.      
 
`slew_on` true enables, false disables  
<Tabs
  groupId="slew_drive_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void slew_drive_set(bool slew_on);
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
  chassis.slew_drive_set(true);

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_drive_set(24_in, 110, false);
  chassis.pid_wait();

  // This will slew because it's the default state
  chassis.pid_drive_set(-24_in, 110);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>



### slew_drive_forward_set()
Sets the default slew for drive forwards motions, can be overwritten in movement functions.      
 
`slew_on` true enables, false disables  
<Tabs
  groupId="slew_drive_forward_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void slew_drive_forward_set(bool slew_on);
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
  chassis.slew_drive_forward_set(true);

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_drive_set(24_in, 110, false);
  chassis.pid_wait();

  // This will slew because it's the default state going forward
  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait();

  // This will not slew because we haven't set a default state for going backward
  chassis.pid_drive_set(-48_in, 110);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>


### slew_drive_backward_set()
Sets the default slew for drive backward motions, can be overwritten in movement functions.      
 
`slew_on` true enables, false disables  
<Tabs
  groupId="slew_drive_backward_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void slew_drive_backward_set(bool slew_on);
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
  chassis.slew_drive_backward_set(true);

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_drive_set(-24_in, 110, false);
  chassis.pid_wait();

  // This will slew because it's the default state going backward
  chassis.pid_drive_set(-24_in, 110);
  chassis.pid_wait();

  // This will not slew because we haven't set a default state for going forward
  chassis.pid_drive_set(48_in, 110);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>



















## Getter

### pid_drive_constants_get()
Returns the PID constants for driving, as a `PID::Constants` with `kp`, `ki`, `kd` and `start_i`.  If the forward and backward constants were set to different values, this prints `Forward and Reverse constants are not the same!` and returns `{-1, -1, -1, -1}`.  Use the forward and backward getters when they differ.  
<Tabs
  groupId="pid_drive_constants_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
  ez::PID::Constants c = chassis.pid_drive_constants_get();
  printf("kp %.2f  ki %.2f  kd %.2f  start_i %.2f\n", c.kp, c.ki, c.kd, c.start_i);  // Prints kp 20.00  ki 0.00  kd 100.00  start_i 0.00
}
```

</TabItem>

<TabItem value="proto">

```cpp
PID::Constants pid_drive_constants_get();
```

</TabItem>
</Tabs>

### pid_drive_constants_forward_get()
Returns the PID constants for driving forward, as a `PID::Constants` with `kp`, `ki`, `kd` and `start_i`.  
<Tabs
  groupId="pid_drive_constants_forward_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_drive_constants_forward_set(20.0, 0.0, 100.0);
  ez::PID::Constants c = chassis.pid_drive_constants_forward_get();
  printf("kp %.2f  ki %.2f  kd %.2f  start_i %.2f\n", c.kp, c.ki, c.kd, c.start_i);  // Prints kp 20.00  ki 0.00  kd 100.00  start_i 0.00
}
```

</TabItem>

<TabItem value="proto">

```cpp
PID::Constants pid_drive_constants_forward_get();
```

</TabItem>
</Tabs>

### pid_drive_constants_backward_get()
Returns the PID constants for driving backward, as a `PID::Constants` with `kp`, `ki`, `kd` and `start_i`.  
<Tabs
  groupId="pid_drive_constants_backward_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_drive_constants_backward_set(20.0, 0.0, 100.0);
  ez::PID::Constants c = chassis.pid_drive_constants_backward_get();
  printf("kp %.2f  ki %.2f  kd %.2f  start_i %.2f\n", c.kp, c.ki, c.kd, c.start_i);  // Prints kp 20.00  ki 0.00  kd 100.00  start_i 0.00
}
```

</TabItem>

<TabItem value="proto">

```cpp
PID::Constants pid_drive_constants_backward_get();
```

</TabItem>
</Tabs>

### pid_heading_constants_get()
Returns the PID constants that correct the robot's heading during drive motions, as a `PID::Constants` with `kp`, `ki`, `kd` and `start_i`.  
<Tabs
  groupId="pid_heading_constants_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  ez::PID::Constants c = chassis.pid_heading_constants_get();
  printf("kp %.2f  ki %.2f  kd %.2f  start_i %.2f\n", c.kp, c.ki, c.kd, c.start_i);  // Prints kp 11.00  ki 0.00  kd 20.00  start_i 0.00
}
```

</TabItem>

<TabItem value="proto">

```cpp
PID::Constants pid_heading_constants_get();
```

</TabItem>
</Tabs>












### pid_drive_chain_forward_constant_get()
Returns the amount that the PID will overshoot target by to maintain momentum into the next motion for driving forward.         
<Tabs
  groupId="pid_drive_chain_forward_constant_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_drive_chain_constant_set(3_in);
  printf("%.2f\n", chassis.pid_drive_chain_forward_constant_get());
}
```
</TabItem>
<TabItem value="proto">

```cpp
double pid_drive_chain_forward_constant_get();
```
</TabItem>
</Tabs>


### pid_drive_chain_backward_constant_get()
Returns the amount that the PID will overshoot target by to maintain momentum into the next motion for driving backward.      
<Tabs
  groupId="pid_drive_chain_backward_constant_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_drive_chain_constant_set(3_in);
  printf("%.2f\n", chassis.pid_drive_chain_backward_constant_get());
}
```
</TabItem>
<TabItem value="proto">

```cpp
double pid_drive_chain_backward_constant_get();
```
</TabItem>
</Tabs>
























### slew_drive_forward_get()
Returns true if slew is enabled for all drive forward movements, false otherwise.    
<Tabs
  groupId="slew_drive_forward_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
bool slew_drive_forward_get();
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
  chassis.slew_drive_forward_set(true);
  if (chassis.slew_drive_forward_get())
    printf("Slew Forward is Enabled!\n");

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_drive_set(24_in, 110, false);
  chassis.pid_wait();

  // This will slew because it's the default state going forward
  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait();

  // This will not slew because we haven't set a default state for going backward
  chassis.pid_drive_set(-48_in, 110);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>




### slew_drive_backward_get()
Returns true if slew is enabled for all drive backward movements, false otherwise.    
<Tabs
  groupId="slew_drive_backward_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
bool slew_drive_backward_get();
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
  chassis.slew_drive_backward_set(true);
  if (chassis.slew_drive_backward_get())
    printf("Slew Backward is Enabled!\n");

  // This will not use slew because we explicitly told the robot to not slew
  chassis.pid_drive_set(-24_in, 110, false);
  chassis.pid_wait();

  // This will slew because it's the default state going backward
  chassis.pid_drive_set(-24_in, 110);
  chassis.pid_wait();

  // This will not slew because we haven't set a default state for going forward
  chassis.pid_drive_set(48_in, 110);
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>