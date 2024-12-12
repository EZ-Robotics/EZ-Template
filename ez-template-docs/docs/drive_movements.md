---
layout: default
title: Driving
description:  Going forward and backward
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';



## Functions with Okapi Units

### pid_drive_set()
Sets the drive to go forward using PID and heading correction.  

`p_target` is in an okapi length unit.  
`speed` is 0 to 127.  It's recommended to keep this at 110.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
`toggle_heading` will disable heading correction when false. 
<Tabs
  groupId="pid_drive_set_okapi"
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
void pid_drive_set(okapi::QLength p_target, int speed, bool slew_on = false, bool toggle_heading = true);
```

</TabItem>
</Tabs>
































































### pid_drive_exit_condition_set()
Sets the exit condition constants for driving. This uses the exit conditions from the PID class.  
 
`p_small_exit_time` time, in okapi units, before exiting `p_small_error`  
`p_small_error` small error threshold in okapi length unit  
`p_big_exit_time` time, in okapi units, before exiting `p_big_error`  
`p_big_error` big error threshold, in okapi length unit  
`p_velocity_exit_time` time, in okapi units, for velocity to be 0  
`p_mA_timeout` time, in okapi units, for `is_over_current` to be true   
`use_imu` boolean, true adds the IMU to velocity timeouts, false only uses the PID sensor.  This defaults to `true`     
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
void pid_drive_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QLength p_small_error, okapi::QTime p_big_exit_time, okapi::QLength p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, use_imu = true);
```

</TabItem>
</Tabs>


















### pid_drive_chain_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for drive motions.    
 
`input` okapi length unit       
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
void pid_drive_chain_constant_set(okapi::QLength input);
```
</TabItem>
</Tabs>


### pid_drive_chain_forward_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for forward drive motions.    
 
`input` okapi length unit    
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
void pid_drive_chain_forward_constant_set(okapi::QLength input);
```
</TabItem>
</Tabs>


### pid_drive_chain_backward_constant_set()
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for backward drive motions.    
 
`input` okapi length unit    
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
void pid_drive_chain_backward_constant_set(okapi::QLength input);
```
</TabItem>
</Tabs>









### slew_drive_constants_set()
Sets constants for slew for driving.     

Slew ramps up the speed of the robot until the set distance is traveled.     

`distance` the distance the robot travels before reaching max speed, an okapi distance unit   
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
void slew_drive_constants_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>




### slew_drive_constants_forward_set()
Sets constants for slew for driving forward.     

Slew ramps up the speed of the robot until the set distance is traveled.     

`distance` the distance the robot travels before reaching max speed, an okapi distance unit   
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
void slew_drive_constants_forward_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>




### slew_drive_constants_backward_set()
Sets constants for slew for driving backward.     

Slew ramps up the speed of the robot until the set distance is traveled.     

`distance` the distance the robot travels before reaching max speed, an okapi distance unit   
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
void slew_drive_constants_backward_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>














## Functions without Okapi Units

### pid_drive_set()
Sets the drive to go forward using PID and heading correction.  

`target` is in inches.  
`speed` is 0 to 127.  It's recommended to keep this at 110.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
`toggle_heading` will disable heading correction when false. 
<Tabs
  groupId="pid_drive_set"
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
void pid_drive_set(double target, int speed, bool slew_on = false, bool toggle_heading = true);
```

</TabItem>
</Tabs>












### pid_drive_exit_condition_set()
Sets the exit condition constants for driving. This uses the exit conditions from the PID class.  

This function can also be used without okapi units.  
`p_small_exit_time` time, in ms, before exiting `p_small_error`  
`p_small_error` small error threshold, assumed inches  
`p_big_exit_time` time, in ms, before exiting `p_big_error`  
`p_big_error` big error threshold, assumed inches  
`p_velocity_exit_time` time, in ms, for velocity to be 0  
`p_mA_timeout` time, in ms, for `is_over_current` to be true   
`use_imu` boolean, true adds the IMU to velocity timeouts, false only uses the PID sensor.  This defaults to `true`     
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
void pid_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, use_imu = true);
```

</TabItem>
</Tabs>
















### pid_drive_constants_set()
Set PID drive constants for forwards and backwards.   
 
`p` proportion constant  
`i` integral constant  
`d` derivative constant  
`p_start_i` error needs to be within this for i to start      
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
 
`p` proportion constant  
`i` integral constant  
`d` derivative constant  
`p_start_i` error needs to be within this for i to start      
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
 
`p` proportion constant  
`i` integral constant  
`d` derivative constant  
`p_start_i` error needs to be within this for i to start      
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
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for drive motions.    
 
`input` double, length in inches   
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
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for forward drive motions.    
 
`input` double, length in inches   
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
Sets the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for backward drive motions.    
 
`input` double, length in inches   
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
Sets the default slew for drive forwards motions, can be overwritten in movement functions.      
 
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











### pid_drive_chain_forward_constant_get()
Returns a double that's the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for forward drive motions.      
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
Returns a double that's the amount that the PID will overshoot target by to maintain momentum into the next motion when using `pid_wait_quick_chain()` for backward drive motions.      
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
Returns true if slew is enabled for all drive forward movements, false otherwise.    
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