---
layout: default
title: Autonomous Functions
parent: Docs
description:  ""
nav_order: 4
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
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

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











### pid_turn_set()
Sets the drive to turn using PID.  

`p_target` is an okapi angle unit.  
`speed` is 0 to 127.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!
<Tabs
  groupId="pid_turn_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_turn_set(24_deg, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(okapi::QAngle p_target, int speed, bool slew_on = false);
```

</TabItem>
</Tabs>





### pid_turn_relative_set()
Sets the drive to turn using PID.  Target is relative here, the robot will add the target to your current angle.  

`p_target` is an okapi angle unit.  
`speed` is 0 to 127.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!
<Tabs
  groupId="pid_turn_rel_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

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
void pid_turn_relative_set(okapi::QAngle p_target, int speed, bool slew_on = false);
```

</TabItem>
</Tabs>






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





















### drive_angle_set()
Sets the angle of the robot.  This is useful when your robot is setup in at an unconventional angle and you want 0 to be when you're square with the field.         

`p_angle` an okapi angle unit, angle that the robot will think it's now facing.
<Tabs
  groupId="drive_angle_set_okapi"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void drive_angle_set(okapi::QAngle p_angle);
```


</TabItem>


<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset(); // Resets PID targets to 0
  chassis.drive_imu_reset(); // Reset gyro position to 0
  chassis.drive_sensor_reset(); // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  chassis.drive_angle_set(45_deg);

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();
}
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

### pid_wait_until()
Locks the code in place until the drive has passed the input parameter.  This uses the exit conditions from the PID class.  This only works for drive motions.             

`target` the distance the robot needs to travel before unlocking the code as an okapi length unit.     
<Tabs
  groupId="pid_wait_until_distance"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_drive_set(48_in, 110);
  chassis.pid_wait_until(24_in);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait();
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_wait_until(okapi::QLength target);
```


</TabItem>
</Tabs>



### pid_wait_until()
Locks the code in place until the drive has passed the input parameter.  This uses the exit conditions from the PID class.  This only works for turn and swing motions.             

`target` the distance the robot needs to travel before unlocking the code as an okapi angle unit.     
<Tabs
  groupId="pid_wait_until_angle"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait_until(45_deg);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait();
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_wait_until(okapi::QAngle target);
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
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

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
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

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
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

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


### drive_angle_set()
Sets the angle of the robot.  This is useful when your robot is setup in at an unconventional angle and you want 0 to be when you're square with the field.         

`angle` is in degrees, angle that the robot will think it's now facing.
<Tabs
  groupId="drive_angle_set_double"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void drive_angle_set(double angle);
```


</TabItem>


<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_targets_reset(); // Resets PID targets to 0
  chassis.drive_imu_reset(); // Reset gyro position to 0
  chassis.drive_sensor_reset(); // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  chassis.drive_angle_set(45);

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();
}
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



### pid_speed_max_set()
Sets the max speed of the drive.  

`speed` an integer between -127 and 127.     
<Tabs
  groupId="pid_speed_max_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_drive_set(48, 110);
  chassis.pid_wait_until(24);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait();
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_speed_max_set(int speed);
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






### drive_mode_set()
Sets the current mode of the drive.  

`p_mode` the current task running for the drive.  accepts `ez::DISABLE`, `ez::SWING`, `ez::TURN`, `ez::DRIVE`           
<Tabs
  groupId="examples13"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_drive_set(12, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.drive_mode_set(ez::DISABLE); // Disable drive

  chassis.drive_set(-127, -127); // Run drive motors myself
  pros::delay(2000);
  chassis.drive_set(0, 0);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void drive_mode_set(e_mode p_mode);
```


</TabItem>
</Tabs>




### pid_drive_toggle()
Enables/disables the drive from moving in autonomous.  This is useful for debugging and checking PID variables.     

`toggle` true enables the drive, false disables the drive     
<Tabs
  groupId="examples14"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_drive_set(12, DRIVE_SPEED);
  chassis.pid_wait();

  pid_drive_toggle(false); // Disable drive

  chassis.pid_drive_set(-12, DRIVE_SPEED);
  while (true) {
    printf(" Left Error: %f  Right Error: %f\n", chassis.leftPID.error, chassis.rightPID.error);
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_drive_toggle(bool toggle);
```


</TabItem>
</Tabs>








### pid_print_toggle()
Enables/disables the drive functions printing every drive motion.  This is useful when you're debugging something and don't want terminal cluttered.  

`toggle` true enables printing, false disables
<Tabs
  groupId="examples15"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_drive_set(12, DRIVE_SPEED); // This will print
  chassis.pid_wait(); // This will print

  pid_print_toggle(false); // Disable prints

  chassis.pid_drive_set(-12, DRIVE_SPEED); // This won't print
  chassis.pid_wait(); // This won't print
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_print_toggle(bool toggle);
```


</TabItem>
</Tabs>


### pid_wait_until()
Locks the code in place until the drive has passed the input parameter.  This uses the exit conditions from the PID class.           

`target` the distance the robot needs to travel before unlocking the code.  This is degrees if turning or swinging, and inches if driving.  
<Tabs
  groupId="pid_wait_until_double"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_drive_set(48, 110);
  chassis.pid_wait_until(24);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait();
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_wait_until(double target);
```


</TabItem>
</Tabs>

### pid_targets_reset()
Resets all drive PID targets to 0.       

<Tabs
  groupId="pid_targets_reset"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">


```cpp
void autonomous() {
  chassis.pid_targets_reset(); // Resets PID targets to 0
  chassis.drive_imu_reset(); // Reset gyro position to 0
  chassis.drive_sensor_reset(); // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.selected_auton_call(); // Calls selected auton from autonomous selector.
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_targets_reset();
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








### interfered
Boolean that returns true when `pid_wait()` or `pid_wait_until()` exit with velocity or is_over_current.  This can be used to detect unwanted motion and stop the drive motors from overheating during autonomous.     

<Tabs
  groupId="examples18"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
 void tug (int attempts) {
   for (int i=0; i<attempts-1; i++) {
     // Attempt to drive backwards
     printf("i - %i", i);
     chassis.pid_drive_set(-12, 127);
     chassis.pid_wait();

     // If failsafed...
     if (chassis.interfered) {
       chassis.drive_sensor_reset();
       chassis.pid_drive_set(-2, 20);
       pros::delay(1000);
     }
     // If robot successfully drove back, return
     else {
       return;
     }
   }
 }

void auto1() {
  chassis.pid_drive_set(24, 110, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90, 90);
  chassis.pid_wait();
}
```


</TabItem>


<TabItem value="proto">

```cpp
bool interfered = false;
```


</TabItem>
</Tabs>







### drive_mode_get()
Returns the current drive mode that the task is running.  Returns `ez::DISABLE`, `ez::SWING`, `ez::TURN`, `ez::DRIVE`.           

<Tabs
  groupId="examples19"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.pid_drive_set(12, DRIVE_SPEED);
  chassis.pid_wait();

  if (chassis.interfered)
    chassis.drive_mode_set(ez::DISABLE);
  
  if (chassis.drive_mode_get() == ez::DISABLE) {
    chassis.drive_set(-127, -127); // Run drive motors myself
    pros::delay(2000);
    chassis.drive_set(0, 0);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
e_mode drive_mode_get();
```


</TabItem>
</Tabs>













### drive_tick_per_inch()
Returns the conversion between raw sensor value and inches.
<Tabs
  groupId="examples20"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  printf("Tick Per Inch: %f\n", chassis.drive_tick_per_inch());
}
```

</TabItem>


<TabItem value="proto">

```cpp
double drive_tick_per_inch();
```


</TabItem>
</Tabs>








## Misc.

### pid_wait()
Locks the code in place until the drive has settled.  This uses the exit conditions from the PID class.      

<Tabs
  groupId="examples21"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_turn_set(90, 110);
  chassis.pid_wait();

  chassis.pid_turn_set(0, 110);
  chassis.pid_wait();
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_wait();
```


</TabItem>
</Tabs>










