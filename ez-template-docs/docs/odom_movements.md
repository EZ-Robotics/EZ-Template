---
layout: default
title: Movements
description:  ""
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';



## Functions with Okapi Units


### pid_odom_set()
Sets the robot to go forward/backward the distance you give it, but it uses odometry, without slew.     

`p_target` is in an okapi length unit.  
`speed` is 0 to 127.  It's recommended to keep this at 110.  
<Tabs
  groupId="erjk"
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

  chassis.odom_odom_set(24_in, 110);
  chassis.pid_wait();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_odom_set(okapi::QLength p_target, int speed);
```

</TabItem>
</Tabs>





### pid_odom_set()
Sets the robot to go forward/backward the distance you give it, but it uses odometry.  

`p_target` is in an okapi length unit.  
`speed` is 0 to 127.  It's recommended to keep this at 110.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="12342141234"
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

  chassis.odom_odom_set(24_in, 110, true);
  chassis.pid_wait();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_odom_set(okapi::QLength p_target, int speed, bool slew_on);
```

</TabItem>
</Tabs>














### pid_odom_set()
Go to an xy coordinate without slew.  This is a wrapper for `pid_odom_ptp_set()`.  

`p_imovement` united_pose, expecting `{0_in, 0_in}`
<Tabs
  groupId="as0d98f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_set(united_odom p_imovement);
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

  chassis.odom_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 

### pid_odom_set()
Go to an xy coordinate with slew.  This is a wrapper for `pid_odom_ptp_set()`.  

`p_imovement` united_pose, expecting `{0_in, 0_in}`
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0d9809asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_set(united_odom p_imovement, bool slew_on);
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

  chassis.odom_odom_set({{24_in, 24_in}, fwd, 110}, true);
  chassis.pid_wait();
}
```

</TabItem>


</Tabs>



### pid_odom_set()
Create and smooth out a path from the given points without slew.  The path will switch to boomerang is angle is specified for that point.  This is a wrapper for `pid_odom_smooth_pp_set`.  

`p_imovements` vector of united_pose
<Tabs
  groupId="as0d9123jsad8f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_set(std::vector{united_odom} p_imovements);
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

chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                      {{24_in, 24_in}, fwd, 110}});
chassis.pid_wait();
}
```

</TabItem>
</Tabs>

 



### pid_odom_set()
Create and smooth out a path from the given points.  The path will switch to boomerang is angle is specified for that point.  This is a wrapper for `pid_odom_smooth_pp_set`.  

`p_imovements` vector of united_pose
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0d9809z123f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_set(std::vector{united_odom} p_imovements, bool slew_on);
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

chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                      {{24_in, 24_in}, fwd, 110}},
                     true);
chassis.pid_wait();
}
```

</TabItem>
</Tabs>









 

 





### pid_odom_ptp_set()
Go to an xy coordinate without slew.   

`p_imovement` united_pose, expecting `{0_in, 0_in}`
<Tabs
  groupId="as0d98098asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_ptp_set(united_odom p_imovement);
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

  chassis.pid_odom_ptp_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 



### pid_odom_ptp_set()
Go to an xy coordinate with slew.  

`p_imovement` united_pose, expecting `{0_in, 0_in}`
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0d9kljhlkjhasd8f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_ptp_set(united_odom p_imovement, bool slew_on);
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

  chassis.pid_odom_ptp_set({{24_in, 24_in}, fwd, 110}, true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>


  
 



 

 






 
 






 
### pid_odom_pp_set()
Pure pursuits through all the points given.    

`p_imovements` vector of united_pose
<Tabs
  groupId="as0d98iuqec098asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_pp_set(std::vector{united_odom} p_imovements);
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

  chassis.pid_odom_pp_set({{{0_in, 24_in}, fwd, 110},
                           {{24_in, 24_in}, fwd, 110}});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 


### pid_odom_pp_set()
Pure pursuits through all the points given.    

`p_imovements` vector of united_pose   
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0d9klasdsadkjhjhlkjhaadasdsd8f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_pp_set(std::vector{united_odom} p_imovements, bool slew_on);
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

  chassis.pid_odom_pp_set({{{0_in, 24_in}, fwd, 110},
                           {{24_in, 24_in}, fwd, 110}}, 
                           true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>







 


 


### pid_odom_injected_pp_set()
Creates a new path that injects points between the input points, then pure pursuits along this new path.       

`p_imovements` vector of united_pose
<Tabs
  groupId="as0d9asdhh8iasdsadu97865qec098asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_injected_pp_set(std::vector{united_odom} p_imovements);
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

  chassis.pid_odom_injected_pp_set({{{0_in, 24_in}, fwd, 110},
                                    {{24_in, 24_in}, fwd, 110}});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>



### pid_odom_injected_pp_set()
Creates a new path that injects points between the input points, then pure pursuits along this new path.       

`p_imovements` vector of united_pose
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0d98iasdsadu9asdasdsadgf7865qec098asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_injected_pp_set(std::vector{united_odom} p_imovements, bool slew_on);
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

  chassis.pid_odom_injected_pp_set({{{0_in, 24_in}, fwd, 110},
                                    {{24_in, 24_in}, fwd, 110}}, 
                                    true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>











 




### pid_odom_smooth_pp_set()
Creates a new path that injects points between the input points, then smooths the corners of this path, then pure pursuits along this new path.       

`p_imovements` vector of united_pose
<Tabs
  groupId="as0d98iasdsadu9asdasdsadgf78a0918234sdads65qec098asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_smooth_pp_set(std::vector{united_odom} p_imovements);
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

  chassis.pid_odom_smooth_pp_set({{{0_in, 24_in}, fwd, 110},
                                  {{24_in, 24_in}, fwd, 110}});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 



 


### pid_odom_smooth_pp_set()
Creates a new path that injects points between the input points, then smooths the corners of this path, then pure pursuits along this new path.       

`p_imovements` vector of united_pose
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0d98iasdsadu9asdasdsadgf78asdads65qec098asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_smooth_pp_set(std::vector{united_odom} p_imovements, bool slew_on);
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

  chassis.pid_odom_smooth_pp_set({{{0_in, 24_in}, fwd, 110},
                                  {{24_in, 24_in}, fwd, 110}}, 
                                  true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>

 


 
 
 









 





### pid_odom_boomerang_set()
Goes to an xy coordinate at a set heading.  

`p_imovement` united_pose, expecting `{0_in, 0_in, 0_deg}`
<Tabs
  groupId="as0d098098fa98f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_boomerang_set(united_odom p_imovement);
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

  chassis.odom_odom_set({{24_in, 24_in, 0_deg}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 



### pid_odom_boomerang_set()
Goes to an xy coordinate at a set heading.  

`p_imovement` united_pose, expecting `{0_in, 0_in, 0_deg}`
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0asd190984d98f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_boomerang_set(united_odom p_imovement, bool slew_on);
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

  chassis.odom_odom_set({{24_in, 24_in, 0_deg}, fwd, 110}, true);
  chassis.pid_wait();
}
```

</TabItem>


</Tabs>








### pid_turn_set()
Sets the robot to turn face a point using PID and odometry.  

`p_itarget` `{x, y}` a target point to face.  this uses okapi units
`dir` face the point fwd or rev
`speed` 0 to 127, max speed during motion   
<Tabs
  groupId="pid_turn_set_oka0981253z215pasdasdasi"
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

  chassis.pid_turn_set({12_in, 12_in}, fwd, 110);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed);
```

</TabItem>
</Tabs>





### pid_turn_set()
Sets the robot to turn face a point using PID and odometry.  

`p_itarget` `{x, y}` a target point to face.  this uses okapi units
`dir` face the point fwd or rev
`speed` 0 to 127, max speed during motion   
`slew_on` ramp up from a lower speed to your target speed   
<Tabs
  groupId="p098721309587zid_turn_set_okapasdasdasi"
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

  chassis.pid_turn_set({12_in, 12_in}, fwd, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, bool slew_on);
```

</TabItem>
</Tabs>





### pid_turn_set()
Sets the robot to turn face a point using PID and odometry.  

`p_itarget` `{x, y}` a target point to face.  this uses okapi units
`dir` face the point fwd or rev
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
<Tabs
  groupId="pid_turn_set_okapasdasdas98721309587zi"
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

  chassis.pid_turn_set({12_in, 12_in}, fwd, 110, ez::longest);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, e_angle_behavior behavior);
```

</TabItem>
</Tabs>




### pid_turn_set()
Sets the robot to turn face a point using PID and odometry.  

`p_itarget` `{x, y}` a target point to face.  this uses okapi units
`dir` face the point fwd or rev
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
`slew_on` ramp up from a lower speed to your target speed   
<Tabs
  groupId="90872130945zpid_turn_set_okapasdasdasi"
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

  chassis.pid_turn_set({12_in, 12_in}, fwd, 110, ez::longest, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(united_pose p_itarget, drive_directions dir, int speed, e_angle_behavior behavior, bool slew_on);
```

</TabItem>
</Tabs>

























### pid_wait_until_point()
Lock the code in a while loop until this point has been passed.

`target` `{x, y}` pose with units for the robot to pass through before the while loop is released  
<Tabs
  groupId="pid_wait_until_oka_2134point_"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_wait_until_point(united_pose target);
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

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{12_in, 24_in}, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until_point({12_in, 24_in});  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>





### pid_wait_until()
Lock the code in a while loop until this point has been passed.

`target` `{x, y}` pose with units for the robot to pass through before the while loop is released  
<Tabs
  groupId="pid_wait_until_point_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_wait_until(united_pose target);
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

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{12_in, 24_in}, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until({12_in, 24_in});  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>















## Functions Without Okapi Units



### pid_odom_set()
Sets the robot to go forward/backward the distance you give it, but it uses odometry, without slew.      

`target` double, expecting inches
`speed` is 0 to 127.  It's recommended to keep this at 110.  
<Tabs
  groupId="98ysad"
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

  chassis.odom_odom_set(24, 110);
  chassis.pid_wait();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_odom_set(double target, int speed);
```

</TabItem>
</Tabs>





### pid_odom_set()
Sets the robot to go forward/backward the distance you give it, but it uses odometry.  

`target` double, expecting inches
`speed` is 0 to 127.  It's recommended to keep this at 110.  
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="kjh341"
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

  chassis.odom_odom_set(24, 110, true);
  chassis.pid_wait();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pid_odom_set(double target, int speed, bool slew_on);
```

</TabItem>
</Tabs>




### pid_odom_set()
Go to an xy coordinate without slew.  This is a wrapper for `pid_odom_ptp_set()`.  

`imovement` pose, expecting `{0, 0}`
<Tabs
  groupId="saldkjasdfdsaf"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_set(odom imovement);
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

  chassis.odom_odom_set({{24, 24}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 





### pid_odom_set()
Go to an xy coordinate with slew.  This is a wrapper for `pid_odom_ptp_set()`.  

`imovement` pose, expecting `{0, 0}`
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0d98ljlksajdhlkjf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_set(odom imovement, bool slew_on);
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

  chassis.odom_odom_set({{24, 24}, fwd, 110}, true);
  chassis.pid_wait();
}
```

</TabItem>


</Tabs>




### pid_odom_set()
Create and smooth out a path from the given points without slew.  The path will switch to boomerang is angle is specified for that point.  This is a wrapper for `pid_odom_smooth_pp_set`.    

`imovements` vector of united_pose
<Tabs
  groupId="as0sadfdsgd98f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_set(std::vector{odom} imovements)
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

chassis.pid_odom_set({{{0, 24}, fwd, 110},
                      {{24, 24}, fwd, 110}});
chassis.pid_wait();
}
```

</TabItem>
</Tabs>

 


### pid_odom_set()
Create and smooth out a path from the given points.  The path will switch to boomerang is angle is specified for that point.  This is a wrapper for `pid_odom_smooth_pp_set`.  

`imovements` vector of pose
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0d98kaljhflkjhf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_set(std::vector{odom} imovements, bool slew_on);
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

chassis.pid_odom_set({{{0, 24}, fwd, 110},
                      {{24, 24}, fwd, 110}},
                     true);
chassis.pid_wait();
}
```

</TabItem>
</Tabs>






### pid_odom_ptp_set()
Go to an xy coordinate without slew.   

`imovement` pose, expecting `{0, 0}`
<Tabs
  groupId="as0d9809adsasd8asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_ptp_set(odom imovement);
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

  chassis.pid_odom_ptp_set({{24, 24}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 




### pid_odom_ptp_set()
Go to an xy coordinate with slew.  

`imovement` united_pose, expecting `{0_in, 0_in}`
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0d9kljhlkjhasasdasdd8f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_ptp_set(odom imovement, bool slew_on);
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

  chassis.pid_odom_ptp_set({{24, 24}, fwd, 110}, true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>







### pid_odom_pp_set()
Pure pursuits through all the points given.    

`imovements` vector of pose
<Tabs
  groupId="as0d98iu97865qec098asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_pp_set(std::vector{odom} imovements);
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

  chassis.pid_odom_pp_set({{{0, 24}, fwd, 110},
                           {{24, 24}, fwd, 110}});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>




### pid_odom_pp_set()
Pure pursuits through all the points given with slew.    

`imovements` vector of united_pose   
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0d9klasdsadkjhkgrkjejhlkjhaadasdsd8f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_pp_set(std::vector{odom} imovements, bool slew_on);
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

  chassis.pid_odom_pp_set({{{0, 24}, fwd, 110},
                           {{24, 24}, fwd, 110}}, 
                           true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>








### pid_odom_injected_pp_set()
Creates a new path that injects points between the input points, then pure pursuits along this new path.       

`imovements` vector of pose
<Tabs
  groupId="as0d98iasdsadu91231231237865qec098asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_injected_pp_set(std::vector{odom} imovements);
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

  chassis.pid_odom_injected_pp_set({{{0, 24}, fwd, 110},
                                    {{24, 24}, fwd, 110}});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>



### pid_odom_injected_pp_set()
Creates a new path that injects points between the input points, then pure pursuits along this new path.       

`imovements` vector of pose
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0d98iasdsadu97860809ads5qec098asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_injected_pp_set(std::vector{odom} imovements, bool slew_on);
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

  chassis.pid_odom_injected_pp_set({{{0, 24}, fwd, 110},
                                    {{24, 24}, fwd, 110}}, 
                                    true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>














### pid_odom_smooth_pp_set()
Creates a new path that injects points between the input points, then smooths the corners of this path, then pure pursuits along this new path.       

`imovements` vector of pose
<Tabs
  groupId="as0d98iasdsadu9asdasdsadgf78asasdas12dads65qec098asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_smooth_pp_set(std::vector{odom} imovements);
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

  chassis.pid_odom_smooth_pp_set({{{0, 24}, fwd, 110},
                                  {{24, 24}, fwd, 110}});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 



### pid_odom_smooth_pp_set()
Creates a new path that injects points between the input points, then smooths the corners of this path, then pure pursuits along this new path.       

`imovements` vector of pose
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!   
<Tabs
  groupId="as0d98iasdsadu9asdasdsadgf78asasdas12dads65qec098asdf0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_smooth_pp_set(std::vector{odom} imovements, bool slew_on);
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

  chassis.pid_odom_smooth_pp_set({{{0, 24}, fwd, 110},
                                  {{24, 24}, fwd, 110}}, 
                                  true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>







### pid_odom_boomerang_set()
Goes to an xy coordinate at a set heading.  

`imovement` united_pose, expecting `{0, 0, 0}`
<Tabs
  groupId="as0d9098af0988f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_boomerang_set(odom imovement);
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

  chassis.odom_odom_set({{24, 24, 0}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 



### pid_odom_boomerang_set()
Goes to an xy coordinate at a set heading.  

`imovement` united_pose, expecting `{0, 0, 0}`
`slew_on` increases the speed of the drive gradually.  You must set slew constants for this to work!  
<Tabs
  groupId="as0asd191j23h0984d98f0f"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void pid_odom_boomerang_set(odom imovement, bool slew_on);
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

  chassis.odom_odom_set({{24, 24,  0}, fwd, 110}, true);
  chassis.pid_wait();
}
```

</TabItem>


</Tabs>












### pid_turn_set()
Sets the robot to turn face a point using PID and odometry.  

`p_itarget` `{x, y`} a target point to face
`dir` face the point fwd or rev
`speed` 0 to 127, max speed during motion   
<Tabs
  groupId="pid_turn_set098712039587z_okapasdasdasi"
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

  chassis.pid_turn_set({12, 12}, fwd, 110);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(pose itarget, drive_directions dir, int speed);
```

</TabItem>
</Tabs>





### pid_turn_set()
Sets the robot to turn face a point using PID and odometry.  

`p_itarget` `{x, y`} a target point to face
`dir` face the point fwd or rev
`speed` 0 to 127, max speed during motion   
`slew_on` ramp up from a lower speed to your target speed   
<Tabs
  groupId="pid_tu09872109587521zrn_set_okapasdasdasi"
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

  chassis.pid_turn_set({12, 12}, fwd, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(pose p_itarget, drive_directions dir, int speed, bool slew_on);
```

</TabItem>
</Tabs>





### pid_turn_set()
Sets the robot to turn face a point using PID and odometry.  

`p_itarget` `{x, y`} a target point to face
`dir` face the point fwd or rev
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
<Tabs
  groupId="pp59870941z45etid_turn_set_okapasdasdasi"
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

  chassis.pid_turn_set({12, 12}, fwd, 110, ez::longest);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(pose p_itarget, drive_directions dir, int speed, e_angle_behavior behavior);
```

</TabItem>
</Tabs>




### pid_turn_set()
Sets the robot to turn face a point using PID and odometry.  

`p_itarget` `{x, y`} a target point to face
`dir` face the point fwd or rev
`speed` 0 to 127, max speed during motion   
`behavior` changes what direction the robot will turn.  can be left, right, shortest, longest, raw   
`slew_on` ramp up from a lower speed to your target speed   
<Tabs
  groupId="pid_turnktar872145e_set_okapasdasdasi"
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

  chassis.pid_turn_set({12, 12}, fwd, 110, ez::longest, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void pid_turn_set(pose p_itarget, drive_directions dir, int speed, e_angle_behavior behavior, bool slew_on);
```

</TabItem>
</Tabs>





























































### pid_wait_until_index()
Lock the code in a while loop until this point has been passed.  

`index` index of your input points, 0 is the first point in the index   
<Tabs
  groupId="pid_wait_until_index"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_wait_until_index(int index);
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

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{12_in, 24_in}, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>






### pid_wait_until_index_started()
Lock the code in a while loop until this point becomes the target. 

`index` index of your input points, 0 is the first point in the index   
<Tabs
  groupId="pid_wait_until_index_started"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_wait_until_index_started(int index);
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

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{12_in, 24_in}, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until_index_started(1);  // Waits until the target becomes 12, 24
  Intake.move(127);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>





### pid_wait_until_point()
Lock the code in a while loop until this point has been passed.

`target` `{x, y}` pose for the robot to pass through before the while loop is released  
<Tabs
  groupId="pid_wait_until_point_no_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_wait_until_point(pose target);
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

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{12_in, 24_in}, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until_point({12, 24});  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>










### pid_wait_until()
Lock the code in a while loop until this point has been passed.

`target` `{x, y}` pose for the robot to pass through before the while loop is released  
<Tabs
  groupId="pid_wait_until_no_oka_pose"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_wait_until(pose target);
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

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{12_in, 24_in}, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until({12, 24});  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>











































