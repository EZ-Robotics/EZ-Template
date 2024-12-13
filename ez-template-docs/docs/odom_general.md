---
layout: default
title: General
description:  ""
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Tracking Wheels 




### odom_tracker_left_set()
Sets the parallel left tracking wheel for odometry.

`input` an ez::tracking_wheel
<Tabs
  groupId="odom_tracker_left_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_tracker_left_set(tracking_wheel* input);
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);  // ADI Encoders
ez::tracking_wheel left_tracker(1, {'C', 'D'}, 2.75, 4.0);  // ADI Encoders plugged into a Smart port
ez::tracking_wheel horiz_tracker(1, 2.75, 4.0);             // Rotation sensors

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  chassis.odom_tracker_right_set(&right_tracker);
  chassis.odom_tracker_left_set(&left_tracker);
  chassis.odom_tracker_back_set(&horiz_tracker);  // Replace `back` to `front` if your tracker is in the front!
}
```
</TabItem>
</Tabs>






### odom_tracker_right_set()
Sets the parallel right tracking wheel for odometry.  

`input` an ez::tracking_wheel
<Tabs
  groupId="odom_tracker_right_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_tracker_right_set(tracking_wheel* input);
```


</TabItem>


<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);  // ADI Encoders
ez::tracking_wheel left_tracker(1, {'C', 'D'}, 2.75, 4.0);  // ADI Encoders plugged into a Smart port
ez::tracking_wheel horiz_tracker(1, 2.75, 4.0);             // Rotation sensors

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  chassis.odom_tracker_right_set(&right_tracker);
  chassis.odom_tracker_left_set(&left_tracker);
  chassis.odom_tracker_back_set(&horiz_tracker);  // Replace `back` to `front` if your tracker is in the front!
}
```

</TabItem>
</Tabs>






### odom_tracker_front_set()
Sets the perpendicular front tracking wheel for odometry.   

`input` an ez::tracking_wheel
<Tabs
  groupId="odom_tracker_front_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>


<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);  // ADI Encoders
ez::tracking_wheel left_tracker(1, {'C', 'D'}, 2.75, 4.0);  // ADI Encoders plugged into a Smart port
ez::tracking_wheel horiz_tracker(1, 2.75, 4.0);             // Rotation sensors

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  chassis.odom_tracker_right_set(&right_tracker);
  chassis.odom_tracker_left_set(&left_tracker);
  chassis.odom_tracker_front_set(&horiz_tracker);  // Replace `back` to `front` if your tracker is in the front!
}
```

</TabItem>
<TabItem value="proto">

```cpp
void odom_tracker_front_set(tracking_wheel* input);
```


</TabItem>
</Tabs>







### odom_tracker_back_set()
Sets the perpendicular back tracking wheel for odometry.   

`input` an ez::tracking_wheel

<Tabs
  groupId="odom_tracker_back_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_tracker_back_set(tracking_wheel* input);
```
</TabItem>


<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);  // ADI Encoders
ez::tracking_wheel left_tracker(1, {'C', 'D'}, 2.75, 4.0);  // ADI Encoders plugged into a Smart port
ez::tracking_wheel horiz_tracker(1, 2.75, 4.0);             // Rotation sensors

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  chassis.odom_tracker_right_set(&right_tracker);
  chassis.odom_tracker_left_set(&left_tracker);
  chassis.odom_tracker_back_set(&horiz_tracker);  // Replace `back` to `front` if your tracker is in the front!
}
```

</TabItem>
</Tabs>
















## Pose


### odom_enable()
Enables / disables tracking.     

`input` true enables tracking, false disables tracking
<Tabs
  groupId="odom_enable"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_enable(bool input);
```
</TabItem>
<TabItem value="example">

```cpp
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Enable tracking
  chassis.odom_enable(true);
}
```
</TabItem>
</Tabs>




### odom_enabled()
Returns whether the bot is tracking with odometry.   

True means tracking is enabled, false means tracking is disabled.    
<Tabs
  groupId="odom_tracker_left_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
bool odom_enabled();
```
</TabItem>
<TabItem value="example">

```cpp
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Enable tracking
  chassis.odom_enable(true);

  if (chassis.odom_enabled()) 
    printf("Odom is enabled!\n");
}
```
</TabItem>
</Tabs>


### odom_x_set()
Sets the current x position of the robot.     

`x` new x coordinate in inches   
<Tabs
  groupId="odom_x_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_x_set(double x);
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

  chassis.pid_odom_set({{24_in, 0_in}, fwd, 110});
  chassis.pid_wait();

  chassis.odom_x_set(0);  // Set current x to 0

  // This will go back to the starting location
  chassis.pid_odom_set({{-24_in, 0_in}, rev, 110});
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>




### odom_x_set()
Sets the current X coordinate of the robot.     

`p_x` new x coordinate as an okapi unit  
<Tabs
  groupId="odom_x_set_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void odom_x_set(okapi::QLength p_x);
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

  chassis.pid_odom_set({{24_in, 0_in}, fwd, 110});
  chassis.pid_wait();

  chassis.odom_x_set(0_in);  // Set current x to 0

  // This will go back to the starting location
  chassis.pid_odom_set({{-24_in, 0_in}, rev, 110});
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>




  
### odom_y_set()
Sets the current Y coordinate of the robot.        

`y` new y coordinate in inches
<Tabs
  groupId="odom_y_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void odom_y_set(double y);
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

  chassis.pid_odom_set({{0_in, 24_in}, fwd, 110});
  chassis.pid_wait();

  chassis.odom_y_set(0);  // Set current y to 0

  // This will go back to the starting location
  chassis.pid_odom_set({{0_in, -24_in}, rev, 110});
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>




### odom_y_set()
Sets the current Y coordinate of the robot.       

`p_y` new y coordinate as an okapi unit  
<Tabs
  groupId="odom_y_set_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void odom_y_set(okapi::QLength p_y);
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

  chassis.pid_odom_set({{0_in, 24_in}, fwd, 110});
  chassis.pid_wait();

  chassis.odom_y_set(0_in);  // Set current y to 0

  // This will go back to the starting location
  chassis.pid_odom_set({{0_in, -24_in}, rev, 110});
  chassis.pid_wait();
}
```
</TabItem>
</Tabs>




### odom_theta_set()
Sets the current Theta of the robot.     

`a` new angle in degrees
<Tabs
  groupId="odom_theta_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_theta_set(double a);
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

  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait();

  chassis.odom_theta_set(0);  // Set current theta to 0

  // This will go back to the starting location
  chassis.pid_turn_set(-90_deg, 110);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>




### odom_theta_set()
Sets the current angle of the robot.     

`p_y` new angle as an okapi unit
<Tabs
  groupId="odom_theta_set_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_theta_set(okapi::QAngle p_a);
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

  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait();

  chassis.odom_theta_set(0_deg);  // Set current theta to 0

  // This will go back to the starting location
  chassis.pid_turn_set(-90_deg, 110);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>









  

### odom_xy_set()
Sets the current X and Y coordinate for the robot.     

`x` new x value, in inches  
`y` new y value, in inches    
<Tabs
  groupId="odom_xy_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_xy_set(double x, double y);
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

  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();

  chassis.odom_xy_set(0, 0);  // Set current x and y to 0

  // This will go back to the starting location
  chassis.pid_odom_set({{-24_in, -24_in}, rev, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>

 

 
  


### odom_xy_set()
Sets the current X and Y coordinate for the robot.    

`p_x` new x value, okapi unit   
`p_y` new y value, okapi unit    
<Tabs
  groupId="odom_xy_set_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_xy_set(okapi::QLength p_x, okapi::QLength p_y);
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

  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();

  chassis.odom_xy_set(0_in, 0_in);  // Set current x and y to 0

  // This will go back to the starting location
  chassis.pid_odom_set({{-24_in, -24_in}, rev, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>




 


### odom_xyt_set()
Sets the current X, Y, and Theta values for the robot.       

`x` new x value, in inches  
`y` new y value, in inches     
`t` new theta value, in degrees
<Tabs
  groupId="odom_xyt_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_xyt_set(double x, double y, double t);
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

  chassis.pid_odom_set({{24_in, 24_in, 45_deg}, fwd, 110});
  chassis.pid_wait();

  chassis.odom_xyt_set(0, 0, -45);  

  // This will go back to the starting location
  chassis.pid_odom_set({{-24_in, -24_in}, rev, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>

 

 
  


### odom_xy_set()
Sets the current X, Y, and Theta values for the robot.     

`p_x` new x value, okapi unit   
`p_y` new y value, okapi unit  
`p_t` new theta value, okapi unit
<Tabs
  groupId="odom_xy_set_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_xyt_set(okapi::QLength p_x, okapi::QLength p_y, okapi::QAngle p_t);
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

  chassis.pid_odom_set({{24_in, 24_in, 45_deg}, fwd, 110});
  chassis.pid_wait();

  chassis.odom_xyt_set(0_in, 0_in, -45_deg);  

  // This will go back to the starting location
  chassis.pid_odom_set({{-24_in, -24_in}, rev, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>








### odom_pose_set()
Sets the current pose of the robot.        

`itarget` `{x, y, t}` units in inches and degrees   
<Tabs
  groupId="odom_pose_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_pose_set(pose itarget);
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

  chassis.pid_odom_set({{24_in, 24_in, 45_deg}, fwd, 110});
  chassis.pid_wait();

  ez::pose new_pose = {0, 0, -45};
  chassis.odom_xyt_set(new_pose);  

  // This will go back to the starting location
  chassis.pid_odom_set({{-24_in, -24_in}, rev, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>

 

  


### odom_pose_set()
Set the current pose of the robot.        

`itarget` `{x, y, t}` as an okapi unit   
<Tabs
  groupId="odom_pose_set_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_pose_set(united_pose itarget);
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

  chassis.pid_odom_set({{24_in, 24_in, 45_deg}, fwd, 110});
  chassis.pid_wait();

  ez::united_pose new_pose = {0_in, 0_in, -45_deg};
  chassis.odom_xyt_set(new_pose);  

  // This will go back to the starting location
  chassis.pid_odom_set({{-24_in, -24_in}, rev, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 











 









### odom_x_flip()
Flips the C axis.  

`flip` true means left is positive x, false means right is positive x
<Tabs
  groupId="odom_x_flip"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_x_flip(bool flip = true);
```


</TabItem>


<TabItem value="example">

```cpp
void base() {
  ez::united_pose start_intaking_here;

  if (chassis.odom_x_direction_get() == false)
    start_intaking_here = {12_in, 24_in};  // If red
  else
    start_intaking_here = {15_in, 24_in};  // If blue

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {start_intaking_here, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 48_in, 45_deg}, fwd, 110},
                       true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, 110, true);
  chassis.pid_wait();
}

void red() {
  base();
}

void blue() {
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  base();
}


void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  red(); 
  // blue();
}
```

</TabItem>
</Tabs>



 









### odom_y_flip()
Flips the Y axis.   

`flip` true means down is positive y, false means up is positive y    
<Tabs
  groupId="odom_y_flip"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_y_flip(bool flip = true);
```


</TabItem>


<TabItem value="example">

```cpp
void base() {
  ez::united_pose start_intaking_here;

  if (chassis.odom_y_direction_get() == false)
    start_intaking_here = {12_in, 24_in};  // If red
  else
    start_intaking_here = {15_in, 24_in};  // If blue

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {start_intaking_here, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 48_in, 45_deg}, fwd, 110},
                       true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, 110, true);
  chassis.pid_wait();
}

void red() {
  base();
}

void blue() {
  chassis.odom_y_flip();
  chassis.odom_theta_flip();
  base();
}


void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);   // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  red(); 
  // blue();
}
```

</TabItem>
</Tabs>



















 

### odom_theta_flip()
Flips the rotation axis.   

`flip` true means counterclockwise is positive, false means clockwise is positive
<Tabs
  groupId="odom_theta_flip"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_theta_flip(bool flip = true);
```


</TabItem>


<TabItem value="example">

```cpp
void base() {
  ez::united_pose start_intaking_here;

  if (chassis.odom_theta_direction_get() == false)
    start_intaking_here = {12_in, 24_in};  // If red
  else
    start_intaking_here = {15_in, 24_in};  // If blue

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {start_intaking_here, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 48_in, 45_deg}, fwd, 110},
                       true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, 110, true);
  chassis.pid_wait();
}

void red() {
  base();
}

void blue() {
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  base();
}


void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  red(); 
  // blue();
}
```

</TabItem>
</Tabs>










### odom_x_get()
Returns the current x position of the robot in inches.             
<Tabs
  groupId="odom_x_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double odom_x_get();
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

  chassis.pid_odom_set({{24_in, 24_in, 45_deg}, fwd, 110});
  chassis.pid_wait();

  printf("X: %.2f  Y: %.2f  T: %.2f\n", chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());
}
```

</TabItem>
</Tabs>




### odom_y_get()
Returns the current Y coordinate of the robot in inches.             
<Tabs
  groupId="odom_y_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double odom_y_get();
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

  chassis.pid_odom_set({{24_in, 24_in, 45_deg}, fwd, 110});
  chassis.pid_wait();

  printf("X: %.2f  Y: %.2f  T: %.2f\n", chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());
}
```

</TabItem>
</Tabs>

### odom_theta_get()
Returns the current Theta of the robot in degrees.             
<Tabs
  groupId="odom_theta_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double odom_theta_get();
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

  chassis.pid_odom_set({{24_in, 24_in, 45_deg}, fwd, 110});
  chassis.pid_wait();

  printf("X: %.2f  Y: %.2f  T: %.2f\n", chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());
}
```

</TabItem>
</Tabs>
 
 



### odom_pose_get()
Returns the current pose of the robot.           
<Tabs
  groupId="odom_pose_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
pose odom_pose_get();
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

  chassis.pid_odom_set({{24_in, 24_in, 45_deg}, fwd, 110});
  chassis.pid_wait();

  ez::pose c_pose = chassis.odom_pose_get();
  printf("X: %.2f  Y: %.2f  T: %.2f\n", c_pose.x, c_pose.y, c_pose.theta);
}
```

</TabItem>
</Tabs>

### odom_x_direction_get()
Checks if X axis is flipped.   

True means left is positive X, false means right is positive X.   
<Tabs
  groupId="odom_x_direction_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
bool odom_x_direction_get();
```


</TabItem>


<TabItem value="example">

```cpp
void base() {
  ez::united_pose start_intaking_here;

  if (chassis.odom_x_direction_get() == false)
    start_intaking_here = {12_in, 24_in};  // If red
  else
    start_intaking_here = {15_in, 24_in};  // If blue

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {start_intaking_here, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 48_in, 45_deg}, fwd, 110},
                       true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, 110, true);
  chassis.pid_wait();
}

void red() {
  base();
}

void blue() {
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  base();
}


void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  red(); 
  // blue();
}
```

</TabItem>
</Tabs>

### odom_y_direction_get()
Checks if Y axis is flipped.   

True means down is positive Y, false means up is positive Y.   
<Tabs
  groupId="odom_y_direction_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
bool odom_y_direction_get();
```


</TabItem>


<TabItem value="example">

```cpp
void base() {
  ez::united_pose start_intaking_here;

  if (chassis.odom_y_direction_get() == false)
    start_intaking_here = {12_in, 24_in};  // If red
  else
    start_intaking_here = {15_in, 24_in};  // If blue

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {start_intaking_here, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 48_in, 45_deg}, fwd, 110},
                       true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, 110, true);
  chassis.pid_wait();
}

void red() {
  base();
}

void blue() {
  chassis.odom_y_flip();
  chassis.odom_theta_flip();
  base();
}


void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);   // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  red(); 
  // blue();
}
```

</TabItem>
</Tabs>

### odom_theta_direction_get()
Checks if the rotation axis is flipped.   

True means counterclockwise is positive, false means clockwise is positive.   
<Tabs
  groupId="odom_theta_direction_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
bool odom_theta_direction_get();
```


</TabItem>


<TabItem value="example">

```cpp
void base() {
  ez::united_pose start_intaking_here;

  if (chassis.odom_theta_direction_get() == false)
    start_intaking_here = {12_in, 24_in};  // If red
  else
    start_intaking_here = {15_in, 24_in};  // If blue

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {start_intaking_here, rev, 110},
                        {{24_in, 24_in}, rev, 110}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  Intake.move(127);
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 48_in, 45_deg}, fwd, 110},
                       true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, 110, true);
  chassis.pid_wait();
}

void red() {
  base();
}

void blue() {
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  base();
}


void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  red(); 
  // blue();
}
```

</TabItem>
</Tabs>
  











 

 
 
 
 
 
 
 
## Movement Constants




### pid_odom_angular_constants_set()
Set the odom angular pid constants object.   
 
`p` proportional term   
`i` integral term  
`d` derivative term   
`p_start_i` error threshold to start integral    
<Tabs
  groupId="pid_odom_angular_constants_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_odom_angular_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);
```
</TabItem>
<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_odom_angular_constants_set(5, 0, 50);
}
```
</TabItem>
</Tabs>





### odom_turn_bias_set()
A proportion of how prioritized turning is during odometry motions.  

Turning is prioritized so the robot "applies brakes" while turning.  Lower number means more braking.  

`bias` a number between 0 and 1
<Tabs
  groupId="odom_turn_bias_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_turn_bias_set(double bias);
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

  chassis.odom_turn_bias_set(1.0);  // Set turn bias to 1

  // Go to 24, 24
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();

  // Reset your angle and position
  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in);

  chassis.odom_turn_bias_set(0.5);  // Set turn bias to 0

  // Go to 24, 24 relative to where the robot ended, but with a new turn bias
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 



### pid_odom_drive_exit_condition_set()
Set's constants for odom driving exit conditions.  

`p_small_exit_time` time to exit when within smalL_error, okapi unit     
`p_small_error` small timer will start when error is within this, okapi unit     
`p_big_exit_time` time to exit when within big_error, okapi unit             
`p_big_error` big timer will start when error is within this, okapi unit        
`p_velocity_exit_time` time, in okapi units, for velocity to be 0          
`p_mA_timeout` velocity timer will start when velocity is 0, okapi unit     
`use_imu` true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't         
<Tabs
  groupId="pid_odom_drive_exit_condition_set_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_odom_drive_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QLength p_small_error, okapi::QTime p_big_exit_time, okapi::QLength p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu = true);
```
</TabItem>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_odom_drive_exit_condition_set(300_ms, 1_in, 500_ms, 3_in, 750_ms, 750_ms);
}
```
</TabItem>
</Tabs>




### pid_odom_drive_exit_condition_set()
Set's constants for odom driving exit conditions.  

`p_small_exit_time` time to exit when within smalL_error, in ms     
`p_small_error` small timer will start when error is within this, in inches     
`p_big_exit_time` time to exit when within big_error, in ms             
`p_big_error` big timer will start when error is within this, in inches        
`p_velocity_exit_time` velocity timer will start when velocity is 0, in ms   
`p_mA_timeout` mA timer will start when the motors are pulling too much current, in ms      
`use_imu` true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't         
<Tabs
  groupId="pid_odom_drive_exit_condition_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_odom_drive_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu = true);
```
</TabItem>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_odom_drive_exit_condition_set(300, 1, 500, 3, 750, 750);
}
```
</TabItem>
</Tabs>





### pid_odom_turn_exit_condition_set()
Set's constants for odom turning exit conditions.  

`p_small_exit_time` time to exit when within smalL_error, okapi unit     
`p_small_error` small timer will start when error is within this, okapi unit     
`p_big_exit_time` time to exit when within big_error, okapi unit             
`p_big_error` big timer will start when error is within this, okapi unit        
`p_velocity_exit_time` time, in okapi units, for velocity to be 0          
`p_mA_timeout` velocity timer will start when velocity is 0, okapi unit     
`use_imu` true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't         
<Tabs
  groupId="pid_odom_turn_exit_condition_set_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_odom_turn_exit_condition_set(okapi::QTime p_small_exit_time, okapi::QAngle p_small_error, okapi::QTime p_big_exit_time, okapi::QAngle p_big_error, okapi::QTime p_velocity_exit_time, okapi::QTime p_mA_timeout, bool use_imu = true);
```
</TabItem>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_odom_turn_exit_condition_set(300_ms, 3_deg, 500_ms, 3_deg, 750_ms, 750_ms);
}
```
</TabItem>
</Tabs>

### pid_odom_turn_exit_condition_set()
Set's constants for odom turning exit conditions.  

`p_small_exit_time` time to exit when within smalL_error, in ms  
`p_small_error` small timer will start when error is within this, in degrees
`p_big_exit_time` time to exit when within big_error, in ms
`p_big_error` big timer will start when error is within this, in degrees
`p_velocity_exit_time`  velocity timer will start when velocity is 0, in ms
`p_mA_timeout` mA timer will start when the motors are pulling too much current, in ms   
`use_imu` true adds the imu for velocity calculation in conjunction with the main sensor, false doesn't    
<Tabs
  groupId="pid_odom_turn_exit_condition_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_odom_turn_exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout, bool use_imu = true);
```
</TabItem>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_odom_turn_exit_condition_set(300, 3, 500, 7, 750, 750);
}
```
</TabItem>
</Tabs>






 
### odom_look_ahead_set()
Sets how far away the robot looks in the path during pure pursuits.  

`distance` how long the "carrot on a stick" is, in inches
<Tabs
  groupId="odom_look_ahead_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_look_ahead_set(double distance);
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

  chassis.odom_look_ahead_set(7.0);  // Set look ahead to 7in

  // Go to 24, 24
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();

  // Reset your angle and position
  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in);

  chassis.odom_look_ahead_set(14.0);  // Set look ahead to 14in

  // Go to 24, 24 relative to where the robot ended, but with a new look ahead
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 



### odom_look_ahead_set()
Sets how far away the robot looks in the path during pure pursuits.  

`distance` how long the "carrot on a stick" is, in okapi units
<Tabs
  groupId="odom_look_ahead_se_okat"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_look_ahead_set(okapi::QLength p_distance);
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

  chassis.odom_look_ahead_set(7_in);  // Set look ahead to 7in

  // Go to 24, 24
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();

  // Reset your angle and position
  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in);

  chassis.odom_look_ahead_set(14_in);  // Set look ahead to 14in

  // Go to 24, 24 relative to where the robot ended, but with a new look ahead
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>







### pid_odom_behavior_set()
Sets the default behavior for turns in odom turning movements.   

`behavior` ez::shortest, ez::longest, ez::left, ez::right, ez::raw     
<Tabs
  groupId="pid_odom_behavior_set"
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

  chassis.pid_odom_behavior_set(ez::longest);  // Set the robot to take the longest path there

  // This will make the robot go the long way around to get to 24, 0
  chassis.pid_odom_set({{24_in, 0_in}, fwd, 110});
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_odom_behavior_set(e_angle_behavior behavior);
```
</TabItem>
</Tabs>

### pid_odom_behavior_get()
Returns the turn behavior for odom turns.    
<Tabs
  groupId="pid_odom_behavior_get"
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

  chassis.pid_odom_behavior_set(ez::longest);  // Set the robot to take the longest path there

  if (chassis.pid_odom_behavior_get() == ez::longest) {
    printf("Current behavior is ez::longest!\n");
  } else {
    printf("Current behavior is not ez::longest!\n");
  }

  // This will make the robot go the long way around to get to 24, 0
  chassis.pid_odom_set({{24_in, 0_in}, fwd, 110});
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
e_angle_behavior pid_odom_behavior_get();
```
</TabItem>
</Tabs>



### odom_path_spacing_set()
Sets the spacing between points when points get injected into the path.   

`p_spacing` a small number in okapi units
<Tabs
  groupId="odom_path_spacing_set_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_path_spacing_set(okapi::QLength p_spacing);
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

  chassis.odom_path_spacing_set(0.5_in);  // Set path spacing to 0.5 inches
  printf("Path Spacing: %.2f\n", chassis.odom_path_spacing_get());

  // Go to 24, 24
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>



### odom_path_spacing_set()
Sets the spacing between points when points get injected into the path.   

`spacing` a small number in inches
<Tabs
  groupId="odom_path_spacing_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_path_spacing_set(double spacing);
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

  chassis.odom_path_spacing_set(0.5_in);  // Set path spacing to 0.5 inches
  printf("Path Spacing: %.2f\n", chassis.odom_path_spacing_get());

  // Go to 24, 24
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>




### odom_turn_bias_get()
Returns the proportion of how prioritized turning is during odometry motions.    
<Tabs
  groupId="odom_turn_bias_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double odom_turn_bias_get();
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

  chassis.odom_turn_bias_set(1.0);  // Set turn bias to 1
  printf("Turn Bias: %.2f\n", chassis.odom_turn_bias_get());

  // Go to 24, 24
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();

  // Reset your angle and position
  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in);

  chassis.odom_turn_bias_set(0.5);  // Set turn bias to 0
  printf("Turn Bias: %.2f\n", chassis.odom_turn_bias_get());

  // Go to 24, 24 relative to where the robot ended, but with a new turn bias
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>
 
### odom_look_ahead_get()
Returns how far away the robot looks in the path during pure pursuits.   
<Tabs
  groupId="odom_look_ahead_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double odom_look_ahead_get();
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

  chassis.odom_look_ahead_set(7_in);  // Set look ahead to 7in
  printf("Look Ahead: %.2f\n", chassis.odom_look_ahead_get());

  // Go to 24, 24
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();

  // Reset your angle and position
  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in);

  chassis.odom_look_ahead_set(14_in);  // Set look ahead to 14in
  printf("Look Ahead: %.2f\n", chassis.odom_look_ahead_get());

  // Go to 24, 24 relative to where the robot ended, but with a new look ahead
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>














### odom_path_spacing_get()
Returns the spacing between points when points get injected into the path.    
<Tabs
  groupId="odom_path_spacing_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double odom_path_spacing_get();
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

  chassis.odom_path_spacing_set(0.5_in);  // Set path spacing to 0.5 inches
  printf("Path Spacing: %.2f\n", chassis.odom_path_spacing_get());

  // Go to 24, 24
  chassis.pid_odom_set({{24_in, 24_in}, fwd, 110});
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>




 

### slew_odom_reenable()
Allows slew to reenable when the new input speed is larger than the current speed during pure pursuits.        

`slew_on` true enables, false disables   
<Tabs
  groupId="slew_odom_reenable"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void slew_odom_reenable(bool reenable);
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

  // Allow slew to reenable when speed dips too low
  chassis.slew_odom_reenable(true);

  // This will slew from the start to 110, 
  // and once the speed dips down to 50 it'll slew again up to 90
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, 110},
                        {{0_in, 20_in}, fwd, 50},
                        {{0_in, 30_in}, fwd, 90}},
                       true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>





### slew_odom_reenabled()
Allows slew to reenable when the new input speed is larger than the current speed during pure pursuits.          
<Tabs
  groupId="slew_odom_reenabled"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
bool slew_odom_reenabled();
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

  // Allow slew to reenable when speed dips too low
  chassis.slew_odom_reenable(true);
  if (chassis.slew_odom_reenabled()) 
    printf("Slew will Reenable in Odom!\n");

  // This will slew from the start to 110, 
  // and once the speed dips down to 50 it'll slew again up to 90
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, 110},
                        {{0_in, 20_in}, fwd, 50},
                        {{0_in, 30_in}, fwd, 90}},
                       true);
  chassis.pid_wait();
}
```

</TabItem>
</Tabs>




### odom_path_smooth_constants_set()
Sets the constants for smoothing out a path.   

Path smoothing based on [https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4](https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4)  

`weight_smooth` how much weight to update the data   
`weight_data` how much weight to smooth the coordinates   
`tolerance` how much change per iteration is necessary to keep iterating    
<Tabs
  groupId="odom_path_smooth_constants_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void odom_path_smooth_constants_set(double weight_smooth, double weight_data, double tolerance);
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

  chassis.pid_drive_toggle(false);  // Disable the drive
  chassis.pid_print_toggle(false);  // Disable movement printing

  // Print default smooth constants
  std::vector<double> smooth_consts = chassis.odom_path_smooth_constants_get();
  printf("Weight Smooth: %.2f   Weight Data: %.2f   Tolerance: %.2f\n", smooth_consts[0], smooth_consts[1], smooth_consts[2]);

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{24_in, 24_in}, fwd, 110}},
                       false);
  chassis.odom_path_print();  // Print the full path to terminal

  // Print updated smooth constants
  chassis.odom_path_smooth_constants_set(0.5, 0.003, 0.0001);
  smooth_consts = chassis.odom_path_smooth_constants_get();
  printf("Weight Smooth: %.2f   Weight Data: %.2f   Tolerance: %.2f\n", smooth_consts[0], smooth_consts[1], smooth_consts[2]);

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{24_in, 24_in}, fwd, 110}},
                       false);
  chassis.odom_path_print();  // Print the full path to terminal
}
```

</TabItem>
</Tabs>



### odom_path_smooth_constants_get()
Returns the constants for smoothing out a path.  

In order of:   
<ul>
  <li>weight_smooth </li>
  <li>weight_data  </li>
  <li>tolerance   </li>
</ul>
<Tabs
  groupId="odom_path_smooth_constants_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
std::vector<double> odom_path_smooth_constants_get();
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

  chassis.pid_drive_toggle(false);  // Disable the drive
  chassis.pid_print_toggle(false);  // Disable movement printing

  // Print default smooth constants
  std::vector<double> smooth_consts = chassis.odom_path_smooth_constants_get();
  printf("Weight Smooth: %.2f   Weight Data: %.2f   Tolerance: %.2f\n", smooth_consts[0], smooth_consts[1], smooth_consts[2]);

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{24_in, 24_in}, fwd, 110}},
                       false);
  chassis.odom_path_print();  // Print the full path to terminal

  // Print updated smooth constants
  chassis.odom_path_smooth_constants_set(0.5, 0.003, 0.0001);
  smooth_consts = chassis.odom_path_smooth_constants_get();
  printf("Weight Smooth: %.2f   Weight Data: %.2f   Tolerance: %.2f\n", smooth_consts[0], smooth_consts[1], smooth_consts[2]);

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{24_in, 24_in}, fwd, 110}},
                       false);
  chassis.odom_path_print();  // Print the full path to terminal
}
```

</TabItem>
</Tabs>


### odom_path_print()
Prints the current path the robot is following.  
<Tabs
  groupId="odom_path_print"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void odom_path_print();
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

  chassis.pid_drive_toggle(false);  // Disable the drive
  chassis.pid_print_toggle(false);  // Disable movement printing

  // Print default smooth constants
  std::vector<double> smooth_consts = chassis.odom_path_smooth_constants_get();
  printf("Weight Smooth: %.2f   Weight Data: %.2f   Tolerance: %.2f\n", smooth_consts[0], smooth_consts[1], smooth_consts[2]);

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{24_in, 24_in}, fwd, 110}},
                       false);
  chassis.odom_path_print();  // Print the full path to terminal

  // Print updated smooth constants
  chassis.odom_path_smooth_constants_set(0.5, 0.003, 0.0001);
  smooth_consts = chassis.odom_path_smooth_constants_get();
  printf("Weight Smooth: %.2f   Weight Data: %.2f   Tolerance: %.2f\n", smooth_consts[0], smooth_consts[1], smooth_consts[2]);

  chassis.pid_odom_set({{{0_in, 24_in}, fwd, 110},
                        {{24_in, 24_in}, fwd, 110}},
                       false);
  chassis.odom_path_print();  // Print the full path to terminal
}
```

</TabItem>
</Tabs>










 

## Boomerang Behavior



### pid_odom_boomerang_constants_set()
Set the odom boomerang pid constants object.   
 
`p` proportional term   
`i` integral term  
`d` derivative term   
`p_start_i` error threshold to start integral    
<Tabs
  groupId="pid_odom_boomerang_constants_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void pid_odom_boomerang_constants_set(double p, double i = 0.0, double d = 0.0, double p_start_i = 0.0);
```
</TabItem>

<TabItem value="example">

```cpp
void initialize() {
  chassis.pid_odom_boomerang_constants_set(5, 0, 50);
}
```
</TabItem>
</Tabs>



### odom_boomerang_dlead_set()
Sets a new dlead.   

Dlead is a proportional value of how much to make the robot curve during boomerang motions.    

`input` a value between 0 and 1   
<Tabs
  groupId="odom_boomerang_dlead_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_boomerang_dlead_set(double input);
```


</TabItem>


<TabItem value="example">

```cpp
void initialize() {
  chassis.odom_boomerang_dlead_set(0.5);
}
```

</TabItem>
</Tabs>
 











### odom_boomerang_distance_set()
Sets how far away the carrot point can be from the target point.    

`distance` distance in inches  
<Tabs
  groupId="odom_boomerang_distance_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_boomerang_distance_set(double distance);
```

</TabItem>

<TabItem value="example">

```cpp
void initialize() {
  chassis.odom_boomerang_dlead_set(0.5);
  chassis.odom_boomerang_distance_set(12);
}
```

</TabItem>
</Tabs>







### odom_boomerang_distance_set()
Sets how far away the carrot point can be from the target point.   

`distance` distance as an okapi unit  
<Tabs
  groupId="odom_boomerang_distance_set_oka"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void odom_boomerang_distance_set(okapi::QLength p_distance);
```


</TabItem>


<TabItem value="example">

```cpp
void initialize() {
  chassis.odom_boomerang_dlead_set(0.5);
  chassis.odom_boomerang_distance_set(12_in);
}
```

</TabItem>
</Tabs>
 






### odom_boomerang_dlead_get()
Returns the current dlead.    
<Tabs
  groupId="odom_boomerang_dlead_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double odom_boomerang_dlead_get();
```


</TabItem>


<TabItem value="example">

```cpp
void initialize() {
  chassis.odom_boomerang_dlead_set(0.5);

  printf("dlead: %.2f\n", chassis.odom_boomerang_dlead_get());
}
```

</TabItem>
</Tabs>

### odom_boomerang_distance_get()
Returns the maximum distance the carrot point can be away from the target point.   
<Tabs
  groupId="odom_boomerang_distance_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double odom_boomerang_distance_get();
```


</TabItem>


<TabItem value="example">

```cpp
void initialize() {
  chassis.odom_boomerang_dlead_set(0.5);
  chassis.odom_boomerang_distance_set(12_in);

  printf("Max Distance: %.2f\n", chassis.odom_boomerang_distance_get());
}
```

</TabItem>
</Tabs>








































