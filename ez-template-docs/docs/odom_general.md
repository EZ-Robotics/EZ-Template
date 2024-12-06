---
layout: default
title: General
description:  ""
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Tracking Wheels 




### odom_tracker_left_set()
Sets a parallel tracker that's on the left side of the robot.   

`input` an ez tracking wheel
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
Sets a parallel tracker that's on the right side of the robot.  

`input` an ez tracking wheel
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
Sets a horizontal tracker that's at the front of the robot.   

`input` an ez tracking wheel
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
Sets a horizontal tracker that's at the back of the robot.   

`input` an ez tracking wheel

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


### odom_x_set()
Sets the current x position of the robot.     

`x` double, expecting inches
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
Sets the current x position of the robot.     

`p_x` okapi length unit
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
Sets the current y position of the robot.     

`y` double, expecting inches
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
Sets the current y position of the robot.     

`p_y` okapi length unit
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
Sets the current theta of the robot.     

`a` double, expecting degrees
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
Sets the current theta of the robot.     

`p_y` okapi angle unit
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
Sets the current x and y position of the robot.     

`x` double, expecting inches  
`y` double, expecting inches    
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
Sets the current x and y position of the robot.     

`p_x` okapi length unit   
`p_y` okapi length unit    
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
Sets the current x and y position of the robot, and the current angle.     

`x` double, expecting inches  
`y` double, expecting inches     
`t` double, expecting inches
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
Sets the current x and y position of the robot, and the current angle.     

`p_x` okapi length unit   
`p_y` okapi length unit  
`p_t` okapi angle unit   
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
Set the current pose of the robot.        

`itarget` pose, expecting `{0, 0, 0}`     
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

`itarget` pose with okapi units, expecting `{0_in, 0_in, 0_deg}`     
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
Flips the x axis.  By default, right is positive x.  True makes left positive x, false makes right positive x.  

`flip` bool to flip x axis
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
Flips the y axis.  By default, forward is positive y.  True makes backwards positive y, false makes forward positive y.  

`flip` bool to flip y axis
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
Flips the rotation axis.  By default, clockwise is positive angle.  True makes clockwise positive angle, false makes counter clockwise positive angle.  

`flip` bool to flip rotation axis
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
Returns the current y position of the robot in inches.             
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
Returns the current angle of the robot in degrees.             
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
  groupId="odom_pose_set"
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
Returns the direction of the x axis.  False means right is positive x, true means left is positive x.  
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
Returns the direction of the y axis.  False means forward is positive y, false means backward is positive y.   
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
Returns the direction of the rotation axis.  False means counter clockwise is positive angle, true means clockwise is positive angle.    
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



### odom_turn_bias_set()
By default, EZ-Template uses cosine scaling.  This is a scaling value that amplifies what cosine scaling does.  `1.0` is the least turn priority, and smaller numbers give more turn priority.   

`bias` double, a number between 0 and 1
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
 








 
### odom_look_ahead_set()
Sets the look ahead distance.  This is how far ahead in the path the robot looks at.  Larger numbers will result in worse path following.   

`distance` double, expecting inches
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
Sets the look ahead distance.  This is how far ahead in the path the robot looks at.  Larger numbers will result in worse path following.   

`distance` okapi distance unit
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
Changes the default turn behavior for odom motions.   

`behavior` the default turn behavior for odom motions.  This can be `ez::shortest`, `ez::longest`, `ez::cw`, `ez::ccw`, or `ez::raw`.     
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
Returns the default turn behavior for odom motions.  This can be `ez::shortest`, `ez::longest`, `ez::cw`, `ez::ccw`, or `ez::raw`.    
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
Sets the spacing between injected points in a path.   

`p_spacing` okapi length unit
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



### odom_turn_bias_get()
Returns the current turn bias.  
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
Returns the current look ahead distance.   
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
Returns the current spacing between injected points.    
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




 

 

## Boomerang Behavior




### odom_boomerang_dlead_set()
Sets the dlead for boomerang.  Dlead is a value you'll tune that sets how the carrot point reacts relative to where the robot is currently.  You can play with [this demos](https://www.desmos.com/calculator/sptjw5szex) to get a better idea on how it works.   

`input` double, a number between 0-1
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
Sets a maximum distance that the carrot point can be away from the target.  This helps bring consistency in how boomerang behaves across multiple target points.  You can play with [this demos](https://www.desmos.com/calculator/sptjw5szex) to get a better idea on how boomerang works.   

`distance` double, expecting inches
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
Sets a maximum distance that the carrot point can be away from the target.  This helps bring consistency in how boomerang behaves across multiple target points.  You can play with [this demos](https://www.desmos.com/calculator/sptjw5szex) to get a better idea on how boomerang works.   

`distance` okapi length unit
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
Returns the dlead for boomerang.  
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






### odom_path_spacing_set()
Sets the spacing between injected points in a path.   

`spacing` double, expecting inches
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






