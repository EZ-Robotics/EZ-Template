---
layout: default
title: General
description:  ""
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Tracking Wheels 




### odom_tracker_left_set()
Sets a tracker that's on the left side of the robot.   

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
Sets a tracker that's on the left side of the robot.   

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
  chassis.odom_tracker_back_set(&horiz_tracker);  // Replace `back` to `front` if your tracker is in the front!
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
Sets a tracker that's on the left side of the robot.   

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




## General












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



  /**
   * Sets a new pose for the robot
   *
   * \param pose
   *        {x, y, t} units in inches and degrees
   */
  void odom_pose_set(pose itarget);

  /**
   * Sets a new pose for the robot
   *
   * \param united pose
   *        {x, y, t} as an okapi unit
   */
  void odom_pose_set(united_pose itarget);

  /**
   * Sets a new X and Y value for the robot
   *
   * \param x
   *        new x value, in inches
   * \param y
   *        new y value, in inches
   */
  void odom_xy_set(double x, double y);

  /**
   * Sets a new X and Y value for the robot
   *
   * \param p_x
   *        new x value, okapi unit
   * \param p_y
   *        new y value, okapi unit
   */
  void odom_xy_set(okapi::QLength p_x, okapi::QLength p_y);

  /**
   * Sets a new X, Y, and Theta value for the robot
   *
   * \param x
   *        new x value, in inches
   * \param y
   *        new y value, in inches
   * \param t
   *        new theta value, in degrees
   */
  void odom_xyt_set(double x, double y, double t);

  /**
   * Sets a new X, Y, and Theta value for the robot
   *
   * \param p_x
   *        new x value, okapi unit
   * \param p_y
   *        new y value, okapi unit
   * \param p_t
   *        new theta value, okapi unit
   */
  void odom_xyt_set(okapi::QLength p_x, okapi::QLength p_y, okapi::QAngle p_t);

  /**
   * Returns the current pose of the robot
   */
  pose odom_pose_get();

  /**
   * Resets xyt to 0
   */
  void odom_reset();

  /**
   * Flips the X axis
   *
   * \param flip
   *        true means left is positive x, false means right is positive x
   */
  void odom_x_flip(bool flip = true);

  /**
   * Checks if x axis is flipped.  True means left is positive x, false means right is positive x
   */
  bool odom_x_direction_get();

  /**
   * Flips the Y axis
   *
   * \param flip
   *        true means down is positive Y, false means up is positive Y
   */
  void odom_y_flip(bool flip = true);

  /**
   * Checks if y axis is flipped.  True means down is positive Y, false means up is positive Y
   */
  bool odom_y_direction_get();

  /**
   * Flips the rotation axis
   *
   * \param flip
   *        true means counterclockwise is positive, false means clockwise is positive
   */
  void odom_theta_flip(bool flip = true);

  /**
   * Checks if the rotation axis is flipped.  True means counterclockwise is positive, false means clockwise is positive
   */
  bool odom_theta_direction_get();





  /**
   * Returns the current x coordinate of the robot
   */
  double odom_x_get();
    /**
   * Returns the current y coordinate of the robot
   */
  double odom_y_get();
    /**
   * Returns the current angle of the robot
   */
  double odom_theta_get();





  ## Boomerang Behavior
    /**
   * Sets a new dlead.  Dlead is a proportional value of how much to make the robot curve during boomerang motions.
   *
   * \param input
   *        a value between 0 and 1.
   */
  void odom_boomerang_dlead_set(double input);

  /**
   * Returns the current dlead.
   */
  double odom_boomerang_dlead_get();

  /**
   * This maxes out how far away the carrot point can be from the target.
   *
   * \param distance
   *        distance in inches
   */
  void odom_boomerang_distance_set(double distance);

  /**
   * This maxes out how far away the carrot point can be from the target.
   *
   * \param distance
   *        distance as an okapi unit
   */
  void odom_boomerang_distance_set(okapi::QLength p_distance);

  /**
   * Returns how far away the carrot point can be from target
   */
  double odom_boomerang_distance_get();









  ## Movement Constants
    /**
   * A proportion of how prioritized turning is during odometry motions.  Turning is prioritized so the robot correctly slows down during turns.
   *
   * \param bias
   *        some number probably less than 5
   */
  void odom_turn_bias_set(double bias);

  /**
   * Returns the proportion of how prioritized turning is during odometry motions.
   */
  double odom_turn_bias_get();

  /**
   * The spacing between points when points get injected into the path
   *
   * \param spacing
   *        a small number in inches
   */
  void odom_path_spacing_set(double spacing);

  /**
   * How far away the robot looks in the path during pure pursuits
   *
   * \param distance
   *        how long the "carrot on a stick" is, in inches
   */
  void odom_look_ahead_set(double distance);

  /**
   * How far away the robot looks in the path during pure pursuits
   *
   * \param distance
   *        how long the "carrot on a stick" is, in okapi units
   */
  void odom_look_ahead_set(okapi::QLength p_distance);
  


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







  ## Path
    /**
   * The spacing between points when points get injected into the path
   *
   * \param spacing
   *        a small number in okapi units
   */
  void odom_path_spacing_set(okapi::QLength p_spacing);

  /**
   * Returns the spacing between points when points get injected into the path
   */
  double odom_path_spacing_get();
