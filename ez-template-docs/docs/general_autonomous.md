---
layout: default
title: General
description:  stuff that applies to multiple types of movements
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


## Functions with Units

### drive_angle_set()
Sets the angle of the robot.  This is useful when your robot is setup in at an unconventional angle and you want 0 to be when you're square with the field.         

`p_angle` an angle unit, angle that the robot will think it's now facing.
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
void drive_angle_set(ez::QAngle p_angle);
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





### pid_wait_until()
Lock the code in a while loop until the robot has driven this far, with units.  

The target is how far the robot has driven since *this* motion started, with the same sign as the drive.  It is not measured from the end of the drive.  `pid_drive_set(24_in, 110); pid_wait_until(6_in);` returns after 6 inches, with 18 inches still to go.  Backward drives use negative numbers: `pid_drive_set(-24_in, 110); pid_wait_until(-6_in);`.  

If the checkpoint can't be reached (past the distance the drive goes, or the wrong sign), the wait returns when the motion finishes and prints why.  `interfered` is only set when something actually stopped the robot.  For example, an odom movement that ends before the robot has traveled `target`, like a 24 inch move waiting until 30 inches, releases the loop when the movement finishes.              

`target` distance driven since this motion started, using units, the same sign as the drive     
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
void pid_wait_until(ez::QLength target);
```


</TabItem>
</Tabs>



### pid_wait_until()
Lock the code in a while loop until the robot has turned or swung past this heading, with units.  

The target is an absolute heading, the same as the heading you gave the turn or swing.  `pid_turn_set(90_deg, 90); pid_wait_until(45_deg);` returns when the robot faces 45 degrees, with 45 degrees still to go.  

If the checkpoint can't be reached (past the heading the motion goes to, or on the other side of where it started), the wait returns when the motion finishes and prints why.  `interfered` is only set when something actually stopped the robot.             

`target` absolute heading for a turn or swing, using units     
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
void pid_wait_until(ez::QAngle target);
```


</TabItem>
</Tabs>




### pid_speed_max_set()
Changes max speed during a drive motion.  

This also applies mid-motion to a running odom motion (`pid_odom_set()`, `pid_odom_pp_set()`, `pid_odom_injected_pp_set()`, `pid_odom_smooth_pp_set()`, `pid_odom_boomerang_set()` and `pid_odom_ptp_set()`).  The new cap replaces the stored speed on every remaining point of the path, not just the current one, and lasts only for the motion currently running.  The next `pid_*_set()` call starts fresh with its own speed.  

`speed` new clipped speed, between 0 and 127     
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




## Functions without Units



### drive_mode_set()
Sets the current mode of the drive.  

:::note

When your brain is connected to a competition switch or field control, EZ-Template sets the drive mode to `ez::DISABLE` while the robot is disabled, and once when autonomous ends.  This stops a motion that field control cut off from resuming on its own when driver control starts.  It does not happen when entering autonomous, and it never happens without a competition switch, so testing at your desk or running autons from the brain menu is not affected.  

:::

`p_mode` the current task running for the drive.  accepts `ez::DISABLE`, `ez::SWING`, `ez::TURN`, `ez::DRIVE`           
`stop_drive` if the drive motors stop when `p_mode` is `ez::DISABLE`.  Defaults to true  
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
void drive_mode_set(e_mode p_mode, bool stop_drive = true);
```


</TabItem>
</Tabs>


### drive_rpm_set()
Sets the wheel's RPM: the same number as the constructor's `ticks`, cartridge RPM * (motor gear / wheel gear).  It's the one number EZ-Template has for your gearing, and it's how a shifting transmission tells the drive about a gear change: call it with the wheel RPM of the new gear when you shift.  

You can call it at any time, even in the middle of a motion.  Odom, the running motion, active brake and the waits carry on from where the robot is, and only the distance traveled after the change uses the new RPM.  Has no effect if you have two tracking wheels.  

`rpm` the wheel's RPM, as in the constructor's `ticks`
<Tabs
  groupId="drive_rpm_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void drive_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.drive_rpm_set(50);  // Engage torque rpm

  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.drive_rpm_set(343);  // Return back to normal rpm
}
```

</TabItem>


<TabItem value="proto">

```cpp
void drive_rpm_set(double rpm);
```


</TabItem>
</Tabs>


### pid_drive_toggle()
Toggles set drive in autonomous.       

`toggle` true enables, false disables       
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

  chassis.pid_drive_toggle(false); // Disable drive

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
Toggles printing in autonomous.  

`toggle` true enables, false disables  
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

  chassis.pid_print_toggle(false); // Disable prints

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
Lock the code in a while loop until the robot has driven this far, or turned or swung past this heading, without units.  

For drives the target is how far the robot has driven since *this* motion started, in inches, with the same sign as the drive.  `pid_drive_set(24_in, 110); pid_wait_until(6);` returns after 6 inches, with 18 inches still to go.  Backward drives use negative numbers: `pid_drive_set(-24_in, 110); pid_wait_until(-6);`.  

For turns and swings the target is an absolute heading in degrees, the same as the turn target.  `pid_turn_set(90_deg, 90); pid_wait_until(45);` returns when the robot faces 45 degrees.  

If the checkpoint can't be reached (past the target, or the wrong sign), the wait returns when the motion finishes and prints why.  `interfered` is only set when something actually stopped the robot.  This check is for drives, turns and swings.  An odom motion returns when it finishes without a message, for example a 24 inch move waiting until 30 inches.          

`target` for driving, inches driven since this motion started (same sign as the drive).  For turns/swings, an absolute heading in degrees  
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






### pid_angle_behavior_set()
Sets the default behavior for turns in odom, swinging, and turning.   

`behavior` ez::shortest, ez::longest, ez::ccw, ez::cw, ez::raw    
<Tabs
  groupId="pid_angle_behavior_set"
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

  chassis.pid_angle_behavior_set(ez::longest);  // Set the robot to take the longest path there

  // This will make the robot go the long way around to get to 90 degrees
  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait();

  // This will make the robot go the long way around to get to 0 degrees
  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 110);
  chassis.pid_wait();

  // This will make the robot go the long way around to get to 24, 0
  chassis.pid_odom_set({{24_in, 0_in}, ez::fwd, 110});
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_angle_behavior_set(e_angle_behavior behavior);
```
</TabItem>
</Tabs>







### pid_angle_behavior_tolerance_set() 
Gives some wiggle room in shortest vs longest, so a 180.1 and 179.9 degree turns have consistent behavior.   

`p_tolerance` angle wiggle room, a unit    
<Tabs
  groupId="pid_angle_behavior_tolerance_set_oka"
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

  chassis.pid_angle_behavior_tolerance_set(3_deg);  // Set the behavior tolerance to 3 degrees
  chassis.pid_angle_behavior_bias_set(ez::cw);      // When a turn is within the tolerance above, the behavior will default to this

  chassis.odom_theta_set(-1_deg);

  // Even though the fastest way here is to go counter clockwise, the robot will go clockwise 
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_angle_behavior_tolerance_set(ez::QAngle p_tolerance);
```
</TabItem>
</Tabs>




### pid_angle_behavior_tolerance_set()
Gives some wiggle room in shortest vs longest, so a 180.1 and 179.9 degree turns have consistent behavior.  

`tolerance` angle wiggle room, in degrees  
<Tabs
  groupId="pid_angle_behavior_tolerance_set"
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

  chassis.pid_angle_behavior_tolerance_set(3);  // Set the behavior tolerance to 3 degrees
  chassis.pid_angle_behavior_bias_set(ez::cw);  // When a turn is within the tolerance above, the behavior will default to this

  chassis.odom_theta_set(-1_deg);

  // Even though the fastest way here is to go counter clockwise, the robot will go clockwise 
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_angle_behavior_tolerance_set(double tolerance);
```
</TabItem>
</Tabs>


### pid_angle_behavior_bias_set()
When a turn is within its tolerance, you can have it bias left or right.   

`behavior` ez::ccw or ez::cw  
<Tabs
  groupId="pid_angle_behavior_bias_set"
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

  chassis.pid_angle_behavior_tolerance_set(3_deg);  // Set the behavior tolerance to 3 degrees
  chassis.pid_angle_behavior_bias_set(ez::cw);      // When a turn is within the tolerance above, the behavior will default to this

  chassis.odom_theta_set(-1_deg);

  // Even though the fastest way here is to go counter clockwise, the robot will go clockwise 
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
void pid_angle_behavior_bias_set(e_angle_behavior behavior);
```
</TabItem>
</Tabs>





## Getter



### drive_mode_get()
Returns the current drive mode that the task is running.  

Returns `ez::DISABLE`, `ez::SWING`, `ez::TURN`, `ez::TURN_TO_POINT`, `ez::DRIVE`, `ez::POINT_TO_POINT`, or `ez::PURE_PURSUIT`.           
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



### drive_rpm_get()
Returns the wheel's RPM: the constructor's `ticks`, or the last value given to `drive_rpm_set()`.     
<Tabs
  groupId="drive_rpm_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void drive_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.drive_rpm_set(50);  // Engage torque rpm
  printf("%.2f\n", chassis.drive_rpm_get());

  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.drive_rpm_set(343);  // Return back to normal 
  printf("%.2f\n", chassis.drive_rpm_get());
}
```

</TabItem>


<TabItem value="proto">

```cpp
double drive_rpm_get();
```


</TabItem>
</Tabs>


### pid_angle_behavior_tolerance_get()
Returns the wiggle room in shortest vs longest, so a 180.1 and 179.9 degree turns have consistent behavior.   
<Tabs
  groupId="pid_angle_behavior_tolerance_get"
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

  chassis.pid_angle_behavior_tolerance_set(3_deg);  // Set the behavior tolerance to 3 degrees
  chassis.pid_angle_behavior_bias_set(ez::cw);      // When a turn is within the tolerance above, the behavior will default to this

  printf("Tolerance is: %.2f\n", chassis.pid_angle_behavior_tolerance_get());  // This will print 3

  chassis.odom_theta_set(-1_deg);

  // Even though the fastest way here is to go counter clockwise, the robot will go clockwise 
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
double pid_angle_behavior_tolerance_get();
```
</TabItem>
</Tabs>











### pid_angle_behavior_bias_get()
Returns the behavior when a turn is within its tolerance, you can have it bias left or right.    
<Tabs
  groupId="pid_angle_behavior_bias_get"
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

  chassis.pid_angle_behavior_tolerance_set(3);  // Set the behavior tolerance to 3 degrees
  chassis.pid_angle_behavior_bias_set(ez::cw);  // When a turn is within the tolerance above, the behavior will default to this

  if (chassis.pid_angle_behavior_bias_get() == ez::cw) {
    printf("Behavior bias is cw!\n");
  } else {
    printf("Behavior bias is not cw!\n");
  }

  chassis.odom_theta_set(-1_deg);

  // Even though the fastest way here is to go counter clockwise, the robot will go clockwise 
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait();
}
```
</TabItem>
<TabItem value="proto">

```cpp
e_angle_behavior pid_angle_behavior_bias_get();
```
</TabItem>
</Tabs>





## Misc.

### pid_wait()
Lock the code in a while loop until the robot has settled.     
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


### pid_wait_quick()
Lock the code in a while loop until the robot has passed its target (or an exit condition fires first).   

Wrapper for pid_wait_until(target), target is your previously input target.        
<Tabs
  groupId="pid_wait_quick"
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
  chassis.pid_wait_quick();

  chassis.pid_turn_set(0, 110);
  chassis.pid_wait_quick();
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_wait_quick();
```


</TabItem>
</Tabs>


### pid_wait_quick_chain()
Lock the code in a while loop until the robot has passed its target (or an exit condition fires first).   

This also adds distance to target, and then exits with pid_wait_quick.   

This will exit the motion while carrying momentum into the next motion.   
<Tabs
  groupId="pid_wait_quick_chain"
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
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0, 110);
  chassis.pid_wait_quick();
}
```


</TabItem>


<TabItem value="proto">

```cpp
void pid_wait_quick_chain();
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






### interfered
Boolean that returns true when `pid_wait()` or `pid_wait_until()` exit with velocity or is_over_current.  It goes back to false at the start of every new motion.  This can be used to detect unwanted motion and stop the drive motors from overheating during autonomous.     
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
















