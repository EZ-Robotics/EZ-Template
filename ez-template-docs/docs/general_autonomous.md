---
layout: default
title: General
description:  stuff that applies to multiple types of movements
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


## Functions with Okapi Units

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
Lock the code in a while loop until this position has passed for driving with okapi units.               

`target` for driving and swings, using okapi units     
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
Lock the code in a while loop until this position has passed for turning or swinging with okapi units.             

`target` for turning, using okapi units     
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




### pid_speed_max_set()
Changes max speed during a drive motion.  

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




## Functions without Okapi Units



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


### drive_rpm_set()
Set the cartridge/wheel rpm of the robot.     

`rpm` rpm of the cartridge or wheel
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


### drive_ratio_set()
Set the ratio of the robot.  

`ratio` ratio of the gears  
<Tabs
  groupId="drive_ratio_set"
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

  chassis.drive_ratio_set(0.083);  // Engage torque rpm

  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.drive_ratio_set(1.79);  // Return back to normal rpm
}
```

</TabItem>


<TabItem value="proto">

```cpp
void drive_ratio_set(double ratio);
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
Lock the code in a while loop until this position has passed for driving without okapi units.           

`target` for driving or turning, using a double.  degrees for turns/swings, inches for driving  
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

`behavior` ez::shortest, ez::longest, ez::left, ez::right, ez::raw    
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
  chassis.pid_odom_set({{24_in, 0_in}, fwd, 110});
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

`p_tolerance` angle wiggle room, an okapi unit    
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
void pid_angle_behavior_tolerance_set(okapi::QAngle p_tolerance);
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

`behavior` ez::left or ez::right  
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

Returns `ez::DISABLE`, `ez::SWING`, `ez::TURN`, `ez::DRIVE`.           
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
Returns the current cartridge / wheel rpm.     
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


### drive_ratio_get()
Returns the ratio of the drive.    
<Tabs
  groupId="drive_ratio_get"
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

  chassis.drive_ratio_set(0.083);  // Engage torque rpm
  printf("%.2f\n", chassis.drive_ratio_get());

  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.drive_ratio_set(1.79);  // Return back to normal rpm
  printf("%.2f\n", chassis.drive_ratio_get());
}
```

</TabItem>


<TabItem value="proto">

```cpp
double drive_ratio_get();
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
e_angle_behavior pid_angle_behavior_bias_get(e_angle_behavior);
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
Lock the code in a while loop until the robot has settled.   

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
Lock the code in a while loop until the robot has settled.   

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
















