---
layout: default
title: Drive and Telemetry
description: get and set stuff to the drive
---


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


## Initialize Drive

### initialize()
Runs `opcontrol_curve_sd_initialize()` and `drive_imu_calibrate()`.    

`run_loading_animation` false skips the loading animation on the brain screen while the IMU calibrates
<Tabs
  groupId="drive_init"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.initialize();
}
```


</TabItem>


<TabItem value="proto">


```cpp
void Drive::initialize(bool run_loading_animation = true);
```



</TabItem>
</Tabs>


## Set Drive

### drive_set()
Sets the chassis to voltage.   

Disables PID when called.     

`left` voltage for left side, -127 to 127  
`right` voltage for right side, -127 to 127  
<Tabs
  groupId="drive_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  drive_set(127, 127);
  pros::delay(1000); // Wait 1 second
  drive_set(0, 0);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void drive_set(int left, int right);
```

</TabItem>
</Tabs>




 


### drive_brake_set()
Changes the way the drive behaves when it is not under active user control.  

`brake_type` the 'brake mode' of the motor e.g. 'pros::E_MOTOR_BRAKE_COAST' 'pros::E_MOTOR_BRAKE_BRAKE' 'pros::E_MOTOR_BRAKE_HOLD'   
<Tabs
  groupId="drive_brake_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  drive_brake_set_mode(MOTOR_BRAKE_COAST);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void drive_brake_set(pros::motor_brake_mode_e_t brake_type);
```

</TabItem>
</Tabs>





 


### drive_current_limit_set()
Sets the limit for the current on the drive.  

`mA` input in miliamps    
<Tabs
  groupId="drive_current_limit_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  drive_brake_set_mode(1000);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void drive_current_limit_set(int mA);
```

</TabItem>
</Tabs>


### drive_imu_scaler_set()
Calibrates the imu's scale using a physical turn.   

Physically turn the robot 3600 degrees (10 full rotations) and pass in what the imu reported for that turn.  Internally, this is used to divide the imu's raw reading so it reports the true 3600.  See the [IMU Scaling tutorial](https://ez-robotics.github.io/EZ-Template/tutorials/tuning_imu_scale) for the full calibration walkthrough.     

`imu_value_after_3600`  what the imu reads after physically turning the robot 3600 degrees
<Tabs
  groupId="drive_imu_scaler_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.initialize();

  // After physically turning the robot 3600 degrees and reading what the
  // imu reported, pass that value in here.
  chassis.drive_imu_scaler_set(3625.42);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void drive_imu_scaler_set(double imu_value_after_3600);
```

</TabItem>
</Tabs>


### drive_imus_scalers_set()
Calibrates the scale of all IMUs using a physical turn, for drives built with the redundant IMU constructor.   

Physically turn the robot 3600 degrees (10 full rotations) and pass in what each imu reported for that turn.     

`imu_values_after_3600` what each imu reads after physically turning the robot 3600 degrees, input `{3550, 3625...}`, in the same order as the IMU ports passed to the constructor
<Tabs
  groupId="drive_imus_scalers_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.initialize();

  // After physically turning the robot 3600 degrees and reading what each
  // imu reported, pass those values in here, in the same order as the imu
  // ports passed to the constructor.
  chassis.drive_imus_scalers_set({3550.0, 3625.0});
}
```


</TabItem>


<TabItem value="proto">

```cpp
void drive_imus_scalers_set(std::vector<double> imu_values_after_3600);
```

</TabItem>
</Tabs>

 


## Telemetry

### drive_sensor_right()
The position of the right sensor in inches.  

If you have two parallel tracking wheels, this will return tracking wheel position.  Otherwise this returns motor position.  
<Tabs
  groupId="drive_sensor_right"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();

    printf("Right Sensor: %i \n", chassis.drive_sensor_right());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
int drive_sensor_right();
```

</TabItem>
</Tabs>




 


### drive_velocity_right()
The velocity of the right motor.      
<Tabs
  groupId="drive_velocity_right"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();

    printf("Right Velocity: %i \n", chassis.drive_velocity_right());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
int drive_velocity_right();
```

</TabItem>
</Tabs>




 


### drive_mA_right()
The watts of the right motor.      
<Tabs
  groupId="edrive_mA_rightx7"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();

    printf("Right mA: %i \n", chassis.drive_mA_right());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
double drive_mA_right();
```

</TabItem>
</Tabs>




 


### drive_current_right_over()
Return true when the motor is over current.       
<Tabs
  groupId="drive_current_right_over"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();

    printf("Right Over Current: %i \n", chassis.drive_current_right_over());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
bool drive_current_right_over();
```

</TabItem>
</Tabs>





 


### drive_sensor_left()
The position of the left sensor in inches.   

If you have two parallel tracking wheels, this will return tracking wheel position.  Otherwise this returns motor position.   
<Tabs
  groupId="drive_sensor_left"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();

    printf("Left Sensor: %i \n", chassis.drive_sensor_left());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
int drive_sensor_left();
```

</TabItem>
</Tabs>





 


### drive_velocity_left()
The velocity of the left motor.      
<Tabs
  groupId="drive_velocity_left"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();

    printf("Left Velocity: %i \n", chassis.drive_velocity_left());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
int drive_velocity_left();
```

</TabItem>
</Tabs>





 


### drive_mA_left()
The watts of the left motor.      
<Tabs
  groupId="drive_mA_left"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();

    printf("Left mA: %i \n", chassis.drive_mA_left());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
double drive_mA_left();
```

</TabItem>
</Tabs>




 


### drive_current_left_over()
Return true when the motor is over current.     
<Tabs
  groupId="drive_current_left_over"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();

    printf("Left Over Current: %i \n", chassis.drive_current_left_over());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
bool drive_current_left_over();
```

</TabItem>
</Tabs>




 

### drive_sensor_reset()
Reset all the chassis motors and tracking wheels, recommended to run at the start of your autonomous routine.   
<Tabs
  groupId="drive_sensor_reset"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.drive_sensor_reset();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void drive_sensor_reset();
```

</TabItem>
</Tabs>





 


### drive_imu_reset()
Resets the current imu value.  Defaults to 0, recommended to run at the start of your autonomous routine.   

`new_heading_value` new heading value
<Tabs
  groupId="drive_imu_reset"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.drive_imu_reset();
}
```


</TabItem>


<TabItem value="proto">

```cpp
void drive_imu_reset(double new_heading = 0);
```

</TabItem>
</Tabs>




 


### drive_imu_get()
Returns the current imu heading rotation value in degrees.    
<Tabs
  groupId="drive_imu_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();

    printf("Gyro: %f \n", chassis.drive_imu_get());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
double drive_imu_get();
```

</TabItem>
</Tabs>


### drive_imu_accel_get()
Returns the current imu accel x + accel y value.  
<Tabs
  groupId="drive_imu_accel_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();

    printf("Accel x + y: %f \n", chassis.drive_imu_accel_get());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
double drive_imu_accel_get();
```

</TabItem>
</Tabs>




### drive_imu_calibrate()
Calibrates the IMU, recommended to run in initialize().   

`run_loading_animation` true runs the animation, false doesn't      
<Tabs
  groupId="drive_imu_calibrate"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.drive_imu_calibrate();
}
```

</TabItem>


<TabItem value="proto">

```cpp
bool drive_imu_calibrate(bool run_loading_animation = true);
```

</TabItem>
</Tabs>



### drive_imu_scaler_get()
Returns the imu value after a 3600 degree turn that produces the imu's current scale.        
<Tabs
  groupId="drive_imu_scaler_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.initialize();
  chassis.drive_imu_scaler_set(3625.42);
  printf("%.2f\n", chassis.drive_imu_scaler_get()); // Prints 3625.42
}
```


</TabItem>


<TabItem value="proto">

```cpp
double drive_imu_scaler_get();
```

</TabItem>
</Tabs>




 


### drive_imus_scalers_get()
Returns the imu value after a 3600 degree turn that produces each imu's current scale, keyed by port, for drives built with the redundant IMU constructor.
<Tabs
  groupId="drive_imus_scalers_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
chassis.drive_imus_scalers_set({3550.0, 3625.0});
for (auto& [port, value] : chassis.drive_imus_scalers_get())
  printf("port %i: %.2f\n", port, value);
```


</TabItem>


<TabItem value="proto">

```cpp
std::map<int, double> drive_imus_scalers_get();
```

</TabItem>
</Tabs>


### drive_angle_get()
Returns the angle of the robot, from whichever IMU is currently focused.  On a redundant IMU drive, this is the one to read instead of talking to an individual `pros::Imu` directly, since it stays correct across a failover.
<Tabs
  groupId="drive_angle_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
printf("%.2f\n", chassis.drive_angle_get());
```


</TabItem>


<TabItem value="proto">

```cpp
double drive_angle_get();
```

</TabItem>
</Tabs>


### drive_imu_calibrated()
Checks if the imu calibrated successfully or if it took longer than expected.  

Returns true if calibrated successfully, and false if unsuccessful.        
<Tabs
  groupId="drive_imu_calibrated"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}
```


</TabItem>


<TabItem value="proto">

```cpp
bool drive_imu_calibrated();
```

</TabItem>
</Tabs>