---
layout: default
title: Drive and Telemetry
description:  ""
---


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


## Initialize Drive

### initialize()
Runs `opcontrol_curve_sd_initialize()` and `drive_imu_calibrate()`.    

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
void Drive::initialize();
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
Sets a new imu scaling factor.   

This value is multiplied by the imu to change its output.     

`scaler`  factor to scale the imu by   
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
void turn_example() {
  chassis.drive_imu_scaler_set(2);

  // This will now turn to 45 real degrees
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  // This will turn to 22.5 real degrees
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}
```


</TabItem>


<TabItem value="proto">

```cpp
void drive_imu_scaler_set(double scaler);
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
Returns the current imu rotation value in degrees.    
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



### drive_imu_get()
Gets IMU sensor scaler.  This number is multiplied by the imu so users can tune what a "degree" means.     

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
...
```


</TabItem>


<TabItem value="proto">

```cpp
double drive_imu_scaler_get();
```

</TabItem>
</Tabs>


### drive_imu_scaler_get()
Returns the current imu scaling factor.        
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
void turn_example() {
  chassis.drive_imu_scaler_set(2);
  printf("%.2f\n", chassis.drive_imu_scaler_get()); // Prints 2

  // This will now turn to 45 real degrees
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  // This will turn to 22.5 real degrees
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}
```


</TabItem>


<TabItem value="proto">

```cpp
double drive_imu_scaler_get();
```

</TabItem>
</Tabs>






 


### drive_imu_scaler_get()
Checks if the imu calibrated successfully or if it took longer than expected.  

Returns true if calibrated successfully, and false if unsuccessful.        
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