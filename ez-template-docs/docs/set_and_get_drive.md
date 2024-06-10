---
layout: default
title: Drive and Telemetry
parent: Docs
description:  ""
nav_order: 3
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
Sets the drive to voltage.  Sets mode to `ez::DISABLE`.  

`left` an integer between -127 and 127  
`right` an integer between -127 and 127  

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
Sets brake mode for all drive motors.  

`brake_type`  takes either `MOTOR_BRAKE_COAST`, `MOTOR_BRAKE_BRAKE`, and `MOTOR_BRAKE_HOLD` as parameters   

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
Sets mA limit to the drive.  Default is 2500.  

`mA` input miliamps  

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
Sets a scaler for the imu.  This number is multiplied by the imu so users can tune what a "degree" is.   

`scaler`  a small double   
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
Returns right sensor value, either integrated encoder or external encoder.   

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
Returns integrated encoder velocity.      

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
Returns current mA being used.      

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
Returns `true` when the motor is pulling too many amps.     

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
Returns left sensor value, either integrated encoder or external encoder.   

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
Returns integrated encoder velocity.      

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
Returns current mA being used.      

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
Returns `true` when the motor is pulling too many amps.     

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
Resets integrated encoders and trackers if applicable.   

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
Sets current gyro position to parameter, defaulted to 0.    

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
Gets IMU sensor, value is degrees.    

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
Gets imu x + y acceleration.  This is (optionally) used internally as the secondary sensor for velocity exiting.  

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
Calibrates IMU, and vibrates the controller after a successful calibration.      

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
bool drive_imu_calibrate();
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
Gets the scaler for the imu.  This number is multiplied by the imu so users can tune what a "degree" is.     
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