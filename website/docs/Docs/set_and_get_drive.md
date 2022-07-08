---
layout: default
title: Drive and Telemetry
parent: Docs
nav_order: 3
---


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Assumed Constructor

All code below assumes this constructor is used.  As long as the name of the constructor is `chassis`, any of the constructors can be used. 

```cpp
// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {1, -2, 3}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{-4, 5, -6}

  // IMU Port
  ,7

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,4.125

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,2.333

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  ,{1, 2}

  // Right Tracking Wheel Ports (negative port will reverse it!)
  ,{3, 4}
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,9
);

```


# Set Drive

## set_tank()
Sets the drive to voltage.  
`left` an integer between -127 and 127.  
`right` an integer between -127 and 127.  

<Tabs
  groupId="ex2"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  set_tank(127, 127);
  pros::delay(1000); // Wait 1 second
  set_tank(0, 0);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void set_tank(int left, int right);
```

</TabItem>
</Tabs>




 


## set_drive_brake()
Sets brake mode for all drive motors.  
`brake_type`  takes either `MOTOR_BRAKE_COAST`, `MOTOR_BRAKE_BRAKE`, and `MOTOR_BRAKE_HOLD` as parameters.   

<Tabs
  groupId="ex3"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  set_drive_brake_mode(MOTOR_BRAKE_COAST);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void set_drive_brake(pros::motor_brake_mode_e_t brake_type);
```

</TabItem>
</Tabs>



 


## set_drive_current_limit()
Sets mA limit to the drive.  Default is 2500.    
`mA`input miliamps.  

<Tabs
  groupId="ex4"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  set_drive_brake_mode(1000);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void set_drive_current_limit(int mA);
```

</TabItem>
</Tabs>




 


# Telemetry

## right_sensor()
Returns right sensor, either integrated encoder or external encoder.   

<Tabs
  groupId="ex5"
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
    chassis.tank();

    printf("Right Sensor: %i \n", chassis.right_sensor());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
int right_sensor();
```

</TabItem>
</Tabs>




 


## right_velocity()
Returns integrated encoder velocity.      

<Tabs
  groupId="ex6"
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
    chassis.tank();

    printf("Right Velocity: %i \n", chassis.right_velocity());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
int right_velocity();
```

</TabItem>
</Tabs>




 


## right_mA()
Returns current mA being used.      

<Tabs
  groupId="ex7"
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
    chassis.tank();

    printf("Right mA: %i \n", chassis.right_mA());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
double right_mA();
```

</TabItem>
</Tabs>




 


## right_over_current()
Returns `true` when the motor is over current.     

<Tabs
  groupId="ex8"
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
    chassis.tank();

    printf("Right Over Current: %i \n", chassis.right_over_current());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
bool right_over_current();
```

</TabItem>
</Tabs>





 


## left_sensor()
Returns left sensor, either integrated encoder or external encoder.   

<Tabs
  groupId="ex9"
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
    chassis.tank();

    printf("Left Sensor: %i \n", chassis.left_sensor());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
int left_sensor();
```

</TabItem>
</Tabs>





 


## left_velocity()
Returns integrated encoder velocity.      

<Tabs
  groupId="ex10"
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
    chassis.tank();

    printf("Left Velocity: %i \n", chassis.left_velocity());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
int left_velocity();
```

</TabItem>
</Tabs>





 


## left_mA()
Returns current mA being used.      

<Tabs
  groupId="ex11"
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
    chassis.tank();

    printf("Left mA: %i \n", chassis.left_mA());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
double left_mA();
```

</TabItem>
</Tabs>




 


## left_over_current()
Returns `true` when the motor is over current.     

<Tabs
  groupId="ex12"
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
    chassis.tank();

    printf("Left Over Current: %i \n", chassis.left_over_current());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
bool left_over_current();
```

</TabItem>
</Tabs>




 

## reset_drive_sensor()
Resets integrated encoders and trackers if applicable.   

<Tabs
  groupId="ex13"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.reset_drive_sensor();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void reset_drive_sensor();
```

</TabItem>
</Tabs>





 


## reset_gyro()
Sets current gyro position to parameter, defaulted to 0.    

<Tabs
  groupId="ex14"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.reset_gyro();
}
```


</TabItem>


<TabItem value="proto">

```cpp
void reset_gyro(double new_heading = 0);
```

</TabItem>
</Tabs>




 


## get_gyro()
Gets IMU.    

<Tabs
  groupId="ex15"
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
    chassis.tank();

    printf("Gyro: %f \n", chassis.get_gyro());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
double get_gyro();
```

</TabItem>
</Tabs>




 


## imu_calibrate()
Calibrates IMU, and vibrates the controler after a successful calibration.      

<Tabs
  groupId="ex16"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.imu_calibrate();
}
```

</TabItem>


<TabItem value="proto">

```cpp
bool imu_calibrate();
```

</TabItem>
</Tabs>





 