---
layout: default
title: Set Drive and Telemetry
parent: Docs
nav_order: 3
---

# **Set Drive and Telemetry**
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}


---

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

---

# Set Drive

## tank_set()
Sets the drive to voltage.  
`left` an integer between -127 and 127.  
`right` an integer between -127 and 127.  
**Prototype**
```cpp
void tank_set(int left, int right);
```

**Example**
```cpp
void autonomous() {
  tank_set(127, 127);
  pros::delay(1000); // Wait 1 second
  tank_set(0, 0);
}
```


---


## drive_brake_set()
Sets brake mode for all drive motors.  
`brake_type`  takes either `MOTOR_BRAKE_COAST`, `MOTOR_BRAKE_BRAKE`, and `MOTOR_BRAKE_HOLD` as parameters.   
**Prototype**
```cpp
void drive_brake_set(pros::motor_brake_mode_e_t brake_type);
```

**Example**
```cpp
void initialize() {
  set_drive_brake_mode(MOTOR_BRAKE_COAST);
}
```


---


## drive_current_limit_set()
Sets mA limit to the drive.  Default is 2500.    
`mA`input miliamps.  
**Prototype**
```cpp
void drive_current_limit_set(int mA);
```

**Example**
```cpp
void initialize() {
  set_drive_brake_mode(1000);
}
```


---


# Telemetry

## sensor_right()
Returns right sensor, either integrated encoder or external encoder.   
**Prototype**
```cpp
int sensor_right();
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    chassis.tank();

    printf("Right Sensor: %i \n", chassis.sensor_right());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## velocity_right()
Returns integrated encoder velocity.      
**Prototype**
```cpp
int velocity_right();
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    chassis.tank();

    printf("Right Velocity: %i \n", chassis.velocity_right());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## mA_right()
Returns current mA being used.      
**Prototype**
```cpp
double mA_right();
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    chassis.tank();

    printf("Right mA: %i \n", chassis.mA_right());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## over_current_right()
Returns `true` when the motor is over current.     
**Prototype**
```cpp
bool over_current_right();
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    chassis.tank();

    printf("Right Over Current: %i \n", chassis.over_current_right());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## sensor_left()
Returns left sensor, either integrated encoder or external encoder.   
**Prototype**
```cpp
int sensor_left();
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    chassis.tank();

    printf("Left Sensor: %i \n", chassis.sensor_left());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## velocity_left()
Returns integrated encoder velocity.      
**Prototype**
```cpp
int velocity_left();
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    chassis.tank();

    printf("Left Velocity: %i \n", chassis.velocity_left());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## mA_left()
Returns current mA being used.      
**Prototype**
```cpp
double mA_left();
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    chassis.tank();

    printf("Left mA: %i \n", chassis.mA_left());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## over_current_left()
Returns `true` when the motor is over current.     
**Prototype**
```cpp
bool over_current_left();
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    chassis.tank();

    printf("Left Over Current: %i \n", chassis.over_current_left());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---

## drive_sensors_reset()
Resets integrated encoders and trackers if applicable.   
**Prototype**
```cpp
void drive_sensors_reset();
```

**Example**
```cpp
void initialize() {
  chassis.drive_sensors_reset();
}
```


---


## imu_reset()
Sets current gyro position to parameter, defaulted to 0.    
**Prototype**
```cpp
void imu_reset(double new_heading = 0);
```

**Example**
```cpp
void initialize() {
  chassis.imu_reset();
}
```


---


## imu_get()
Gets IMU.    
**Prototype**
```cpp
double imu_get();
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    chassis.tank();

    printf("Gyro: %f \n", chassis.imu_get());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## imu_calibrate()
Calibrates IMU, and vibrates the controler after a successful calibration.      
**Prototype**
```cpp
bool imu_calibrate();
```

**Example**
```cpp
void initialize() {
  chassis.imu_calibrate();
}
```


---