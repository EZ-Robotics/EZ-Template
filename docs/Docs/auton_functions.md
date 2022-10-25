---
layout: default
title: Autonomous Functions
parent: Docs
nav_order: 4
---


# **Autonomous Functions**
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


## pid_drive_set()
Sets the drive to go forward using PID and heading correction.  
`target` is in inches.  
`speed` is -127 to 127.  It's recommended to keep this at 110.  
`slew_on` will ramp the drive up.
`toggle_heading` will disable heading correction when false. 
**Prototype**
```cpp
void pid_drive_set(double target, int speed, bool slew_on = false, bool toggle_heading = true);
```

**Example**
```cpp
void autonomous() {
  chassis.imu_reset(); 
  chassis.drive_sensors_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_drive_set(24, 110, true);
  chassis.drive_wait_exit();
}
```


---


## pid_turn_set()
Sets the drive to turn using PID.  
`target` is in degrees.  
`speed` is -127 to 127.    
**Prototype**
```cpp
void pid_turn_set(double target, int speed);
```

**Example**
```cpp
void autonomous() {
  chassis.imu_reset(); 
  chassis.drive_sensors_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_turn_set(90, 110);
  chassis.drive_wait_exit();
}
```


---


## pid_swing_set()
Sets the robot to swing using PID.  The robot will turn using one side of the drive, either the left or right.  
`type` is either `ez::LEFT_SWING` or `ez::RIGHT_SWING`.  
`target` is in degrees.  
`speed` is -127 to 127.        
**Prototype**
```cpp
void pid_swing_set(e_swing type, double target, int speed);
```

**Example**
```cpp
void autonomous() {
  chassis.imu_reset(); 
  chassis.drive_sensors_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_swing_set(ez::LEFT_SWING, 45, 110);
  chassis.drive_wait_exit();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0, 110);
  chassis.drive_wait_exit();
}
```


---


## drive_wait_exit()
Locks the code in place until the drive has settled.  This uses the exit conditions from the PID class.      
**Prototype**
```cpp
void drive_wait_exit();
```

**Example**
```cpp
void autonomous() {
  chassis.imu_reset(); 
  chassis.drive_sensors_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_turn_set(90, 110);
  chassis.drive_wait_exit();

  chassis.pid_turn_set(0, 110);
  chassis.drive_wait_exit();
}
```


---



## drive_wait_distance()
Locks the code in place until the drive has passed the input parameter.  This uses the exit conditions from the PID class.           
**Prototype**
```cpp
void drive_wait_distance(double target);
```

**Example**
```cpp
void autonomous() {
  chassis.imu_reset(); 
  chassis.drive_sensors_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_drive_set(48, 110);
  chassis.drive_wait_distance(24);
  chassis.max_speed_set(40);
  chassis.drive_wait_exit();
}
```


---



## pid_targets_reset()
Resets all drive PID targets to 0.       
**Prototype**
```cpp
void pid_targets_reset();
```

**Example**
```cpp
void autonomous() {
  chassis.pid_targets_reset(); // Resets PID targets to 0
  chassis.imu_reset(); // Reset gyro position to 0
  chassis.drive_sensors_reset(); // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.selected_auton_call(); // Calls selected auton from autonomous selector.
}
```


---



## angle_set()
Sets the angle of the robot.  This is useful when your robot is setup in at an unconventional angle and you want 0 to be when you're square with the field.         
**Prototype**
```cpp
void angle_set(double angle);
```

**Example**
```cpp
void autonomous() {
  chassis.pid_targets_reset(); // Resets PID targets to 0
  chassis.imu_reset(); // Reset gyro position to 0
  chassis.drive_sensors_reset(); // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  chassis.angle_set(45);

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.drive_wait_exit();
}
```


---



## max_speed_set()
Sets the max speed of the drive. 
`speed` an integer between -127 and 127.     
**Prototype**
```cpp
void max_speed_set(int speed);
```

**Example**
```cpp
void autonomous() {
  chassis.imu_reset(); 
  chassis.drive_sensors_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.pid_drive_set(48, 110);
  chassis.drive_wait_distance(24);
  chassis.max_speed_set(40);
  chassis.drive_wait_exit();
}
```


---


## set_pid_constants()
*Note: this function was changed with 2.0.1*  
Set PID constants.  Below are the defaults.  
`pid` either `&chassis.pid_heading`, `&chassis.pid_forward_drive`, `&chassis.pid_backward_drive`, `&chassis.pid_turn`, or `&chassis.pid_swing`.   
`p` proportion constant.  
`i` integral constant.  
`d` derivative constant.  
`p_start_i` error needs to be within this for i to start.      
**Prototype**
```cpp
void set_pid_constants(PID* pid, double p, double i, double d, double p_start_i);
```

**Example**
```cpp
void initialize() {
  chassis.set_pid_constants(&chassis.pid_heading, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.pid_forward_drive, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.pid_backward_drive, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.pid_turn, 5, 0.003, 35, 15;
  chassis.set_pid_constants(&chassis.pid_swing, 7, 0, 45, 0);
}
```


---


## slew_min_power_set()
Sets the starting speed for slew, with the ability to have different constants for forward and reverse.  Below is the defaults.   
`fwd` integer between -127 and 127.
`rev` integer between -127 and 127.  
**Prototype**
```cpp
void slew_min_power_set(int fwd, int rev);
```

**Example**
```cpp
void initialize() {
  chassis.slew_min_power_set(80, 80);
}
```


---


## slew_distance_set()
Sets the distance the drive will slew for, with the ability to have different constants for forward and reverse.  Input is inches.  Below is the defaults.   
`fwd` a distance in inches.   
`rev` a distance in inches.   
**Prototype**
```cpp
void slew_distance_set (int fwd, int rev);
```

**Example**
```cpp
void initialize() {
  chassis.set_slew_min_distance(7, 7);
}
```


---


## exit_condition_set()
Sets the exit condition constants. This uses the exit conditions from the PID class.  Below is the defaults.  
`type` either `chassis.turn_exit`, `chassis.swing_exit`, or `chassis.drive_exit`  
`p_small_exit_time` time, in ms, before exiting `p_small_error`  
`p_small_error` small error threshold  
`p_big_exit_time` time, in ms, before exiting `p_big_error`  
`p_big_error` big error threshold  
`p_velocity_exit_time` time, in ms, for velocity to be 0  
`p_mA_timeout` time, in ms, for `is_over_current` to be true       
**Prototype**
```cpp
void exit_condition_set(exit_condition_ &type, int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout);

```

**Example**
```cpp
void initialize() {
  chassis.exit_condition_set(chassis.turn_exit,  100, 3,  500, 7,   500, 500);
  chassis.exit_condition_set(chassis.swing_exit, 100, 3,  500, 7,   500, 500);
  chassis.exit_condition_set(chassis.drive_exit, 80,  50, 300, 150, 500, 500);
}
```


---


## swing_min_set()
Sets the max power of the drive when the robot is within `start_i`.  This only enalbes when `i` is enabled, and when the movement is greater then `start_i`.        
**Prototype**
```cpp
void swing_min_set(int min);

```

**Example**
```cpp
void autonomous() {
  chassis.swing_min_set(30);

  chassis.pid_swing_set(45, 110);
  chassis.drive_wait_exit();
}
```


---


## turn_min_set()
Sets the max power of the drive when the robot is within `start_i`.  This only enalbes when `i` is enabled, and when the movement is greater then `start_i`.        
**Prototype**
```cpp
void turn_min_set(int min);

```

**Example**
```cpp
void autonomous() {
  chassis.turn_min_set(30);

  chassis.pid_turn_set(45, 110);
  chassis.drive_wait_exit();
}
```


---


## swing_min_get()
Returns swing min.         
**Prototype**
```cpp
int swing_min_get();

```

**Example**
```cpp
void autonomous() {
  chassis.swing_min_set(30);

  printf("Swing Min: %i", chassis.swing_min_get());
}
```


---


## turn_min_get()
Returns turn min.         
**Prototype**
```cpp
int turn_min_get();

```

**Example**
```cpp
void autonomous() {
  chassis.turn_min_set(30);

  printf("Turn Min: %i", chassis.turn_min_get());
}
```


---


## interfered
Boolean that returns true when `drive_wait_exit()` or `drive_wait_distance()` exit with velocity or is_over_current.      
**Prototype**
```cpp
bool interfered = false;
```

**Example**
```cpp
 void tug (int attempts) {
   for (int i=0; i<attempts-1; i++) {
     // Attempt to drive backwards
     printf("i - %i", i);
     chassis.pid_drive_set(-12, 127);
     chassis.drive_wait_exit();

     // If failsafed...
     if (chassis.interfered) {
       chassis.drive_sensors_reset();
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
  chassis.drive_wait_exit();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90, 90);
  chassis.drive_wait_exit();
}
```


---


## mode_set()
Sets the current mode of the drive.  Accepts `ez::DISABLE`, `ez::SWING`, `ez::TURN`, `ez::DRIVE`.           
**Prototype**
```cpp
void mode_set(e_mode p_mode);
```

**Example**
```cpp
void autonomous() {
  chassis.pid_drive_set(12, DRIVE_SPEED);
  chassis.drive_wait_exit();

  chassis.mode_set(ez::DISABLE); // Disable drive

  chassis.tank_set(-127, -127); // Run drive motors myself
  pros::delay(2000);
  chassis.tank_set(0, 0);
}
```


---


## mode_get()
Returns the current drive mode.  Returns `ez::DISABLE`, `ez::SWING`, `ez::TURN`, `ez::DRIVE`.           
**Prototype**
```cpp
e_mode mode_get();
```

**Example**
```cpp
void autonomous() {
  chassis.pid_drive_set(12, DRIVE_SPEED);
  chassis.drive_wait_exit();

  if (chassis.interfered)
    chassis.mode_set(ez::DISABLE);
  
  if (chassis.mode_get() == ez::DISABLE) {
    chassis.tank_set(-127, -127); // Run drive motors myself
    pros::delay(2000);
    chassis.tank_set(0, 0);
  }
}
```


---


## auto_drive_toggle()
Enables/disables the drive from moving in autonomous.  This is useful for debugging and checking PID variables.  True enables, false disables.         
**Prototype**
```cpp
void auto_drive_toggle(bool toggle);
```

**Example**
```cpp
void autonomous() {
  chassis.pid_drive_set(12, DRIVE_SPEED);
  chassis.drive_wait_exit();

  auto_drive_toggle(false); // Disable drive

  chassis.pid_drive_set(-12, DRIVE_SPEED);
  while (true) {
    printf(" Left Error: %f  Right Error: %f\n", chassis.pid_left.error, chassis.pid_right.error);
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## auto_print_toggle()
Enables/disables the drive functions printing every drive motion.  This is useful when you're debugging something and don't want terminal cluttered.  True enables, false disables.          
**Prototype**
```cpp
void auto_print_toggle(bool toggle);
```

**Example**
```cpp
void autonomous() {
  chassis.pid_drive_set(12, DRIVE_SPEED); // This will print
  chassis.drive_wait_exit(); // This will print

  auto_print_toggle(false); // Disable prints

  chassis.pid_drive_set(-12, DRIVE_SPEED); // This won't print
  chassis.drive_wait_exit(); // This won't print
}
```


---



## tick_per_inch_get()
Returns current tick per inch.           
**Prototype**
```cpp
double tick_per_inch_get();
```

**Example**
```cpp
void initialize() {
  printf("Tick Per Inch: %f\n", chassis.tick_per_inch_get());
}
```


---

