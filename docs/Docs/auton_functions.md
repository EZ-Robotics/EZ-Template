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


## set_drive_pid()
Sets the drive to go forward using PID and heading correction.  
`target` is in inches.  
`speed` is -127 to 127.  It's recommended to keep this at 110.  
`slew_on` will ramp the drive up.
`toggle_heading` will disable heading correction when false. 
**Prototype**
```cpp
void set_drive_pid(double target, int speed, bool slew_on = false, bool toggle_heading = true);
```

**Example**
```cpp
void autonomous() {
  chassis.reset_gyro(); 
  chassis.reset_drive_sensor(); 
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); 

  chassis.set_drive_pid(24, 110, true);
  chassis.wait_drive();
}
```


---


## set_turn_pid()
Sets the drive to turn using PID.  
`target` is in degrees.  
`speed` is -127 to 127.    
**Prototype**
```cpp
void set_turn_pid(double target, int speed);
```

**Example**
```cpp
void autonomous() {
  chassis.reset_gyro(); 
  chassis.reset_drive_sensor(); 
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); 

  chassis.set_turn_pid(90, 110);
  chassis.wait_drive();
}
```


---


## set_swing_pid()
Sets the robot to swing using PID.  The robot will turn using one side of the drive, either the left or right.  
`type` is either `ez::LEFT_SWING` or `ez::RIGHT_SWING`.  
`target` is in degrees.  
`speed` is -127 to 127.        
**Prototype**
```cpp
void set_swing_pid(e_swing type, double target, int speed);
```

**Example**
```cpp
void autonomous() {
  chassis.reset_gyro(); 
  chassis.reset_drive_sensor(); 
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); 

  chassis.set_swing_pid(ez::LEFT_SWING, 45, 110);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, 110);
  chassis.wait_drive();
}
```


---


## wait_drive()
Locks the code in place until the drive has settled.  This uses the exit conditions from the PID class.      
**Prototype**
```cpp
void wait_drive();
```

**Example**
```cpp
void autonomous() {
  chassis.reset_gyro(); 
  chassis.reset_drive_sensor(); 
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); 

  chassis.set_turn_pid(90, 110);
  chassis.wait_drive();

  chassis.set_turn_pid(0, 110);
  chassis.wait_drive();
}
```


---



## wait_until()
Locks the code in place until the drive has passed the input parameter.  This uses the exit conditions from the PID class.           
**Prototype**
```cpp
void wait_until(double target);
```

**Example**
```cpp
void autonomous() {
  chassis.reset_gyro(); 
  chassis.reset_drive_sensor(); 
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); 

  chassis.set_drive_pid(48, 110);
  chassis.wait_until(24);
  chassis.set_max_speed(40);
  chassis.wait_drive();
}
```


---



## reset_pid_targets()
Resets all drive PID targets to 0.       
**Prototype**
```cpp
void reset_pid_targets();
```

**Example**
```cpp
void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}
```


---



## set_angle()
Sets the angle of the robot.  This is useful when your robot is setup in at an unconventional angle and you want 0 to be when you're square with the field.         
**Prototype**
```cpp
void set_angle(double angle);
```

**Example**
```cpp
void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  chassis.set_angle(45);

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
}
```


---



## set_max_speed()
Sets the max speed of the drive. 
`speed` an integer between -127 and 127.     
**Prototype**
```cpp
void set_max_speed(int speed);
```

**Example**
```cpp
void autonomous() {
  chassis.reset_gyro(); 
  chassis.reset_drive_sensor(); 
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); 

  chassis.set_drive_pid(48, 110);
  chassis.wait_until(24);
  chassis.set_max_speed(40);
  chassis.wait_drive();
}
```


---


## set_pid_constants()
*Note: this function was changed with 2.0.1*  
Set PID constants.  Below are the defaults.  
`pid` either `&chassis.headingPID`, `&chassis.forward_drivePID`, `&chassis.backward_drivePID`, `&chassis.turnPID`, or `&chassis.swingPID`.   
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
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15;
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}
```


---


## set_slew_min_power()
Sets the starting speed for slew, with the ability to have different constants for forward and reverse.  Below is the defaults.   
`fwd` integer between -127 and 127.
`rev` integer between -127 and 127.  
**Prototype**
```cpp
void set_slew_min_power(int fwd, int rev);
```

**Example**
```cpp
void initialize() {
  chassis.set_slew_min_power(80, 80);
}
```


---


## set_slew_distance()
Sets the distance the drive will slew for, with the ability to have different constants for forward and reverse.  Input is inches.  Below is the defaults.   
`fwd` a distance in inches.   
`rev` a distance in inches.   
**Prototype**
```cpp
void set_slew_distance (int fwd, int rev);
```

**Example**
```cpp
void initialize() {
  chassis.set_slew_min_distance(7, 7);
}
```


---


## set_exit_condition()
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
void set_exit_condition(exit_condition_ &type, int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout);

```

**Example**
```cpp
void initialize() {
  chassis.set_exit_condition(chassis.turn_exit,  100, 3,  500, 7,   500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3,  500, 7,   500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80,  50, 300, 150, 500, 500);
}
```


---


## set_swing_min()
Sets the max power of the drive when the robot is within `start_i`.  This only enalbes when `i` is enabled, and when the movement is greater then `start_i`.        
**Prototype**
```cpp
void set_swing_min(int min);

```

**Example**
```cpp
void autonomous() {
  chassis.set_swing_min(30);

  chassis.set_swing_pid(45, 110);
  chassis.wait_drive();
}
```


---


## set_turn_min()
Sets the max power of the drive when the robot is within `start_i`.  This only enalbes when `i` is enabled, and when the movement is greater then `start_i`.        
**Prototype**
```cpp
void set_turn_min(int min);

```

**Example**
```cpp
void autonomous() {
  chassis.set_turn_min(30);

  chassis.set_turn_pid(45, 110);
  chassis.wait_drive();
}
```


---


## get_swing_min()
Returns swing min.         
**Prototype**
```cpp
int get_swing_min();

```

**Example**
```cpp
void autonomous() {
  chassis.set_swing_min(30);

  printf("Swing Min: %i", chassis.get_swing_min());
}
```


---


## get_turn_min()
Returns turn min.         
**Prototype**
```cpp
int get_turn_min();

```

**Example**
```cpp
void autonomous() {
  chassis.set_turn_min(30);

  printf("Turn Min: %i", chassis.get_turn_min());
}
```


---


## interfered
Boolean that returns true when `wait_drive()` or `wait_until()` exit with velocity or is_over_current.      
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
     chassis.set_drive_pid(-12, 127);
     chassis.wait_drive();

     // If failsafed...
     if (chassis.interfered) {
       chassis.reset_drive_sensor();
       chassis.set_drive_pid(-2, 20);
       pros::delay(1000);
     }
     // If robot successfully drove back, return
     else {
       return;
     }
   }
 }

void auto1() {
  chassis.set_drive_pid(24, 110, true);
  chassis.wait_drive();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.set_turn_pid(90, 90);
  chassis.wait_drive();
}
```


---


## set_mode()
Sets the current mode of the drive.  Accepts `ez::DISABLE`, `ez::SWING`, `ez::TURN`, `ez::DRIVE`.           
**Prototype**
```cpp
void set_mode(e_mode p_mode);
```

**Example**
```cpp
void autonomous() {
  chassis.set_drive_pid(12, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_mode(ez::DISABLE); // Disable drive

  chassis.set_tank(-127, -127); // Run drive motors myself
  pros::delay(2000);
  chassis.set_tank(0, 0);
}
```


---


## get_mode()
Returns the current drive mode.  Returns `ez::DISABLE`, `ez::SWING`, `ez::TURN`, `ez::DRIVE`.           
**Prototype**
```cpp
e_mode get_mode();
```

**Example**
```cpp
void autonomous() {
  chassis.set_drive_pid(12, DRIVE_SPEED);
  chassis.wait_drive();

  if (chassis.interfered)
    chassis.set_mode(ez::DISABLE);
  
  if (chassis.get_mode() == ez::DISABLE) {
    chassis.set_tank(-127, -127); // Run drive motors myself
    pros::delay(2000);
    chassis.set_tank(0, 0);
  }
}
```


---


## toggle_auto_drive()
Enables/disables the drive from moving in autonomous.  This is useful for debugging and checking PID variables.  True enables, false disables.         
**Prototype**
```cpp
void toggle_auto_drive(bool toggle);
```

**Example**
```cpp
void autonomous() {
  chassis.set_drive_pid(12, DRIVE_SPEED);
  chassis.wait_drive();

  toggle_auto_drive(false); // Disable drive

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  while (true) {
    printf(" Left Error: %f  Right Error: %f\n", chassis.leftPID.error, chassis.rightPID.error);
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## toggle_auto_print()
Enables/disables the drive functions printing every drive motion.  This is useful when you're debugging something and don't want terminal cluttered.  True enables, false disables.          
**Prototype**
```cpp
void toggle_auto_print(bool toggle);
```

**Example**
```cpp
void autonomous() {
  chassis.set_drive_pid(12, DRIVE_SPEED); // This will print
  chassis.wait_drive(); // This will print

  toggle_auto_print(false); // Disable prints

  chassis.set_drive_pid(-12, DRIVE_SPEED); // This won't print
  chassis.wait_drive(); // This won't print
}
```


---



## get_tick_per_inch()
Returns current tick per inch.           
**Prototype**
```cpp
double get_tick_per_inch();
```

**Example**
```cpp
void initialize() {
  printf("Tick Per Inch: %f\n", chassis.get_tick_per_inch());
}
```


---

