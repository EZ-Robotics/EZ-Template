---
layout: default
title: User Control
parent: Docs
nav_order: 2
---


# **User Control**
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


## tank()
Sets the drive to the left and right y axis.  
**Prototype**
```cpp
void tank();
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    chassis.tank();
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## arcade_standard()
Sets the drive to standard arcade.  Left stick is fwd/rev.  
`stick_type` is either `EZ::SPLIT` or `EZ::SINGLE`.
**Prototype**
```cpp
void arcade_standard(e_type stick_type);
``` 
**Example** 
```cpp
void opcontrol() {
  while (true) {
    chassis.arcade_standard(EZ::SPIT); // For split arcade
    // chassis.arcade_standard(EZ::SINGLE); // For single arcade
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## arcade_flipped()
Sets the drive to flipped arcade.  Right stick is fwd/rev.  
`stick_type` is either `EZ::SPLIT` or `EZ::SINGLE`.  
**Prototype**
```cpp
void arcade_flipped(e_type stick_type);
```
 
**Example** 
```cpp
void opcontrol() {
  while (true) {
    chassis.arcade_flipped(EZ::SPIT); // For split arcade
    // chassis.arcade_flipped(EZ::SINGLE); // For single arcade
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## initialize()
Runs `init_curve_sd()` and `imu_calibrate()`.    
**Prototype**
```cpp
void Drive::initialize();
```

**Example** 
```cpp
void initialize() {
  chassis.initialize();
}
```


---


## init_curve_sd()
Sets the left/right curve constants to what's on the sd card.  If the sd card is empty, creates needed files.  
**Prototype**
```cpp
void init_curve_sd();
```

**Example** 
```cpp
void initialize() {
  chassis.init_curve_sd();
}
```


---


## set_curve_defaults()
Sets the left/right curve defaults and saves new values to the sd card.  
`left` left input curve.  
`right` right input curve.  
**Prototype**
```cpp
void set_curve_default(double left, double right);
```

**Example** 
```cpp
void initialize() {
  chassis.set_curve_defaults(2, 2);
}
```


---


## set_active_brake()
Active brake runs a P loop on the drive when joysticks are within their threshold.  
`kp` proportional constant for drive.  
**Prototype**
```cpp
void set_active_brake(double kp);
```

**Example** 
```cpp
void initialize() {
  chassis.set_active_brake(0.1);
}
```


---


## toggle_modify_curve_with_controller()
Enables/disables buttons used for modifying the controller curve with the joystick.   
`toggle` true enables, false disables.  
**Prototype**
```cpp
void toggle_modify_curve_with_controller(bool toggle);
```

**Example** 
```cpp
void initialize() {
  chassis.toggle_modify_curve_with_controller(true);
}
```


---


## set_left_curve_buttons()
Sets the buttons that are used to modify the left input curve.  The example is the default.   
`decrease` a pros button.  
`increase` a pros button.  
**Prototype**
```cpp
void set_left_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);
```

**Example** 
```cpp
void initialize() {
  chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
}
```


---


## set_right_curve_buttons()
Sets the buttons that are used to modify the right input curve.  The example is the default.  
`decrease` a pros button.  
`increase` a pros button.  
**Prototype**
```cpp
void set_right_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);
```

**Example** 
```cpp
void initialize() {
  chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);
}
```


---


## left_curve_function()
Returns the input times the red curve [here](https://www.desmos.com/calculator/rcfjjg83zx).  `tank()`, `arcade_standard()`, and `arcade_flipped()` all handle this for you.  When tank is enabled, only this curve is used.  
`x` input value.  
**Prototype**
```cpp
double left_curve_function(double x);
```

**Example** 
```cpp
void opcontrol() {
  while (true) {
    int l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    int r_stick = left_curve_function(master.get_analog(ANALOG_RIGHT_Y));
    
    chassis.set_tank(l_stick, r_stick);
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## right_curve_function()
Returns the input times the red curve [here](https://www.desmos.com/calculator/rcfjjg83zx).  `tank()`, `arcade_standard()`, and `arcade_flipped()` all handle this for you.  
`x` input value.  
**Prototype**
```cpp
double right_curve_function(double x);
```

**Example** 
```cpp
void opcontrol() {
  while (true) {
    int l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    int r_stick = left_curve_function(master.get_analog(ANALOG_RIGHT_Y));
    
    chassis.set_tank(l_stick, r_stick);
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## set_joystick_threshold()
Threshold the joystick will return 0 within.   
`threshold` an integer, recommended to be less then 5.  
**Prototype**
```cpp
void set_joystick_threshold(int threshold);
```

**Example** 
```cpp
void initialize() {
  chassis.set_joystick_threshold(5);
}
```


---


## joy_thresh_opcontrol()
Runs the joystick control.  Sets the left drive to `l_stick`, and right drive to `r_stick`.  Runs active brake and joystick thresholds.    
**Prototype**
```cpp
void joy_thresh_opcontrol(int l_stick, int r_stick);
```

**Example** 
```cpp
void opcontrol() {
  while (true) {
    chassis.joy_thresh_opcontroL(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));

    pros::delay(ez::util::DELAY_TIME);
  }
  chassis.set_joystick_threshold(5);
}
```


---
## modify_curve_with_controller()
Allows the user to modify the curve with the controller.      
**Prototype**
```cpp
void modify_curve_with_controller();
```

**Example** 
```cpp
void opcontrol() {
  while (true) {
    chassis.joy_thresh_opcontroL(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));

    chassis.modify_curve_with_controller();

    pros::delay(ez::util::DELAY_TIME);
  }
  chassis.set_joystick_threshold(5);
}
```


---
