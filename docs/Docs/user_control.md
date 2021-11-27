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
  {-11, -5, -7}

  // Right Chassis Ports (negative port will reverse it!)
  ,{3, 2, 17}

  // IMU Port
  ,18

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  ,3.25

  // Cartridge RPM
  ,600

  // External Gear Ratio 
  ,1.66666666667
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


## init_curve_sd()
Sets the left/right curve constants to what's on the sd card.  
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
Returns the input times the red curve [here](https://www.desmos.com/calculator/rcfjjg83zx).  `tank()`, `arcade_standard()`, and `arcade_flipped()` all handle this for you.  
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