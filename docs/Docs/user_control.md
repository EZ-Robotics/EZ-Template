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


## tank()
Sets the drive to the left and right y axis.  
**Protottype**
```cpp
void tank();
```

<details closed> <summary> <b>Example</b> </summary> <br>


```cpp
// Chassis constructor
drive chassis (
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

void opcontrol() {
  while (true) {
    chassis.tank();
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</details>


---


### arcade_standard()
**Protottype**
```cpp
void arcade_standard(e_type stick_type);
```

Example 
```cpp

```


---


### arcade_flipped()
**Protottype**
```cpp
void arcade_flipped(e_type stick_type);
```

Example 
```cpp

```


---


### init_curve_sd()
**Protottype**
```cpp
void init_curve_sd();
```

Example 
```cpp

```


---


### set_curve_defaults()
**Protottype**
```cpp
void set_curve_default(double left, double right);
```

Example 
```cpp

```


---


### set_active_brake()
**Protottype**
```cpp
void set_active_brake(double kp);
```

Example 
```cpp

```


---


### toggle_modify_curve_with_controller()
**Protottype**
```cpp
void toggle_modify_curve_with_controller(bool toggle);
```

Example 
```cpp

```


---


### set_left_curve_buttons()
**Protottype**
```cpp
void set_left_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);
```

Example 
```cpp

```


---


### set_right_curve_buttons()
**Protottype**
```cpp
void set_right_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);
```

Example 
```cpp

```


---


### left_curve_function()
**Protottype**
```cpp
double left_curve_function(double x);
```

Example 
```cpp

```


---


### right_curve_function()
**Protottype**
```cpp
double right_curve_function(double x);
```

Example 
```cpp

```


---


### set_joystick_threshold()
**Protottype**
```cpp
void set_joystick_threshold(int threshold);
```

Example 
```cpp

```