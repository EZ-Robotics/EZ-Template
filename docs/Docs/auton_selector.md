---
layout: default
title: Autonomous Selector
parent: Docs
nav_order: 5
---


# **Autonomous Selector**
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}


---

## initialize() 
Initializes the autonomous selector.  If an sd card is plugged in, the current page will set to what's on the sd card.  
**Prototype**
```cpp
void initialize();
```

**Example**
```cpp
void initialize() {
  ez::as::initialize();
}
```


---


---

## limit_switch_lcd_initialize() 
Sets external buttons to increase/decrease the current autonomous page.    
**Prototype**
```cpp
void limit_switch_lcd_initialize(pros::ADIDigitalIn* right_limit, pros::ADIDigitalIn* left_limit = nullptr);
```

**Example**
```cpp
pros::ADIDigitalIn increase('A');
pros::ADIDigitalIn decrease('B');
void initialize() {
  ez::as::initialize();
  ez::as::limit_switch_lcd_initialize(&increase, &decrease);
  // . . .
}
```


---



## shutdown() 
Wrapper for `pros::lcd::shutdown()`.    
**Prototype**
```cpp
void shutdown();
```

**Example**
```cpp
void initialize() {
  ez::as::initialize();

  // Do something

  ez::as::shutdown();
}
```


---


## autons_add();
Adds autonomous routines to the autonomous selector. Uses `ez::screen_print()` to display to the brain.  
**Prototype**
```cpp
void autons_add(std::vector<Auton> autons);
```

**Example**
```cpp
void auto1() {
  // Do something
}
void auto2() {
  // Do something
}
void auto3() {
  // Do something
}

void initialize() {
  ez::as::auton_selector.autons_add({
    Auton("Autonomous 1\nDoes Something", auto1),
    Auton("Autonomous 2\nDoes Something Else", auto2),
    Auton("Autonomous 3\nDoes Something More", auto3),
  });
}
```


---


## selected_auton_print();
Prints the current autonomous mode to the screen.    
**Prototype**
```cpp
void selected_auton_print();
```

**Example**
```cpp
void initialize() {
  ez::as::auton_selector.selected_auton_print(); 
}
```
 

---



## page_down()
Decreases the page. Best used with the lcd callback functions.   
**Prototype**
```cpp
void page_down();
```

**Example**
```cpp
void initialize() {
  pros::lcd::register_btn0_cb(ez::as::page_down);
  pros::lcd::register_btn2_cb(ez::as::page_up);
}
```


---


## page_up()
Increases the page. Best used with the lcd callback functions  
**Prototype**
```cpp
void page_down();
```

**Example**
```cpp
void initialize() {
  pros::lcd::register_btn0_cb(ez::as::page_down);
  pros::lcd::register_btn2_cb(ez::as::page_up);
}
```


---


## selected_auton_call()
Runs the current autonomous that's selected.    
**Prototype**
```cpp
void selected_auton_call();
```

**Example**
```cpp
void autonomous() {
  chassis.imu_reset(); 
  chassis.drive_sensors_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  ez::as::auton_selector.selected_auton_call(); 
}
```


---
