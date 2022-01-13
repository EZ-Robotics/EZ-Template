---
layout: default
title: Util
parent: Docs
nav_order: 8
---


# **Util**
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}


---


## controller
The pros controller is defined globally in our library as `master`.     
**Prototype**
```cpp
extern pros::Controller master();
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


## print_to_screen() 
Prints to the LLEMU.  This function handles text that's too long for a line by finding the last word and starting it on a new line, and takes `\n` to set a new line.   
`text` input string.  
`line` starting line.  
**Prototype**
```cpp
void print_to_screen(, std::string text, int line)
```

**Example 1**  
Returns:  
hello, this is line 0   
this is line 1
```cpp
void initialize() {
  ez::print_to_screen("hello, this is line 0\nthis is line 1");
}
```

**Example 2**  
Returns:  
01234567890123456789012345678901   
hello
```cpp
void initialize() {
  std::string 32char = 01234567890123456789012345678901;
  ez::print_to_screen(32char + "hello");
}
```


---


## print_ez_template() 
Prints our branding on your terimnal :D.   
**Prototype**
```cpp
void print_ez_template();
```

**Example**
```cpp
void initialize() {
  print_ez_template();
}
```


---


## sgn() 
Returns the sgn of the input.  Returns 1 if positive, -1 if negative, and 0 if 0.    
**Prototype**
```cpp
double sgn(double input);
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    printf("Sgn of Controller: %i \n", sgn(master.get_analog(ANALOG_LEFT_Y)));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


---


## clip_num() 
Checks if `input` is within range of `max` and `min`.  If it's out, this returns `max` or `min` respectively.    
**Prototype**
```cpp
double clip_num(double input, double max, double min);
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    int joy = master.get_analog(ANALOG_LEFT_Y);

    // When the joystick is between 100 and 127
    // (or -100 and -127) this will print 100 (or -100).
    printf("Clipped Controller: %i \n", clip_num(joy, 100, -100)); 
  }
}
```


---


## DELAY_TIME 
Standard delay time for loops.    
**Prototype**
```cpp
const int DELAY_TIME = 10;
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


## IS_SD_CARD
Boolean that checks if an sd card is installed.  True if there is one, false if there isn't.    
**Prototype**
```cpp
const bool IS_SD_CARD = pros::usd::is_installed();
```

**Example**
```cpp
void initialize() {
  if (!ez::util::IS_SD_CARD) 
    printf("No SD Card Found!\n");
}
```


---


