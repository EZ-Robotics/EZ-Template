---
layout: default
title: Util
parent: Docs
nav_order: 6
---


# **Util**
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}


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


## print_to_screen() 
Prints to the LLEMU.  This function handles text that's too long for a line, and takes `\n` for a new line.   
`text` input string.  
`line` starting line.  
**Prototype**
```cpp
void print_to_screen(, std::string text, int line)
```

**Example**
```cpp
void initialize() {
  ez::print_to_screen("hello, this is line 0\nthis is line 1");
}
```


---


## sgn() 
Returns the sgn of the input.  Returns 1 if positive, -1 if negative, and 0 if 0.    
**Prototype**
```cpp
void sgn(double input);
```

**Example**
```cpp
void opcontrol() {
  while (true) {
    printf("Sgn of Controller: %i \n", sgn(master.get_analog(ANALOG_LEFT_Y)));

    pros::delay(10);
  }
}
```


---


## clip_num() 
Checks if `input` is within range of `max` and `min`.  If it's out, this returns `max` or `min` respectively.    
**Prototype**
```cpp
void clip_num(double input, double max, double min);
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
