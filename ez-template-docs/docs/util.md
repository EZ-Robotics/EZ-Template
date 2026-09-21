---
layout: default
title: Util
description:  chat, util is short for utility if you're new here.  we're just using slang
---



import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';



## controller
The pros controller is defined globally in our library as `master`.     
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
void opcontrol() {
  while (true) {
    int l_stick = chassis.opcontrol_curve_left(master.get_analog(ANALOG_LEFT_Y));
    int r_stick = chassis.opcontrol_curve_right(master.get_analog(ANALOG_RIGHT_Y));
    
    chassis.drive_set(l_stick, r_stick);
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
extern pros::Controller master;
```



</TabItem>
</Tabs>


 



 


## screen_print() 
Prints to the LLEMU.  This function handles text that's too long for a line by finding the last word and starting it on a new line, and takes `\n` to set a new line.   

There are 8 lines, 0 through 7, and a line holds 38 characters.  If the text wraps past line 7, the last line is cut off and ends in `...`, and the lines above it are left alone.  A `line` outside of 0 to 7 does nothing, and an empty string clears that line.   

`text` input string   
`line` starting line, 0 through 7.  Defaults to 0     
<Tabs
  groupId="ex3"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
    { label: 'Example 2',  value: 'ex2', },
    ]}>

<TabItem value="example">

**Returns:**  
  hello, this is line 0   
  this is line 1


```cpp
void initialize() {
  ez::screen_print("hello, this is line 0\nthis is line 1");
}
```

</TabItem>


<TabItem value="proto">

```cpp
void screen_print(std::string text, int line = 0);
```



</TabItem>


<TabItem value="ex2">

**Returns:**  
  01234567890123456789012345678901234567   
  hello


```cpp
void initialize() {
  std::string digits = "01234567890123456789012345678901234567";  // 38 characters, a full line
  ez::screen_print(digits + "hello", 2);                          // "hello" doesn't fit, so it goes on line 3
}
```

</TabItem>
</Tabs>





## to_string_with_precision() 
Returns a string with a specific number of decimal points.     

`input` your input value     
 `n` the amount of decimals you want to display    
<Tabs
  groupId="to_string_with_precision"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
std::string to_string_with_precision(double input, int n = 2);
```
</TabItem>
<TabItem value="example">

```cpp
void initialize() {
  // This will print 1.23
  double test = 1.23456789;
  std::cout << ez::util::to_string_with_precision(test) << std::endl;
}
```
</TabItem>
</Tabs>



 


## ez_template_print() 
Prints our branding on your terminal :D.   
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
  ez::ez_template_print();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void ez_template_print();
```



</TabItem>
</Tabs>






 


## sgn() 
Returns the sign of the input.  Returns 1 if positive, -1 if negative, and 0 if 0.    

`input` value to check the sign of
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
    printf("Sgn of Controller: %i \n", ez::util::sgn(master.get_analog(ANALOG_LEFT_Y)));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
int sgn(double input);
```



</TabItem>
</Tabs>






 


## clamp() 
Returns input restricted to min-max threshold.    

`input` your input value   
`max` the maximum input can be    
`min` the minimum input can be     
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
    int joy = master.get_analog(ANALOG_LEFT_Y);

    // When the joystick is between 100 and 127
    // (or -100 and -127) this will print 100 (or -100).
    printf("Clipped Controller: %.0f \n", ez::util::clamp(joy, 100, -100)); 
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
double clamp(double input, double max, double min);
```



</TabItem>
</Tabs>




## clamp() 
Returns input restricted to min-max threshold.  

The minimum used is negative max.      

`input` your input value     
`max` the absolute value maximum input can be    
<Tabs
  groupId="ex66"
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
    int joy = master.get_analog(ANALOG_LEFT_Y);

    // When the joystick is between 100 and 127
    // (or -100 and -127) this will print 100 (or -100).
    printf("Clipped Controller: %.0f \n", ez::util::clamp(joy, 100)); 
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
double clamp(double input, double max);
```



</TabItem>
</Tabs>






 


## DELAY_TIME 
Standard delay time for loops in ms.    
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
    chassis.opcontrol_tank();

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
const int DELAY_TIME = 10;
```



</TabItem>
</Tabs>






 


## SD_CARD_ACTIVE
Boolean that checks if an SD card is installed.  True if there is one, false if there isn't.    
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
void initialize() {
  if (!ez::util::SD_CARD_ACTIVE) 
    printf("No SD Card Found!\n");
}
```


</TabItem>


<TabItem value="proto">

```cpp
const bool SD_CARD_ACTIVE = pros::usd::is_installed();
```



</TabItem>
</Tabs>





 



















## to_deg() 
Converts radians to degrees.       

`input` your input radian    
<Tabs
  groupId="to_deg"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
double to_deg(double input);
```
</TabItem>
<TabItem value="example">

```cpp
void initialize() {
  double test = ez::util::to_deg(3.14);
  std::cout << ez::util::to_string_with_precision(test) << std::endl;
}
```
</TabItem>
</Tabs>




## to_rad() 
Converts degrees to radians.       

`input` your input degree    
<Tabs
  groupId="to_rad"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
double to_rad(double input);
```
</TabItem>
<TabItem value="example">

```cpp
void initialize() {
  double test = ez::util::to_rad(180.0);
  std::cout << ez::util::to_string_with_precision(test) << std::endl;
}
```
</TabItem>
</Tabs>




## absolute_angle_to_point() 
Returns the angle between two points.      

`itarget` target pose    
`icurrent` current pose    
<Tabs
  groupId="absolute_angle_to_point"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
double absolute_angle_to_point(pose itarget, pose icurrent);
```
</TabItem>
<TabItem value="example">

```cpp
void initialize() {
  double test = ez::util::absolute_angle_to_point({12, 12}, {0, 0});
  std::cout << ez::util::to_string_with_precision(test) << std::endl;
}
```
</TabItem>
</Tabs>





## distance_to_point() 
Returns the distance between two points.      

`itarget` target pose    
`icurrent` current pose     
<Tabs
  groupId="distance_to_point"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
double distance_to_point(pose itarget, pose icurrent);
```
</TabItem>
<TabItem value="example">

```cpp
void initialize() {
  double test = ez::util::distance_to_point({12, 12}, {0, 0});
  std::cout << ez::util::to_string_with_precision(test) << std::endl;
}
```
</TabItem>
</Tabs>


## wrap_angle() 
Constrains an angle between 180 and -180.     

`theta` input angle in degrees  
<Tabs
  groupId="wrap_angle"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
double wrap_angle(double theta);
```
</TabItem>
<TabItem value="example">

```cpp
void initialize() {
  double test = ez::util::wrap_angle(3600);
  std::cout << ez::util::to_string_with_precision(test) << std::endl;
}
```
</TabItem>
</Tabs>




## vector_off_point() 
Returns a new pose that is projected off of the current pose.     

`added` how far to project a new point   
`icurrent` point to project off of  
<Tabs
  groupId="vector_off_point"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
pose vector_off_point(double added, pose icurrent);
```
</TabItem>
<TabItem value="example">

```cpp
void initialize() {
  ez::pose test = ez::util::vector_off_point(24, {0, 0, 0});
  printf("(%.2f, %.2f, %.2f)\n", test.x, test.y, test.theta);
}
```
</TabItem>
</Tabs>



## Version macros
Macros that tell you which version of EZ-Template your code is being built against.  They're for libraries and shared code that need to work with more than one EZ-Template version.  Include `EZ-Template/api.hpp` and they're available, there's nothing to call.  

:::caution 3.x and early betas don't have these

The version macros are new in 4.0.0-beta.3, and they're not in 4.0.0-beta.1 or 4.0.0-beta.2.  On those, and on 3.x, every macro on this page is missing, so your code has to cope with that.  

:::

| Macro | What it is |
| --- | --- |
| `EZ_TEMPLATE_VERSION_MAJOR`, `EZ_TEMPLATE_VERSION_MINOR`, `EZ_TEMPLATE_VERSION_PATCH` | The version numbers |
| `EZ_TEMPLATE_VERSION_STAGE` | `EZ_TEMPLATE_STAGE_ALPHA`, `EZ_TEMPLATE_STAGE_BETA`, `EZ_TEMPLATE_STAGE_RC` or `EZ_TEMPLATE_STAGE_RELEASE` |
| `EZ_TEMPLATE_VERSION_PRERELEASE_NUM` | The `N` in `beta.N`, or `0` on a full release |
| `EZ_TEMPLATE_VERSION_PRERELEASE` | `"beta.3"`, or `""` on a full release |
| `EZ_TEMPLATE_VERSION_STRING` | `"4.0.0-beta.3"` |
| `EZ_TEMPLATE_VERSION` | One integer that sorts correctly: alpha, then beta, then rc, then release.  `4.0.0-rc.1` is less than `4.0.0` |
| `EZ_TEMPLATE_VERSION_ENCODE(major, minor, patch, stage, num)` | Builds that integer, for comparing against an exact version |
| `EZ_TEMPLATE_VERSION_AT_LEAST(major, minor, patch)` | Feature check.  Ignores the prerelease, so `4.0.0-beta.3` counts as `4.0.0` |

Minor, patch and the prerelease number must each stay under 100.  

### Checking for a version
Include EZ-Template first, then give 3.x a fallback.  3.x doesn't define the macro, so without the fallback the `#if` would fail to compile.  If the fallback comes before the include, the header's own definition still wins, but the compiler warns `macro redefined`.  

```cpp
#include "EZ-Template/api.hpp"  // before the fallback below

#ifndef EZ_TEMPLATE_VERSION_AT_LEAST
#define EZ_TEMPLATE_VERSION_AT_LEAST(major, minor, patch) 0  // 3.x
#endif

#if EZ_TEMPLATE_VERSION_AT_LEAST(4, 0, 0)
  // 4.0 or newer
#else
  // 3.x
#endif
```

Don't write `#if defined(EZ_TEMPLATE_VERSION_MAJOR) && EZ_TEMPLATE_VERSION_AT_LEAST(4, 0, 0)`.  The preprocessor still reads the second half on 3.x, where that macro doesn't exist, and stops with a syntax error.  Nest the `#if` instead.  

### Prereleases are not final
Alpha, beta and rc releases are not final.  APIs can change or be removed between them, and the final release may differ from any prerelease.  `EZ_TEMPLATE_VERSION_AT_LEAST(4, 0, 0)` is true on every 4.0.0 prerelease, so if your code depends on something that changed during the prereleases, compare against a specific stage with `EZ_TEMPLATE_VERSION_ENCODE` instead.  

```cpp
#ifdef EZ_TEMPLATE_VERSION
#if EZ_TEMPLATE_VERSION >= EZ_TEMPLATE_VERSION_ENCODE(4, 0, 0, EZ_TEMPLATE_STAGE_RC, 1)
  // 4.0.0-rc.1 or newer
#endif
#endif
```

To be told when you're building against a prerelease, add a warning:  

```cpp
#ifdef EZ_TEMPLATE_VERSION_STAGE
#if EZ_TEMPLATE_VERSION_STAGE != EZ_TEMPLATE_STAGE_RELEASE
#warning "Built against an EZ-Template prerelease, the API may change"
#endif
#endif
```

### Printing the version

```cpp
void initialize() {
  printf("EZ-Template %s\n", EZ_TEMPLATE_VERSION_STRING);  // EZ-Template 4.0.0-beta.3
}
```
