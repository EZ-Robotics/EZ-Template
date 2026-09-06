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
    int l_stick = opcontrol_curve_left(master.get_analog(ANALOG_LEFT_Y));
    int r_stick = opcontrol_curve_left(master.get_analog(ANALOG_RIGHT_Y));
    
    chassis.drive_set(l_stick, r_stick);
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
extern pros::Controller master();
```



</TabItem>
</Tabs>


 



 


## screen_print() 
Prints to the LLEMU.  This function handles text that's too long for a line by finding the last word and starting it on a new line, and takes `\n` to set a new line.   

`text` input string   
`line` starting line     
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
void screen_print(std::string text, int line)
```



</TabItem>


<TabItem value="ex2">

**Returns:**  
  01234567890123456789012345678901   
  hello


```cpp
void initialize() {
  std::string 32char = 01234567890123456789012345678901;
  ez::print_to_screen(32char + "hello", 2);
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



 


## print_ez_template() 
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
  print_ez_template();
}
```

</TabItem>


<TabItem value="proto">

```cpp
void print_ez_template();
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
    printf("Sgn of Controller: %i \n", sgn(master.get_analog(ANALOG_LEFT_Y)));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
double sgn(double input);
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
    printf("Clipped Controller: %i \n", clamp(joy, 100, -100)); 
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
`max` the maximum input can be    
`min` the absolute value maximum input can be    
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
    printf("Clipped Controller: %i \n", clamp(joy, 100)); 
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






 


## IS_SD_CARD
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
  if (!ez::util::IS_SD_CARD) 
    printf("No SD Card Found!\n");
}
```


</TabItem>


<TabItem value="proto">

```cpp
const bool IS_SD_CARD = pros::usd::is_installed();
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
double to_deg(double input);
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

`wrap_angle` input angle in degrees  
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