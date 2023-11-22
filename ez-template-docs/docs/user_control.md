---
layout: default
title: User Control
parent: Docs
description:  ""
nav_order: 2
---


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# **User Control**


 
## Drive Modes

### opcontrol_tank()
Sets the drive to the left and right y axis.  

<Tabs
  groupId="ex15"
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
void opcontrol_tank();
```



</TabItem>
</Tabs>



### opcontrol_arcade_standard()
Sets the drive to standard arcade.  Left stick is fwd/rev.  

`stick_type` is either `ez::SPLIT` or `ez::SINGLE`

<Tabs
  groupId="ex1"
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
    chassis.opcontrol_arcade_standard(ez::SPIT); // For split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE); // For single arcade
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_arcade_standard(e_type stick_type);
``` 


</TabItem>
</Tabs>




 


### opcontrol_arcade_flipped()
Sets the drive to flipped arcade.  Right stick is fwd/rev.  

`stick_type` is either `ez::SPLIT` or `ez::SINGLE`  

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
    chassis.opcontrol_arcade_flipped(ez::SPIT); // For split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE); // For single arcade
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_arcade_flipped(e_type stick_type);
```
 


</TabItem>
</Tabs>




 






## Joystick Functions

### initialize()
Runs `opcontrol_curve_sd_initialize()` and `drive_imu_calibrate()`.    

<Tabs
  groupId="ex3"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.initialize();
}
```


</TabItem>


<TabItem value="proto">


```cpp
void Drive::initialize();
```



</TabItem>
</Tabs>




 


### opcontrol_curve_sd_initialize()
Sets the left/right curve constants to what's on the SD card.  If the SD card is empty, creates needed files.  

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
  chassis.opcontrol_curve_sd_initialize();
}
```

</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_curve_sd_initialize();
```



</TabItem>
</Tabs>




 


### opcontrol_curve_default_sets()
Sets the left/right curve defaults and saves new values to the SD card.  

`left` left input curve  
`right` right input curve  

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
void initialize() {
  chassis.opcontrol_curve_default_sets(2, 2);
}
```

</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_curve_default_set(double left, double right);
```



</TabItem>
</Tabs>




 


### opcontrol_drive_activebrake_set()
Active brake runs a P loop on the drive when joysticks are within their threshold.  

`kp` proportional constant for drive  

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
void initialize() {
  chassis.opcontrol_drive_activebrake_set(0.1);
}
```


</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_drive_activebrake_set(double kp);
```



</TabItem>
</Tabs>



 


### opcontrol_curve_buttons_toggle()
Enables/disables buttons used for modifying the controller curve with the joystick.   

`toggle` true enables, false disables  

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
void initialize() {
  chassis.opcontrol_curve_buttons_toggle(true);
}
```

</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_curve_buttons_toggle(bool toggle);
```



</TabItem>
</Tabs>




 


### opcontrol_curve_buttons_left_set()
Sets the buttons that are used to modify the left input curve.  The example is the default.   

`decrease` a pros button  
`increase` a pros button  

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
  chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
}
```

</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_curve_buttons_left_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);
```



</TabItem>
</Tabs>




 


### opcontrol_curve_buttons_right_set()
Sets the buttons that are used to modify the right input curve.  The example is the default.  

`decrease` a pros button  
`increase` a pros button  

<Tabs
  groupId="ex9"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);
}
```


</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_curve_buttons_right_set(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);
```


</TabItem>
</Tabs>




 


### opcontrol_curve_left()
Returns the input times the curve [here](https://www.desmos.com/calculator/7oyvwwpmed).  `opcontrol_tank()`, `opcontrol_arcade_standard()`, and `opcontrol_arcade_flipped()` all handle this for you.  When tank is enabled, only this curve is used.  

`x` input value  

<Tabs
  groupId="ex10"
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
double opcontrol_curve_left(double x);
```



</TabItem>
</Tabs>



 


### opcontrol_curve_right()
Returns the input times the curve [here](https://www.desmos.com/calculator/7oyvwwpmed).  `opcontrol_tank()`, `opcontrol_arcade_standard()`, and `opcontrol_arcade_flipped()` all handle this for you.  

`x` input value  

<Tabs
  groupId="ex11"
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
double opcontrol_curve_right(double x);
```



</TabItem>
</Tabs>





 


### opcontrol_joystick_threshold_set()
Threshold the joystick will return 0 within.  This is useful because not all joysticks will return perfectly to 0 when let go. 

`threshold` an integer, recommended to be less then 5  
<Tabs
  groupId="ex12"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.opcontrol_joystick_threshold_set(5);
}
```


</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_joystick_threshold_set(int threshold);
```



</TabItem>
</Tabs>




 


### opcontrol_joystick_threshold_opcontrol()
Runs the joystick control.  Sets the left drive to `l_stick`, and right drive to `r_stick`.  Runs active brake and joystick thresholds.    

`l_stick` left joystick value
`r_stick` right joystick value
<Tabs
  groupId="ex13"
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
    chassis.opcontrol_joystick_threshold_opcontrol(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));

    pros::delay(ez::util::DELAY_TIME);
  }
  chassis.opcontrol_joystick_threshold_set(5);
}
```


</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_joystick_threshold_opcontrol(int l_stick, int r_stick);
```



</TabItem>
</Tabs>




 
### opcontrol_curve_buttons_iterate()
Allows the user to modify the curve with the controller.      

<Tabs
  groupId="ex14"
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
    chassis.opcontrol_joystick_threshold_opcontrol(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));

    chassis.opcontrol_curve_buttons_iterate();

    pros::delay(ez::util::DELAY_TIME);
  }
  chassis.opcontrol_joystick_threshold_set(5);
}
```

</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_curve_buttons_iterate();
```



</TabItem>
</Tabs>




 
