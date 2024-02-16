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
  groupId="opcontrol_tank"
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
  groupId="opcontrol_arcade_standard"
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
  groupId="opcontrol_arcade_flipped"
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




 






## Joystick Curves


 


### opcontrol_curve_sd_initialize()
Sets the left/right curve constants to what's on the SD card.  If the SD card is empty, creates needed files.  

<Tabs
  groupId="opcontrol_curve_sd_initialize"
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
  groupId="opcontrol_curve_default_sets"
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




 




 


### opcontrol_curve_buttons_toggle()
Enables/disables buttons used for modifying the controller curve with the joystick.   

`toggle` true enables, false disables  

<Tabs
  groupId="opcontrol_curve_buttons_toggle"
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



### opcontrol_curve_buttons_toggle_get()
Returns true if modifying the curve with the controller is enabled, and false if it isn't.  

<Tabs
  groupId="opcontrol_curve_buttons_toggle_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  printf("Enabled? %i\n", chassis.opcontrol_curve_buttons_toggle_get()); // Returns false
  chassis.opcontrol_curve_buttons_toggle(true);
  printf("Enabled? %i\n", chassis.opcontrol_curve_buttons_toggle_get()); // Returns true
}
```

</TabItem>


<TabItem value="proto">


```cpp
bool opcontrol_curve_buttons_toggle_get();
```



</TabItem>
</Tabs>


 


### opcontrol_curve_buttons_left_set()
Sets the buttons that are used to modify the left input curve.  The example is the default.   

`decrease` a pros button  
`increase` a pros button  

<Tabs
  groupId="opcontrol_curve_buttons_left_set"
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
  groupId="opcontrol_curve_buttons_right_set"
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
  groupId="opcontrol_curve_left"
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
  groupId="opcontrol_curve_right"
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


### opcontrol_curve_buttons_iterate()
Allows the user to modify the curve with the controller.      

<Tabs
  groupId="opcontrol_curve_buttons_iterate"
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






 ## Joystick General


### opcontrol_joystick_threshold_set()
Threshold the joystick will return 0 within.  This is useful because not all joysticks will return perfectly to 0 when let go. 

`threshold` an integer, recommended to be less then 5  
<Tabs
  groupId="opcontrol_joystick_threshold_set"
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
  groupId="opcontrol_joystick_threshld_opcontrol"
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







## Active Brake
 
### opcontrol_drive_activebrake_set()
Active brake runs a P loop on the drive when joysticks are within their threshold.  

`kp` proportional constant for drive  
<Tabs
  groupId="active_brake_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  chassis.opcontrol_drive_activebrake_set(2.0);
}
```


</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_drive_activebrake_set(double kp);
```



</TabItem>
</Tabs>


### opcontrol_drive_activebrake_get()
Returns the kP of active brake.  

<Tabs
  groupId="active_brake_get"
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
  printf("kP is: %f\n", chassis.opcontrol_drive_activebrake_get);
}
```


</TabItem>


<TabItem value="proto">


```cpp
double opcontrol_drive_activebrake_get();
```



</TabItem>
</Tabs>


## Reversing Drive

### opcontrol_drive_reverse_set()
Setting this to true reverses the drive direction during opcontrol, and false reverts it back to normal.    

`toggle` reverses or reverts drive direction.  
<Tabs
  groupId="drive_reverse_set"
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
    if (master.get_digital_new_press(DIGITAL_B)) 
      chassis.opcontrol_drive_reverse_set(!chassis.opcontrol_drive_reverse_get());

    chassis.opcontrol_tank();

    pros::delay(10);
  }
}
```


</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_drive_reverse_set(bool toggle);
```



</TabItem>
</Tabs>


### opcontrol_drive_reverse_get()
Returns `true` if the drive is currently reversed, and `false` if it isn't.  

<Tabs
  groupId="drive_reverse_get"
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
    if (master.get_digital_new_press(DIGITAL_B)) 
      chassis.opcontrol_drive_reverse_set(!chassis.opcontrol_drive_reverse_get());

    chassis.opcontrol_tank();

    pros::delay(10);
  }
}
```


</TabItem>


<TabItem value="proto">


```cpp
bool opcontrol_drive_reverse_get();
```



</TabItem>
</Tabs>