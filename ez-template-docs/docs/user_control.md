---
layout: default
title: User Control
description:  ""
---


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# **User Control**


 
## Drive Modes

### opcontrol_tank()
Sets the chassis to controller joysticks using tank control.   
Run in usercontrol.   

This passes the controller through the curve functions, but is disabled by default.    
Use opcontrol_curve_buttons_toggle() to enable it.     
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
Sets the chassis to controller joysticks using standard arcade control, where left stick is fwd/rev.    
Run in usercontrol.    

This passes the controller through the curve functions, but is disabled by default.    
Use opcontrol_curve_buttons_toggle() to enable it.    

`stick_type` ez::SINGLE or ez::SPLIT control  
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
Sets the chassis to controller joysticks using flipped arcade control, where right stick is fwd/rev.   
Run in usercontrol.   

This passes the controller through the curve functions, but is disabled by default.   
Use opcontrol_curve_buttons_toggle() to enable it.   

`stick_type` ez::SINGLE or ez::SPLIT control   
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
Initializes left and right curves with the SD card, recommended to run in initialize().     
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
Sets the default joystick curves.   

`left` left default curve  
`right` right default curve  
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
Enables/disables modifying the joystick input curves with the controller.   

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
Gets the current state of the toggle. Enables/disables modifying the joystick input curves with the controller.    

True enabled, false disabled.     
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
Sets buttons for modifying the left joystick curve.   

`decrease` a pros button enumerator  
`increase` a pros button enumerator   
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
Sets buttons for modifying the right joystick curve.  

`decrease` a pros button enumerator  
`increase` a pros button enumerator   
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
Outputs a curve from 5225A In the Zone.   

This gives more control over the robot at lower speeds.    

`x` joystick input   
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
Outputs a curve from 5225A In the Zone.   

This gives more control over the robot at lower speeds.    

`x` joystick input   
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
Sets a new threshold for the joystick.  

The joysticks wil not return a value if they are within this.   

`threshold` new threshold  
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




 

  

### opcontrol_joystick_threshold_iterate()
Runs the joystick control.  Sets the left drive to `l_stick`, and right drive to `r_stick`.  Runs active brake and joystick thresholds.    

`l_stick` left joystick value    
`r_stick` right joystick value   
<Tabs
  groupId="opcontrol_joystick_threshold_iterate"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void opcontrol() {
  chassis.opcontrol_joystick_threshold_set(5);
  while (true) {
    chassis.opcontrol_joystick_threshold_iterate(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">


```cpp
void opcontrol_joystick_threshold_iterate(int l_stick, int r_stick);
```



</TabItem>
</Tabs>











### opcontrol_joystick_practicemode_toggle()
Practice mode for driver practice that shuts off the drive if you go max speed.
 
`toggle`  true enables, false disables      
<Tabs
  groupId="opcontrol_joystick_practicemode_toggle"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
void opcontrol_joystick_practicemode_toggle(bool toggle);
```
</TabItem>
<TabItem value="example">


```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    // Toggle practice mode
    if (master.get_digital_new_press(DIGITAL_L1)) {
      chassis.opcontrol_joystick_practicemode_toggle(!chassis.opcontrol_joystick_practicemode_toggle_get());
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>




### opcontrol_joystick_practicemode_toggle_get()
Gets current state of the toggle.   

True is enabled, false is disabled.    
<Tabs
  groupId="opcontrol_joystick_practicemode_toggle_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>
<TabItem value="proto">

```cpp
ool opcontrol_joystick_practicemode_toggle_get();
```
</TabItem>
<TabItem value="example">


```cpp
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    // Toggle practice mode
    if (master.get_digital_new_press(DIGITAL_L1)) {
      chassis.opcontrol_joystick_practicemode_toggle(!chassis.opcontrol_joystick_practicemode_toggle_get());
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>






## Active Brake
 
### opcontrol_drive_activebrake_set()
Runs a PID loop on the drive when the joysticks are released.  

`kp` proportional term   
`ki` integral term  
`kd` derivative term   
`start_i` error threshold to start integral    
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
void opcontrol_drive_activebrake_set(double kp, double ki = 0.0, double kd = 0.0, double start_i = 0.0);
```



</TabItem>
</Tabs>


### opcontrol_drive_activebrake_get()
Returns kP of active brake.  
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





### opcontrol_drive_activebrake_constants_get()
Returns all PID constants for active brake.  
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

  ez::PID::Constants consts = chassis.opcontrol_drive_activebrake_constants_get();

  printf("kP is: %f\n", consts.kP);
}
```
</TabItem>
<TabItem value="proto">

```cpp
PID::Constants opcontrol_drive_activebrake_constants_get();
```
</TabItem>
</Tabs>


## Reversing Drive

### opcontrol_drive_reverse_set()
Reversal for drivetrain in opcontrol that flips the left and right side and the direction of the drive.       

`toggle` true if you want your drivetrain reversed and false if you do not  
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








