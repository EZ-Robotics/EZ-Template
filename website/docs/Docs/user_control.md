---
layout: default
title: User Control
parent: Docs
nav_order: 2
---


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# **User Control**


 

## Assumed Constructor

All code below assumes this constructor is used.  As long as the name of the constructor is `chassis`, any of the constructors can be used. 

```cpp
// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {1, -2, 3}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{-4, 5, -6}

  // IMU Port
  ,7

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,4.125

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,2.333

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  ,{1, 2}

  // Right Tracking Wheel Ports (negative port will reverse it!)
  ,{3, 4}
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,9
);

```

 
## Drivemodes

### tank()
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
    chassis.tank();
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">


```cpp
void tank();
```



</TabItem>
</Tabs>



### arcade_standard()
Sets the drive to standard arcade.  Left stick is fwd/rev.  
`stick_type` is either `EZ::SPLIT` or `EZ::SINGLE`.

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
    chassis.arcade_standard(EZ::SPIT); // For split arcade
    // chassis.arcade_standard(EZ::SINGLE); // For single arcade
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">


```cpp
void arcade_standard(e_type stick_type);
``` 


</TabItem>
</Tabs>




 


### arcade_flipped()
Sets the drive to flipped arcade.  Right stick is fwd/rev.  
`stick_type` is either `EZ::SPLIT` or `EZ::SINGLE`.  

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
    chassis.arcade_flipped(EZ::SPIT); // For split arcade
    // chassis.arcade_flipped(EZ::SINGLE); // For single arcade
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">


```cpp
void arcade_flipped(e_type stick_type);
```
 


</TabItem>
</Tabs>




 






## Joystick funcs

### initialize()
Runs `init_curve_sd()` and `imu_calibrate()`.    

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




 


### init_curve_sd()
Sets the left/right curve constants to what's on the sd card.  If the sd card is empty, creates needed files.  

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
  chassis.init_curve_sd();
}
```

</TabItem>


<TabItem value="proto">


```cpp
void init_curve_sd();
```



</TabItem>
</Tabs>




 


### set_curve_defaults()
Sets the left/right curve defaults and saves new values to the sd card.  
`left` left input curve.  
`right` right input curve.  

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
  chassis.set_curve_defaults(2, 2);
}
```

</TabItem>


<TabItem value="proto">


```cpp
void set_curve_default(double left, double right);
```



</TabItem>
</Tabs>




 


### set_active_brake()
Active brake runs a P loop on the drive when joysticks are within their threshold.  
`kp` proportional constant for drive.  

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
  chassis.set_active_brake(0.1);
}
```


</TabItem>


<TabItem value="proto">


```cpp
void set_active_brake(double kp);
```



</TabItem>
</Tabs>



 


### toggle_modify_curve_with_controller()
Enables/disables buttons used for modifying the controller curve with the joystick.   
`toggle` true enables, false disables.  

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
  chassis.toggle_modify_curve_with_controller(true);
}
```

</TabItem>


<TabItem value="proto">


```cpp
void toggle_modify_curve_with_controller(bool toggle);
```



</TabItem>
</Tabs>




 


### set_left_curve_buttons()
Sets the buttons that are used to modify the left input curve.  The example is the default.   
`decrease` a pros button.  
`increase` a pros button.  

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
  chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
}
```

</TabItem>


<TabItem value="proto">


```cpp
void set_left_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);
```



</TabItem>
</Tabs>




 


### set_right_curve_buttons()
Sets the buttons that are used to modify the right input curve.  The example is the default.  
`decrease` a pros button.  
`increase` a pros button.  

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
  chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);
}
```


</TabItem>


<TabItem value="proto">


```cpp
void set_right_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);
```


</TabItem>
</Tabs>




 


### left_curve_function()
Returns the input times the red curve [here](https://www.desmos.com/calculator/rcfjjg83zx).  `tank()`, `arcade_standard()`, and `arcade_flipped()` all handle this for you.  When tank is enabled, only this curve is used.  
`x` input value.  

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
    int l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    int r_stick = left_curve_function(master.get_analog(ANALOG_RIGHT_Y));
    
    chassis.set_tank(l_stick, r_stick);
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">


```cpp
double left_curve_function(double x);
```



</TabItem>
</Tabs>



 


### right_curve_function()
Returns the input times the red curve [here](https://www.desmos.com/calculator/rcfjjg83zx).  `tank()`, `arcade_standard()`, and `arcade_flipped()` all handle this for you.  
`x` input value.  

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
    int l_stick = left_curve_function(master.get_analog(ANALOG_LEFT_Y));
    int r_stick = left_curve_function(master.get_analog(ANALOG_RIGHT_Y));
    
    chassis.set_tank(l_stick, r_stick);
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">


```cpp
double right_curve_function(double x);
```



</TabItem>
</Tabs>





 


### set_joystick_threshold()
Threshold the joystick will return 0 within.   
`threshold` an integer, recommended to be less then 5.  

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
  chassis.set_joystick_threshold(5);
}
```


</TabItem>


<TabItem value="proto">


```cpp
void set_joystick_threshold(int threshold);
```



</TabItem>
</Tabs>




 


### joy_thresh_opcontrol()
Runs the joystick control.  Sets the left drive to `l_stick`, and right drive to `r_stick`.  Runs active brake and joystick thresholds.    

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
    chassis.joy_thresh_opcontroL(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));

    pros::delay(ez::util::DELAY_TIME);
  }
  chassis.set_joystick_threshold(5);
}
```


</TabItem>


<TabItem value="proto">


```cpp
void joy_thresh_opcontrol(int l_stick, int r_stick);
```



</TabItem>
</Tabs>




 
### modify_curve_with_controller()
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
    chassis.joy_thresh_opcontroL(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));

    chassis.modify_curve_with_controller();

    pros::delay(ez::util::DELAY_TIME);
  }
  chassis.set_joystick_threshold(5);
}
```

</TabItem>


<TabItem value="proto">


```cpp
void modify_curve_with_controller();
```



</TabItem>
</Tabs>




 
