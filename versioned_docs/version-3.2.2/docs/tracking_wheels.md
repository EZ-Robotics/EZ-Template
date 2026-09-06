---
layout: default
title: Tracking Wheels
description:  wheels that track the ground
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';



## Constructors


### Rotation Sensor
Creates a new tracking wheel with a Rotation sensor.  

`port` the port your Rotation sensor is plugged into    
`wheel_diameter` assumed inches, this is the diameter of your wheel  
`distance_to_center` the distance to the center of your robot, this is used for tracking  
`ratio` the gear ratio of your tracking wheel. if it's not geared, this should be 1.0  
<Tabs
  groupId="constructor_adi_encoder_expander"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
tracking_wheel(int port, double wheel_diameter, double distance_to_center = 0.0, double ratio = 1.0);
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
```
</TabItem>
</Tabs>



### ADI Encoder
Creates a new tracking wheel with an ADI Encoder.

`ports` `{'A', 'B'}` make the encoder negative if reversed   
`wheel_diameter` assumed inches, this is the diameter of your wheel  
`distance_to_center` the distance to the center of your robot, this is used for tracking  
`ratio` the gear ratio of your tracking wheel. if it's not geared, this should be 1.0  
<Tabs
  groupId="constructor_adi_encoder"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
tracking_wheel(std::vector<int> ports, double wheel_diameter, double distance_to_center = 0.0, double ratio = 1.0);
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);
```
</TabItem>
</Tabs>





### ADI Encoder 3-wire Expander
Creates a new tracking wheel with an ADI Encoder plugged into a 3-wire expander.

`smart_port` the smart port your ADI Expander is plugged into  
`ports` `{'A', 'B'}` make the encoder negative if reversed   
`wheel_diameter` assumed inches, this is the diameter of your wheel  
`distance_to_center` the distance to the center of your robot, this is used for tracking  
`ratio` the gear ratio of your tracking wheel. if it's not geared, this should be 1.0  
<Tabs
  groupId="constructor_adi_encoder_expander"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
tracking_wheel(int smart_port, std::vector<int> ports, double wheel_diameter, double distance_to_center = 0.0, double ratio = 1.0);
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, {'C', 'D'}, 2.75, 4.0);
```
</TabItem>
</Tabs>









## Getters

### get()
Returns how far the wheel has traveled in inches.  
<Tabs
  groupId="get_tracker"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double get();
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.get()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>





### get_raw()
Returns the raw sensor value.   
<Tabs
  groupId="get_raw"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double get_raw();
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.get_raw()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>




### distance_to_center_get()
Returns the distance to the center of the robot. 
<Tabs
  groupId="distance_to_center_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double distance_to_center_get();
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.distance_to_center_get()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>




### distance_to_center_flip_get()
Returns if the distance to center is flipped or not.  False is not, true is.  
<Tabs
  groupId="distance_to_center_flip_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double distance_to_center_flip_get();
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.distance_to_center_flip_get()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>





### ticks_per_inch()
Returns the constant for how many ticks is 1 inch.  
<Tabs
  groupId="ticks_per_inch"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double ticks_per_inch();
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.ticks_per_inch()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>



 
 


### ticks_per_rev_get()
Returns the amount of ticks per revolution of your sensor.  
<Tabs
  groupId="ticks_per_rev_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double ticks_per_rev_get();
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.ticks_per_rev_get()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>






### ratio_get()
Returns the gear ratio for your tracking wheel.    
<Tabs
  groupId="ratio_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double ratio_get();
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.ratio_get()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>




### wheel_diameter_get()
Returns the diameter of your wheel.    
<Tabs
  groupId="wheel_diameter_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
double wheel_diameter_get();
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.wheel_diameter_get()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>
















## Setters


### reset()
Returns the diameter of your wheel.    
<Tabs
  groupId="reset"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void reset();
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void initialize() {
    right_tracker.reset();
}
```
</TabItem>
</Tabs>


### distance_to_center_set()
Sets the distance to the center of the robot.   

`input` distance to center of your robot in inches   
<Tabs
  groupId="distance_to_center_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void distance_to_center_set(double input);
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  right_tracker.distance_to_center_set(2.0);  // Update center distance
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.distance_to_center_get()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>





### distance_to_center_flip_set()
Sets the distance to the center to be flipped to negative or not.    

`input` flips distance to center to negative. false leaves it alone, true flips it 
<Tabs
  groupId="distance_to_center_flip_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void distance_to_center_flip_set(bool input);
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  right_tracker.distance_to_center_flip_set(true);  // Make center distance negative
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.wheel_diameter_get()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>



### ticks_per_rev_set()
Sets the amount of ticks per revolution of your sensor.    

This is useful for custom encoders.      

`input` ticks per revolution
<Tabs
  groupId="ticks_per_rev_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void ticks_per_rev_set(double input);
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  right_tracker.ticks_per_rev_set(36000);  // Set ticks per rev to 36000
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.ticks_per_rev_get()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>





### ratio_set()
Sets the gear ratio for your tracking wheel.      

`input` gear ratio of tracking wheel
<Tabs
  groupId="ratio_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void ratio_set(double input);
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  right_tracker.ratio_set(2.0);  // Set ratio to 2:1
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.ratio_get()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>




### wheel_diameter_set()
Sets the diameter of your wheel.       

`input` wheel diameter
<Tabs
  groupId="wheel_diameter_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="proto">

```cpp
void wheel_diameter_set(double input);
```
</TabItem>
<TabItem value="example">

```cpp
ez::tracking_wheel right_tracker(1, 2.75, 4.0);
void opcontrol() {
  right_tracker.wheel_diameter_set(3.25);  // Update wheel diameter
  while (true) {
    chassis.opcontrol_tank();  // Tank control

    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(to_string(right_tracker.wheel_diameter_get()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
```
</TabItem>
</Tabs>

