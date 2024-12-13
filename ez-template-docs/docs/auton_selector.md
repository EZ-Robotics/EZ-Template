---
layout: default
title: Autonomous Selector
description:  ""
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


# **Autonomous Selector**


## initialize() 
Initializes the autonomous selector.  If an SD card is plugged in, the current page will set to what's on the SD card.  
<Tabs
  groupId="initialize_auto_selector"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  ez::as::initialize();
}
```

</TabItem>


<TabItem value="proto">


```cpp
void initialize();
```



</TabItem>
</Tabs>



## limit_switch_lcd_initialize() 
Sets external buttons to increase/decrease the current autonomous page.  

`right_limit` a button to go forward a page
`left_limit` a button to go backwards a page
<Tabs
  groupId="limit_switch_lcd_initialize"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
pros::adi::DigitalIn increase('A');
pros::adi::DigitalIn decrease('B');
void initialize() {
  ez::as::initialize();
  ez::as::limit_switch_lcd_initialize(&increase, &decrease);
  // . . .
}
```

</TabItem>


<TabItem value="proto">


```cpp
void limit_switch_lcd_initialize(pros::adi::DigitalIn* right_limit, pros::adi::DigitalIn* left_limit = nullptr);
```


</TabItem>
</Tabs>




 



## shutdown() 
Wrapper for `pros::lcd::shutdown()` [found here](https://pros.cs.purdue.edu/v5/api/cpp/llemu.html#shutdown).    
<Tabs
  groupId="shutdown"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  ez::as::initialize();

  // Do something

  ez::as::shutdown();
}
```

</TabItem>


<TabItem value="proto">


```cpp
void shutdown();
```



</TabItem>
</Tabs>




 


## autons_add();
Adds autonomous routines to the autonomous selector. Uses `ez::print_to_screen()` to display to the brain.  

`autons` accepts an object of a string and a function
<Tabs
  groupId="autons_add"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

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

</TabItem>


<TabItem value="proto">


```cpp
void autons_add(std::vector<Auton> autons);
```



</TabItem>
</Tabs>






## selected_auton_print();
Prints the current autonomous mode to the screen.    
<Tabs
  groupId="selected_auton_print"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  ez::as::auton_selector.selected_auton_print(); 
}
```
</TabItem>


<TabItem value="proto">


```cpp
void selected_auton_print();
```



</TabItem>
</Tabs>


 





## page_down()
Decreases the page number. Best used with the lcd callback functions.   
<Tabs
  groupId="page_down"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  pros::lcd::register_btn0_cb(ez::as::page_down);
  pros::lcd::register_btn2_cb(ez::as::page_up);
}
```

</TabItem>


<TabItem value="proto">


```cpp
void page_down();
```



</TabItem>
</Tabs>







## page_up()
Increases the page number. Best used with the lcd callback functions  
<Tabs
  groupId="page_up"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

**Example**
```cpp
void initialize() {
  pros::lcd::register_btn2_cb(ez::as::page_up);
}
```

</TabItem>


<TabItem value="proto">


```cpp
void page_down();
void page_up();
```



</TabItem>
</Tabs>







## selected_auton_call()
Runs the current autonomous that's selected.    
<Tabs
  groupId="selected_auton_call"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  ez::as::auton_selector.selected_auton_call(); 
}
```

</TabItem>


<TabItem value="proto">


```cpp
void selected_auton_call();
```



</TabItem>
</Tabs>




## enabled()
Returns true if the auton selector is enabled and false if it isn't.  
<Tabs
  groupId="enabled"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  printf("Enabled? %i\n", ez::as::enabled()); // Returns false
  ez::as::initialize();
  printf("Enabled? %i\n", ez::as::enabled()); // Returns true
}
```

</TabItem>


<TabItem value="proto">


```cpp
bool ez::as::enabled();
```



</TabItem>
</Tabs>



