---
layout: default
title: Autonomous Selector
parent: Docs
nav_order: 5
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


# **Autonomous Selector**


## initialize() 
Initializes the autonomous selector.  If an sd card is plugged in, the current page will set to what's on the sd card.  
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
pros::ADIDigitalIn increase('A');
pros::ADIDigitalIn decrease('B');
void initialize() {
  ez::as::initialize();
  ez::as::limit_switch_lcd_initialize(&increase, &decrease);
  // . . .
}
```

</TabItem>


<TabItem value="proto">


```cpp
void limit_switch_lcd_initialize(pros::ADIDigitalIn* right_limit, pros::ADIDigitalIn* left_limit = nullptr);
```


</TabItem>
</Tabs>




 



## shutdown() 
Wrapper for `pros::lcd::shutdown()`.    
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




 


## add_autons();
Adds autonomous routines to the autonomous selector. Uses `ez::print_to_screen()` to display to the brain.  
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
  ez::as::auton_selector.add_autons({
    Auton("Autonomous 1\nDoes Something", auto1),
    Auton("Autonomous 2\nDoes Something Else", auto2),
    Auton("Autonomous 3\nDoes Something More", auto3),
  });
}
```

</TabItem>


<TabItem value="proto">


```cpp
void add_autons(std::vector<Auton> autons);
```



</TabItem>
</Tabs>






## print_selected_auton();
Prints the current autonomous mode to the screen.    
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
  ez::as::auton_selector.print_selected_auton(); 
}
```
</TabItem>


<TabItem value="proto">


```cpp
void print_selected_auton();
```



</TabItem>
</Tabs>


 





## page_down()
Decreases the page. Best used with the lcd callback functions.   
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
Increases the page. Best used with the lcd callback functions  
<Tabs
  groupId="ex7"
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
  pros::lcd::register_btn0_cb(ez::as::page_down);
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







## call_selected_auton()
Runs the current autonomous that's selected.    
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
void autonomous() {
  chassis.reset_gyro(); 
  chassis.reset_drive_sensor(); 
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); 

  ez::as::auton_selector.call_selected_auton(); 
}
```

</TabItem>


<TabItem value="proto">


```cpp
void call_selected_auton();
```



</TabItem>
</Tabs>





