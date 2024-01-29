---
layout: default
title: Piston
parent: Docs
description:  ""
nav_order: 4
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';



## Constructors


### Piston()
Creates a new Piston object.  

`input_port` ADI port the solenoid is plugged into.  
`default_state` sets the default state of the piston.    
<Tabs
  groupId="piston_no_expander"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::Piston left_wing('A');
```

</TabItem>

<TabItem value="proto">

```cpp
Piston(int input_port, bool default_state = false);
```

</TabItem>
</Tabs>



### Piston()
Creates a new Piston object where the solenoid is plugged into a 3 wire expander.  

`input_port` ADI port the solenoid is plugged into.  
`default_state` sets the default state of the piston.    
<Tabs
  groupId="slew_backward_constant_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::Piston left_wing('A', 1);
```

</TabItem>

<TabItem value="proto">

```cpp
Piston(int input_port, int expander_smart_port, bool default_state = false);
```

</TabItem>
</Tabs>


## Setters

### set()
Sets the piston to fire.  

`input` boolean to fire the piston.       
<Tabs
  groupId="piston"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::Piston left_wing('A');
void autonomous() {
  left_wing.set(true);
}
```

</TabItem>

<TabItem value="proto">

```cpp
void set(bool input);
```

</TabItem>
</Tabs>





### button_toggle()
Toggles the piston back and forward with a single button during opcontrol.

`toggle` an input button.       
<Tabs
  groupId="button_toggle"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::Piston left_wing('A');
void opcontrol() {
  while (true) {
    left_wing.button_toggle(master.get_digital(DIGITAL_X));

    pros::delay(10);
  }
}
```

</TabItem>

<TabItem value="proto">

```cpp
void button_toggle(int toggle);
```

</TabItem>
</Tabs>



### buttons()
Sets the piston to go in and out with 2 buttons during opcontrol.  

`active` the button to activate the piston.   
`deactive` the button to retract the piston.         
<Tabs
  groupId="buttons"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::Piston left_wing('A');
void opcontrol() {
  while (true) {
    left_wing.buttons(master.get_digital(DIGITAL_L1), master.get_digital(DIGITAL_L2));

    pros::delay(10);
  }
}
```

</TabItem>

<TabItem value="proto">

```cpp
void buttons(int active, int deactive);
```

</TabItem>
</Tabs>


## Getter

### get()
Gets the state of the piston.       
<Tabs
  groupId="get_piston"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::Piston left_wing('A');
void autonomous() {
  left_wing.set(!left_wing.get());
}
```

</TabItem>

<TabItem value="proto">

```cpp
bool get();
```

</TabItem>
</Tabs>