---
layout: default
title: PTO
parent: Docs
description:  ""
nav_order: 7
---


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


 


## pto_check()
Checks if the port is in the pto_list.     

`check_if_pto` a PROS motor that is in the drive
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
  pros::delay(500);

  printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]), chassis.pto_check(chassis.right_motors[1]))); // This prints 0 0
  chassis.pto_add({chassis.left_motors[1], chassis.right_motors[1]});
  printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]), chassis.pto_check(chassis.right_motors[1]))); // This prints 1 1
}
```

</TabItem>


<TabItem value="proto">

```cpp
bool pto_check(pros::Motor check_if_pto);
```


</TabItem>
</Tabs>






 


## pto_add()
Adds motors to the pto_list.  You cannot add the first index because it's used for autonomous.     

`pto_list` a vector of motors to remove from the drive
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
void initialize() {
  pros::delay(500);

  printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]), chassis.pto_check(chassis.right_motors[1]))); // This prints 0 0
  chassis.pto_add({chassis.left_motors[1], chassis.right_motors[1]});
  printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]), chassis.pto_check(chassis.right_motors[1]))); // This prints 1 1
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pto_add(std::vector<pros::Motor> pto_list);
```


</TabItem>
</Tabs>






 


## pto_remove()
Removes motors from the pto_list.      

`pto_list` a vector of motors to add back to the drive
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
  pros::delay(500);

  printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]), chassis.pto_check(chassis.right_motors[1]))); // This prints 0 0
  chassis.pto_add({chassis.left_motors[1], chassis.right_motors[1]});
  printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]), chassis.pto_check(chassis.right_motors[1]))); // This prints 1 1
  chassis.pto_remove({chassis.left_motors[1], chassis.right_motors[1]});
  printf("Check: %i %i\n", chassis.pto_check(chassis.left_motors[1]), chassis.pto_check(chassis.right_motors[1]))); // This prints 0 0

}
```

</TabItem>


<TabItem value="proto">

```cpp
void pto_remove(std::vector<pros::Motor> pto_list);
```


</TabItem>
</Tabs>






 


## pto_toggle()
Runs `pto_add` if `toggle` is true, and `pto_remove` if `toggle` is false.       

`pto_list` a vector of motors to add / remove to the drive
`toggle` boolean to add / remove motors to drive
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
void pto_intake(bool toggle) {
  pto_intake_enabled = toggle;
  chassis.pto_toggle({chassis.left_motors[1], chassis.right_motors[1]}, toggle);
  pto_intake_piston.set_value(toggle);
  if (toggle) {
    chassis.left_motors[1].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.right_motors[1].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
void pto_toggle(std::vector<pros::Motor> pto_list, bool toggle);
```


</TabItem>
</Tabs>






 

