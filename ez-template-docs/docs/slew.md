---
layout: default
title: Slew
description:  ""
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


:::note

Click [here](https://ez-robotics.github.io/EZ-Template/category/autonomous-functions) for documentation on slew for each movement in EZ-Template!

:::


## Slew Class


### slew()
Creates a slew object with constants.  Robot will start at `min_speed` and will be at the maximum set speed once `distance` is traveled.

`distance` distance to travel until maximum speed is reached.  
`minimum_speed` starting speed.  
<Tabs
  groupId="slew"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::slew lift_Slew(100, 60);
```

</TabItem>


<TabItem value="proto">

```cpp
slew(double distance, int minimum_speed);
```



</TabItem>
</Tabs>  




### constants_set()
Sets slew constants.    

`distance` distance to travel until maximum speed is reached.  
`minimum_speed` starting speed.  
<Tabs
  groupId="set_constants"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
PID lift_slew;
void initialize() {
  lift_slew.constants_set(100, 50);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void constants_set(double distance, int minimum_speed);
```



</TabItem>
</Tabs>


### initialize()
Setup for slew.  Keeps track of where the starting sensor value is and what the maximum target speed is.  

`enabled` true if you want slew to run, false if you don't.  
`maximum_speed` the target speed for the robot to reach.    
`target` the distance to reach before max speed is hit.  
`current` the current sensor value.  
<Tabs
  groupId="slew_init"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
PID lift_slew;
pros::Motor lift(1);
void initialize() {
  lift_slew.constants_set(100, 50);
  lift_slew.initialize(true, 127, 500, lift.get_position());
}
```

</TabItem>


<TabItem value="proto">

```cpp
void initialize(bool enabled, double maximum_speed, double target, double current);
```



</TabItem>
</Tabs>




### iterate()
Iterates slew calculation and returns what the current max speed should be.  


`current` the current sensor value.  
<Tabs
  groupId="slew_iterate"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
PID lift_slew;
pros::Motor lift(1);
void initialize() {
  lift_slew.constants_set(100, 50);
  lift_slew.initialize(true, 127, 500, lift.get_position());
}

void autonomous() {
  while (lift.get_position() <= 500) {
    lift = lift_slew.iterate(lift.get_position();
    pros::delay(10);
  }
  lift = 0;
}
```

</TabItem>


<TabItem value="proto">

```cpp
double iterate(double current);
```



</TabItem>
</Tabs>




### output()
Returns what the current max speed should be.  

<Tabs
  groupId="slew_output"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
PID lift_slew;
pros::Motor lift(1);
void initialize() {
  lift_slew.constants_set(100, 50);
  lift_slew.initialize(true, 127, 500, lift.get_position());
}

void autonomous() {
  while (lift.get_position() <= 500) {
    lift_slew.iterate(lift.get_position();
    lift = lift_slew.output();
    pros::delay(10);
  }
  lift = 0;
}
```

</TabItem>


<TabItem value="proto">

```cpp
double output();
```



</TabItem>
</Tabs>




### enabled()
Returns if slew is currently active or not.  

<Tabs
  groupId="slew_enabled"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
PID lift_slew;
pros::Motor lift(1);
void initialize() {
  lift_slew.constants_set(100, 50);
  lift_slew.initialize(true, 127, 500, lift.get_position());
}

void autonomous() {
  printf("Slew Enabled? %f\n", lift_slew.enabled()); // Returns true
  while (lift.get_position() <= 500) {
    lift_slew.iterate(lift.get_position();
    lift = lift_slew.output();
    pros::delay(10);
  }
  lift = 0;
  printf("Slew Enabled? %f\n", lift_slew.enabled()); // Returns false
}
```

</TabItem>


<TabItem value="proto">

```cpp
double output();
```



</TabItem>
</Tabs>






### speed_max_set()
Sets the max speed slew can be.   

`speed`  maximum speed the output can be
<Tabs
  groupId="speed_max_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
PID lift_slew;
pros::Motor lift(1);
void initialize() {
  lift_slew.constants_set(100, 50);
  lift_slew.initialize(true, 127, 500, lift.get_position());
}

void autonomous() {
  while (lift.get_position() <= 500) {
    if (lift.get_position() < 100) 
      lift_slew.speed_max_set(50);
    else 
      lift_slew.speed_max_set(127);
    lift_slew.iterate(lift.get_position();
    lift = lift_slew.output();
    pros::delay(10);
  }
  lift = 0;
}
```

</TabItem>


<TabItem value="proto">

```cpp
void speed_max_set(double speed);
```



</TabItem>
</Tabs>




### speed_max_set()
Returns the max speed slew can be.   
<Tabs
  groupId="speed_max_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
PID lift_slew;
pros::Motor lift(1);
void initialize() {
  lift_slew.constants_set(100, 50);
  lift_slew.initialize(true, 127, 500, lift.get_position());
}

void autonomous() {
  while (lift.get_position() <= 500) {
    if (lift.get_position() < 100) 
      lift_slew.speed_max_set(50);
    else 
      lift_slew.speed_max_set(127);
    printf("%.2f", lift_slew.speed_max_get());

    lift_slew.iterate(lift.get_position();
    lift = lift_slew.output();
    pros::delay(10);
  }
  lift = 0;
}
```

</TabItem>


<TabItem value="proto">

```cpp
double speed_max_get();
```



</TabItem>
</Tabs>

  /**
   * Sets the max speed the slew can be
   *
   * \param speed
   *        maximum speed
   */
  void speed_max_set(double speed);

  /**
   * Returns the max speed the slew can be
   */
  double speed_max_get();