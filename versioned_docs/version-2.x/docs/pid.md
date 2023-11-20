---
layout: default
title: PID
parent: Docs
description:  ""
nav_order: 6
---


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';


## Constructors 


### PID()
Creates a PID object with constants.  Everything past kP has a default starting value, so you can just put kP.  

`p` kP  
`i` kI  
`d` kD  
`p_start_i` i will start when error is within this
`name` a string for the name of the PID
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
PID liftPID{1, 0.003, 4, 100, "Lift"};
```

</TabItem>


<TabItem value="proto">

```cpp
PID(double p, double i = 0, double d = 0, double start_i = 0, std::string name = "");
```



</TabItem>
</Tabs>  



## Functions

### set_constants()
Sets PID constants.    

`p` kP  
`i` kI  
`d` kD  
`p_start_i` i will start when error is within this
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
PID liftPID;
void initialize() {
  liftPID.set_constants(1, 0, 4);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void set_constants(double p, double i = 0, double d = 0, double p_start_i = 0);
```



</TabItem>
</Tabs>





 



### set_target()
Sets PID target.   

`target` the goal position for your subsystem  
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
PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor lift_motor(1);
void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.set_target(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.set_target(0);
    }
    lift_motor = liftPID.compute(lift_motor.get_position());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
void set_target(double input);
```



</TabItem>
</Tabs>




 



### set_exit_condition()
Sets the exit condition constants.  To disable one of the conditions, set the constants relating to it to `0`.    

`p_small_exit_time` time, in ms, before exiting `p_small_error`  
`p_small_error` small error threshold  
`p_big_exit_time` time, in ms, before exiting `p_big_error`  
`p_big_error` big error threshold  
`p_velocity_exit_time` time, in ms, for velocity to be 0  
`p_mA_timeout` time, in ms, for `is_over_current` to be true       

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
PID liftPID{1, 0.003, 4, 100, "Lift"};  
void initialize() {
  liftPID.set_exit_condition(100, 3, 500, 7, 500, 500);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void set_exit_condition(int p_small_exit_time, double p_small_error, int p_big_exit_time = 0, double p_big_error = 0, int p_velocity_exit_time = 0, int p_mA_timeout = 0);
```


</TabItem>
</Tabs>





 



### set_name()
A string that prints when exit conditions are met.  When you have multiple mechanisms using exit conditions and you're debugging, seeing which exit condition is doing what can be useful.     

`name` a string for the name of the PID
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
PID liftPID{1, 0.003, 4, 100};
void initialize() {
  liftPID.set_name("Lift");
}  
```


</TabItem>


<TabItem value="proto">

```cpp
void set_name(std::string name);
```



</TabItem>
</Tabs>



 



### compute()
Computes PID.  

`current` the current sensor value for the subsystem
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
PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor lift_motor(1);
void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.set_target(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.set_target(0);
    }
    lift_motor = liftPID.compute(lift_motor.get_position());

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
double compute(double current);
```



</TabItem>
</Tabs>



 



## Exit Conditions
Exit conditions are a series of things that need to happen for you to know your subsystem has arrived at the desired target.  

### exit_output
The `.exit_condition()` function can return any of the following variables depending on what triggered it to exit.  
```cpp
enum exit_output { RUNNING = 1,
                   SMALL_EXIT = 2,
                   BIG_EXIT = 3,
                   VELOCITY_EXIT = 4,
                   mA_EXIT = 5,
                   ERROR_NO_CONSTANTS = 6 };
```

### No Motor
Exit conditions without a motor will check if the error is small for X amount of time, if error is a little bigger for Y amount of time, or if velocity is 0 for Z amount of time, if you have constants enabled for them in [set_exit_condition()](https://ez-robotics.github.io/EZ-Template/docs/pid#set_exit_condition).

Outputs one of the `exit_output` states.  This exit condition checks `small_error`, `big_error` and `velocity` if they are enabled. 

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
PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor lift_motor(1);

void initialize() {
  liftPID.set_exit_condition(100, 3, 500, 7, 500, 500);
}

void autonomous() {
  liftPID.set_target(500);
  while (liftPID.exit_condition(true) == ez::RUNNING) {
    lift_motor = liftPID.compute(lift_motor.get_position());
    pros::delay(ez::util::DELAY_TIME);
  }

  liftPID.set_target(0);
  while (liftPID.exit_condition(true) == ez::RUNNING) {
    lift_motor = liftPID.compute(lift_motor.get_position());
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
ez::exit_output exit_condition(bool print = false);
```



</TabItem>
</Tabs>



 



### One Motor
Exit conditions with a motor will check if the error is small for X amount of time, if error is a little bigger for Y amount of time, if velocity is 0 for Z amount of time, then they will check if the motor is pulling too many amps for A amount of time, only if you have constants enabled for them in [set_exit_condition()](https://ez-robotics.github.io/EZ-Template/docs/pid#set_exit_condition).

Outputs one of the `exit_output` states.  This exit condition checks `small_error`, `big_error`, `velocity` and `mA` if they are enabled.    

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
PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor lift_motor(1);

void initialize() {
  liftPID.set_exit_condition(100, 3, 500, 7, 500, 500);
}

void autonomous() {
  liftPID.set_target(500);
  while (liftPID.exit_condition(lift_motor, true) == ez::RUNNING) {
    lift_motor = liftPID.compute(lift_motor.get_position());
    pros::delay(ez::util::DELAY_TIME);
  }

  liftPID.set_target(0);
  while (liftPID.exit_condition(lift_motor, true) == ez::RUNNING) {
    lift_motor = liftPID.compute(lift_motor.get_position());
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
ez::exit_output exit_condition(pros::Motor sensor, bool print = false);
```



</TabItem>
</Tabs>


 



### Multiple Motors
This checks the same thing as one motor, except it will check through multiple motors instead of 1.  If any of the motors are pulling too many amps the function will start the timer for mA. 

Outputs one of the `exit_output` states.  This exit condition checks `small_error`, `big_error`, `velocity` and `mA` if they are enabled.  When any of the motors trip `mA`, it returns `mA_EXIT`.    

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
PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor l_lift_motor(1);
pros::Motor r_lift_motor(2, true);

void set_lift(int input) {
  l_lift_motor = input;
  r_lift_motor = input;
}

void initialize() {
  liftPID.set_exit_condition(100, 3, 500, 7, 500, 500);
}

void autonomous() {
  liftPID.set_target(500);
  while (liftPID.exit_condition({r_lift_motor, l_lift_motor}, true) == ez::RUNNING) {
    set_lift(liftPID.compute(lift_motor.get_position()));
    pros::delay(ez::util::DELAY_TIME);
  }

  liftPID.set_target(0);
  while (liftPID.exit_condition({r_lift_motor, l_lift_motor}, true) == ez::RUNNING) {
    set_lift(liftPID.compute(lift_motor.get_position()));
    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
ez::exit_output exit_condition(std::vector<pros::Motor> sensor, bool print = false);
```



</TabItem>
</Tabs>
