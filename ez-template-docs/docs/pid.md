---
layout: default
title: PID
description:  ""
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
ez::PID liftPID{1, 0.003, 4, 100, "Lift"};
```

</TabItem>


<TabItem value="proto">

```cpp
PID(double p, double i = 0, double d = 0, double start_i = 0, std::string name = "");
```



</TabItem>
</Tabs>  


## Functions


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
ez::PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor lift_motor(1);
void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.target_set(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.target_set(0);
    }
    lift_motor.move(liftPID.compute(lift_motor.get_position()));

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





### compute_error()
Computes PID based on error.  This function ignores target entirely and the user has to calculate error.  

`error` the target minus current, you calculate this yourself    
`current` the current sensor value for the subsystem
<Tabs
  groupId="compute_error"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor lift_motor(1);
void opcontrol() {
  double target = 0.0;
  double error = 0.0;
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      target = 500.0;
    }
    else if (master.get_digital(DIGITAL_L2)) {
      target = 0.0;
    }
    error = target - lift_motor.get_position();
    lift_motor.move(liftPID.compute_error(error, lift_motor.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
double compute_error(double err, double current);
```



</TabItem>
</Tabs>



## Setters

### constants_set()
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
ez::PID liftPID;
void initialize() {
  liftPID.constants_set(1, 0, 4);
}
```

</TabItem>


<TabItem value="proto">

```cpp
void constants_set(double p, double i = 0, double d = 0, double p_start_i = 0);
```



</TabItem>
</Tabs>





 



### target_set()
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
ez::PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor lift_motor(1);
void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.target_set(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.target_set(0);
    }
    lift_motor.move(liftPID.compute(lift_motor.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
void target_set(double input);
```



</TabItem>
</Tabs>




 



### exit_condition_set()
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
ez::PID liftPID{1, 0.003, 4, 100, "Lift"};  
void initialize() {
  liftPID.exit_condition_set(100, 3, 500, 7, 500, 500);
}
```


</TabItem>


<TabItem value="proto">

```cpp
void exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time = 0, double p_big_error = 0, int p_velocity_exit_time = 0, int p_mA_timeout = 0);
```


</TabItem>
</Tabs>




### velocity_sensor_secondary_toggle_set()  
Enables / disables the use of the second sensor.  True enables this and uses the secondary sensor in velocity exits, false disables.    

`toggle` sensor value for the secondary sensor  
<Tabs
  groupId="velocity_sensor_secondary_toggle_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::PID liftPID;
void initialize() {
  liftPID.velocity_sensor_secondary_toggle_set(true);  // Enable the secondary sensor
}
```

</TabItem>


<TabItem value="proto">

```cpp
void velocity_sensor_secondary_toggle_set(bool toggle);
```

</TabItem>
</Tabs>







### velocity_sensor_secondary_set()
This sets the sensor value for the secondary sensor.  The secondary sensor is used in addition to the main sensor for determining velocity exits, if enabled.      
 
`secondary_sensor` sensor value for the secondary sensor  
<Tabs
  groupId="velocity_sensor_secondary_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
...
```

</TabItem>


<TabItem value="proto">

```cpp
void velocity_sensor_secondary_set(double secondary_sensor);
```

</TabItem>
</Tabs>
















 







### velocity_sensor_main_exit_set()
Sets a threshold for the main sensors velocity.  The velocity timer will start increasing when the main sensor is within this value.  This is defaulted to `0.05`.  

`zero` double, a small threshold   
<Tabs
  groupId="velocity_sensor_main_exit_set()"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::PID liftPID;
void initialize() {
  liftPID.velocity_sensor_secondary_toggle_set(true);  // Enable the secondary sensor
}
```

</TabItem>


<TabItem value="proto">

```cpp
void velocity_sensor_main_exit_set(double zero);
```

</TabItem>
</Tabs>



### velocity_sensor_secondary_exit_set()
Sets a threshold for the secondary sensors velocity.  The velocity timer will start increasing when the secondary sensor is within this value.  This is defaulted to `0.1`.  This is only used when the secondary sensor is enabled.   

`zero` double, a small threshold   
<Tabs
  groupId="velocity_sensor_secondary_exit_set()"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::PID liftPID;
void initialize() {
  liftPID.velocity_sensor_secondary_toggle_set(true);  // Enable the secondary sensor
}
```

</TabItem>


<TabItem value="proto">

```cpp
void velocity_sensor_secondary_exit_set(double zero);
```

</TabItem>
</Tabs>

 



### name_set()
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
ez::PID liftPID{1, 0.003, 4, 100};
void initialize() {
  liftPID.name_set("Lift");
}  
```


</TabItem>


<TabItem value="proto">

```cpp
void name_set(std::string name);
```



</TabItem>
</Tabs>




## Getters
 

### target_set()
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
ez::PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor lift_motor(1);
void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.target_set(500);
      printf("%.2f\n", liftPID.target_get()); // This prints 500
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.target_set(0);
      printf("%.2f\n", liftPID.target_get()); // This prints 0
    }
    lift_motor.move(liftPID.compute(lift_motor.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
double target_set();
```



</TabItem>
</Tabs>



### velocity_sensor_secondary_toggle_get()
Returns the value for the secondary sensor.  The secondary sensor is used in addition to the main sensor for determining velocity exits, if enabled.   
<Tabs
  groupId="velocity_sensor_secondary_toggle_get()"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::PID liftPID;
void initialize() {
  printf("%i\n",liftPID.velocity_sensor_secondary_toggle_get());  // This prints 0
  liftPID.velocity_sensor_secondary_toggle_set(true);  // Enable the secondary sensor
  printf("%i\n",liftPID.velocity_sensor_secondary_toggle_get());  // This prints 1
}
```

</TabItem>


<TabItem value="proto">

```cpp
double velocity_sensor_secondary_toggle_get();
```

</TabItem>
</Tabs>








### velocity_sensor_main_exit_get()
Gets a threshold for the secondary sensors velocity.  The velocity timer will start increasing when the secondary sensor is within this value.  This is defaulted to `0.1`.    
<Tabs
  groupId="velocity_sensor_main_exit_get()"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::PID liftPID;
void initialize() {
  liftPID.velocity_sensor_secondary_toggle_set(true);  // Enable the secondary sensor
  printf("%.2f\n",liftPID.velocity_sensor_main_exit_get());  // This prints 0.05
}
```

</TabItem>


<TabItem value="proto">

```cpp
double velocity_sensor_main_exit_get();
```

</TabItem>
</Tabs>








### velocity_sensor_secondary_exit_get()
Gets a threshold for the secondary sensors velocity.  The velocity timer will start increasing when the secondary sensor is within this value.  This is defaulted to `0.1`.  This is only used when the secondary sensor is enabled.     
<Tabs
  groupId="velocity_sensor_secondary_exit_get()"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::PID liftPID;
void initialize() {
  liftPID.velocity_sensor_secondary_toggle_set(true);  // Enable the secondary sensor
  printf("%.2f\n",liftPID.velocity_sensor_secondary_exit_get());  // This prints 0.1
}
```

</TabItem>


<TabItem value="proto">

```cpp
double velocity_sensor_secondary_exit_get();
```

</TabItem>
</Tabs>






### velocity_sensor_secondary_get()
Returns if the secondary sensor is enabled or disables.  True means this is enabled and the secondary sensor is in use, false means disabled.    
<Tabs
  groupId="velocity_sensor_secondary_get()"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::PID liftPID;
void initialize() {
  liftPID.velocity_sensor_secondary_toggle_set(true);  // Enable the secondary sensor
}
```

</TabItem>


<TabItem value="proto">

```cpp
double velocity_sensor_secondary_get();
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
Exit conditions without a motor will check if the error is small for X amount of time, if error is a little bigger for Y amount of time, or if velocity is 0 for Z amount of time, if you have constants enabled for them in [exit_condition_set()](https://ez-robotics.github.io/EZ-Template/docs/pid#exit_condition_set).

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
ez::PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor lift_motor(1);

void initialize() {
  liftPID.exit_condition_set(100, 3, 500, 7, 500, 500);
}

void autonomous() {
  liftPID.target_set(500);
  while (liftPID.exit_condition(true) == ez::RUNNING) {
    lift_motor.move(liftPID.compute(lift_motor.get_position()));
    pros::delay(ez::util::DELAY_TIME);
  }

  liftPID.target_set(0);
  while (liftPID.exit_condition(true) == ez::RUNNING) {
    lift_motor.move(liftPID.compute(lift_motor.get_position()));
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
Exit conditions with a motor will check if the error is small for X amount of time, if error is a little bigger for Y amount of time, if velocity is 0 for Z amount of time, then they will check if the motor is pulling too many amps for A amount of time, only if you have constants enabled for them in [exit_condition_set()](https://ez-robotics.github.io/EZ-Template/docs/pid#exit_condition_set).

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
ez::PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor lift_motor(1);

void initialize() {
  liftPID.exit_condition_set(100, 3, 500, 7, 500, 500);
}

void autonomous() {
  liftPID.target_set(500);
  while (liftPID.exit_condition(lift_motor, true) == ez::RUNNING) {
    lift_motor.move(liftPID.compute(lift_motor.get_position()));
    pros::delay(ez::util::DELAY_TIME);
  }

  liftPID.target_set(0);
  while (liftPID.exit_condition(lift_motor, true) == ez::RUNNING) {
    lift_motor.move(liftPID.compute(lift_motor.get_position()));
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
ez::PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor l_lift_motor(1);
pros::Motor r_lift_motor(2, true);

void set_lift(int input) {
  l_lift_motor.move(input);
  r_lift_motor.move(input);
}

void initialize() {
  liftPID.exit_condition_set(100, 3, 500, 7, 500, 500);
}

void autonomous() {
  liftPID.target_set(500);
  while (liftPID.exit_condition({r_lift_motor, l_lift_motor}, true) == ez::RUNNING) {
    set_lift(liftPID.compute(lift_motor.get_position()));
    pros::delay(ez::util::DELAY_TIME);
  }

  liftPID.target_set(0);
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



### timers_reset()
Resets all timers for exit conditions.     
<Tabs
  groupId="timers_reset"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::PID liftPID{1, 0.003, 4, 100, "Lift"};  
pros::Motor lift_motor(1);
void opcontrol() {
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      liftPID.timers_reset();
      liftPID.target_set(500);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      liftPID.timers_reset();
      liftPID.target_set(0);
    }
    lift_motor.move(liftPID.compute(lift_motor.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
```


</TabItem>


<TabItem value="proto">

```cpp
void timers_reset();
```



</TabItem>
</Tabs>