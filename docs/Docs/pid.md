---
layout: default
title: PID
parent: Docs
nav_order: 6
---


# **PID**
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}


---


# Constructors 

## PID()
Creates a blank PID object.  
**Prototype**
```cpp
PID();
```

**Example** 
```cpp
PID liftPID;
```


---



## PID()
Creates a PID object with constants.  Everything past kP has a default starting value, so you can juts put kP.  
**Prototype**
```cpp
PID(double p, double i = 0, double d = 0, double start_i = 0, std::string name = "");
```

**Example** 
```cpp
PID liftPID{1, 0.003, 4, 100, "Lift"};
```


---

# Functions

## set_constants()
Sets PID constants.  
`p` kP
`i` kI
`d` kD
`p_start_i` i will start when error is within this    
**Prototype**
```cpp
void set_constants(double p, double i = 0, double d = 0, double p_start_i = 0);
```

**Example** 
```cpp
PID liftPID;
void initialize() {
  liftPID.set_constants(1, 0, 4);
}
```



---



## set_target()
Sets PID target.   
**Prototype**
```cpp
void set_target(double input);
```

**Example** 
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


---



## set_exit_condition()
Sets the exit condition constants.  To disable one of the conditions, set the constants relating to it to `0`.    
`p_small_exit_time` time, in ms, before exiting `p_small_error`  
`p_small_error` small error threshold  
`p_big_exit_time` time, in ms, before exiting `p_big_error`  
`p_big_error` big error threshold  
`p_velocity_exit_time` time, in ms, for velocity to be 0  
`p_mA_timeout` time, in ms, for `is_over_current` to be true       
**Prototype**
```cpp
void set_exit_condition(int p_small_exit_time, double p_small_error, int p_big_exit_time = 0, double p_big_error = 0, int p_velocity_exit_time = 0, int p_mA_timeout = 0);

```

**Example**
```cpp
PID liftPID{1, 0.003, 4, 100, "Lift"};  
void initialize() {
  liftPID.set_exit_condition(100, 3, 500, 7, 500, 500);
}
```



---



## set_name()
A string that prints when exit conditions are met.  When you have multiple mechanisms using exit conditions and you're debugging, seeing which exit condition is doing what can be useful.     
**Prototype**
```cpp
void set_name(std::string name);
```

**Example** 
```cpp
PID liftPID{1, 0.003, 4, 100};
void initialize() {
  liftPID.set_name("Lift");
}  
```


---



## compute()
Computes PID.  
**Prototype**
```cpp
double compute(double current);
```

**Example** 
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


---



# Exit Conditions

## No Motor
Outputs one of the `exit_output` states.  This exit condition checks `small_error`, `big_error` and `velocity` if they are enabled.    
**Prototype**
```cpp
ez::exit_output exit_condition(bool print = false);
```

**Example** 
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


---



## One Motor
Outputs one of the `exit_output` states.  This exit condition checks `small_error`, `big_error`, `velocity` and `mA` if they are enabled.    
**Prototype**
```cpp
ez::exit_output exit_condition(pros::Motor sensor, bool print = false);
```

**Example** 
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


---



## Multiple Motors
Outputs one of the `exit_output` states.  This exit condition checks `small_error`, `big_error`, `velocity` and `mA` if they are enabled.  When any of the motors trip `mA`, it returns `mA_EXIT`.    
**Prototype**
```cpp
ez::exit_output exit_condition(std::vector<pros::Motor> sensor, bool print = false);
```

**Example** 
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


---