---
layout: default
title: Slew
parent: Docs
description:  ""
nav_order: 4
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';



## EZ-Template Slew Wrappers


### slew_drive_constants_set()
Sets constants for slew during drive forward motions.  This sets forward and backward constants.  Robot will start at `min_speed` and will be at the maximum set speed once `distance` is traveled.  

`distance` is in an okapi distance unit.  
`min_speed` is 0 to 127.    
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
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.slew_drive_constants_set(5_in, 50);

  chassis.pid_drive_set(12_in, 110, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_drive_constants_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>




### slew_drive_constants_forward_set()
Sets constants for slew during drive forward motions.  Robot will start at `min_speed` and will be at the maximum set speed once `distance` is traveled.  

`distance` is in an okapi distance unit.  
`min_speed` is 0 to 127.    
<Tabs
  groupId="slew_forward_constant_set"
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

  chassis.slew_drive_constants_forward_set(5_in, 50);

  chassis.pid_drive_set(12_in, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_drive_constants_forward_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>




### slew_drive_constants_backward_set()
Sets constants for slew during drive forward motions.  Robot will start at `min_speed` and will be at the maximum set speed once `distance` is traveled.  

`distance` is in an okapi distance unit.  
`min_speed` is 0 to 127.    
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
void autonomous() {
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset(); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); 

  chassis.slew_drive_constants_backward_set(5_in, 50);

  chassis.pid_drive_set(-12_in, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_drive_constants_backward_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>


### slew_turn_constants_set()
Sets constants for slew during turns.  Robot will start at `min_speed` and will be at the maximum set speed once `distance` is traveled.  

`distance` is in an okapi angle unit.  
`min_speed` is 0 to 127.    
<Tabs
  groupId="slew_turn_constant_set"
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

  chassis.slew_turn_constants_set(5_deg, 50);

  chassis.pid_turn_set(-90_deg, 110, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_turn_constants_set(okapi::QAngle distance, int min_speed);
```

</TabItem>
</Tabs>


### slew_swing_constants_set()
Sets constants for slew during swing turns.  This sets forward and backwards constants.  Robot will start at `min_speed` and will be at the maximum set speed once `distance` is traveled.  

`distance` is in an okapi length unit.  
`min_speed` is 0 to 127.    
<Tabs
  groupId="slew_swing_constants_set_okapi_distance"
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

  chassis.slew_swing_constants_set(5_in, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>

### slew_swing_constants_set()
Sets constants for slew during swing turns.  This sets forward and backwards constants.  Robot will start at `min_speed` and will be at the maximum set speed once `distance` is traveled.  

`distance` is in an okapi angle unit.    
`min_speed` is 0 to 127.      
<Tabs
  groupId="slew_swing_constants_set_okapi_angle"
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

  chassis.slew_swing_constants_set(7_deg, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_set(okapi::QAngle distance, int min_speed);
```

</TabItem>
</Tabs>




### slew_swing_constants_forward_set()
Sets constants for slew during forward swing turns.  Robot will start at `min_speed` and will be at the maximum set speed once `distance` is traveled.  

`distance` is in an okapi length unit.  
`min_speed` is 0 to 127.    
<Tabs
  groupId="slew_forward_swing_constants_set_okapi_distance"
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

  chassis.slew_swing_constants_forward_set(5_in, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_forward_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>

### slew_swing_constants_forward_set()
Sets constants for slew during forward swing turns.  Robot will start at `min_speed` and will be at the maximum set speed once `distance` is traveled.  

`distance` is in an okapi angle unit.    
`min_speed` is 0 to 127.    
<Tabs
  groupId="slew_forward_swing_constants_set_okapi_angle"
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

  chassis.slew_swing_constants_forward_set(7_deg, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_forward_set(okapi::QAngle distance, int min_speed);
```

</TabItem>
</Tabs>



### slew_swing_constants_backward_set()
Sets constants for slew during backward swing turns.  Robot will start at `min_speed` and will be at the maximum set speed once `distance` is traveled.  

`distance` is in an okapi length unit.  
`min_speed` is 0 to 127.    
<Tabs
  groupId="slew_backward_swing_constants_set_okapi_distance"
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

  chassis.slew_swing_constants_backward_set(5_in, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_backward_set(okapi::QLength distance, int min_speed);
```

</TabItem>
</Tabs>

### slew_swing_constants_backward_set()
Sets constants for slew during backward swing turns.  Robot will start at `min_speed` and will be at the maximum set speed once `distance` is traveled.  

`distance` is in an okapi angle unit.   
`min_speed` is 0 to 127.    
<Tabs
  groupId="slew_bakward_swing_constants_set_okapi_angle"
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

  chassis.slew_swing_constants_backward_set(7_deg, 50);

  chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 110, 0, true);
  chassis.pid_wait();
}
```

</TabItem>

<TabItem value="proto">

```cpp
void slew_swing_constants_backward_set(okapi::QAngle distance, int min_speed);
```

</TabItem>
</Tabs>



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


