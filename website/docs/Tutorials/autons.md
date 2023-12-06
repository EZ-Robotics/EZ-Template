---
title: Adding Autonomous Routines
description: The process of creating auton routines.
---

# **Adding Autonomous Routines**



## Step 1  
Read through `src/autons.cpp` ([click here](https://github.com/EZ-Robotics/EZ-Template-Example/blob/main/src/autons.cpp)) and learn how to use the autonomous functions by reading through the example routines!  

## Step 2  
Make a new function in `src/autons.cpp` and name it something that says what the autonomous will do.  
```cpp
void SoloAWP() {
  // . . .
  // Autonomous code goes here
  // . . .
}

void ScoreRingsPlatDown() {
  // . . .
  // Autonomous code goes here
  // . . .
}

void NeutralStealPlatDown() {
  // . . .
  // Autonomous code goes here
  // . . .
}

void NeutralStealPlatUp() {
  // . . .
  // Autonomous code goes here
  // . . .
}
```

## Step 3  
In `include/autons.hpp` add the name of your function.  
```cpp
void SoloAWP();
void ScoreRingsPlatDown();
void NeutralStealPlatDown();
void NeutralStealPlatUp();
```
## Step 4  
To add the autonomous mode to the on screen selector, in `src/main.cpp` go to `void initialize()` and either replace an existing autonomous mode or add new pages.    
```cpp
void initialize() {
  . . . 

  // Autonomous Selector using LLEMMU
  ez::as::auton_selector.add_autons({
    Auton("Solo AWP\n\nStarting Position: Plat Down", SoloAWP),
    Auton("Score Rings on Amogo\n\nStarting Position: Plat Down", ScoreRingsPlatDown),
    Auton("Neutral Steal\n\nStarting Position: Plat Down", NeutralStealPlatDown),
    Auton("Neutral Steal\n\nStarting Position: Plat Up", NeutralStealPlatUp),
  });

  . . .
}
```