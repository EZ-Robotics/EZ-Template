---
title: Adding Autonomous Routines
description: Add autonomous routines to the selector
---

# **Adding Autonomous Routines**



## Learn to use Autonomous Functions
Read through `src/autons.cpp` ([or click here](https://ez-robotics.github.io/EZ-Template/tutorials/example_autons)) and learn how to use the autonomous functions by reading through the example routines!  

## Make Your Own Autonomous Routine
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

## Add the Function to `autons.hpp` 
In `include/autons.hpp` add the name of your function.  
```cpp
void SoloAWP();
void ScoreRingsPlatDown();
void NeutralStealPlatDown();
void NeutralStealPlatUp();
```
## Add the Function to the Autonomous Selector
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