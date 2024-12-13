---
layout: default
title: Adding Autonomous Routines
description: adding autos to the selector
---

## Learn to use Autonomous Functions
Read through `src/autons.cpp` ([or click here](https://ez-robotics.github.io/EZ-Template/tutorials/example_autons)) and learn how to use the autonomous functions by reading through the example routines!  

## Make the Autonomous Routines
Make a new function in `src/autons.cpp` and name it something that quickly explains what the autonomous will do.  
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

## Add it to `autons.hpp` 
In `include/autons.hpp` add the name of your function.  If this isn't done, `main.cpp` won't know what any of these names mean.  
```cpp
#pragma once

void SoloAWP();
void ScoreRingsPlatDown();
void NeutralStealPlatDown();
void NeutralStealPlatUp();
```

## Add it to the Autonomous Selector
To add the autonomous mode to the on-screen selector, in `src/main.cpp` go to `void initialize()` and either replace an existing autonomous mode or add new pages.  [Click here](https://ez-robotics.github.io/EZ-Template/tutorials/using_auton_selector) to learn how to use the autonomous selector.    
```cpp
void initialize() {
 . . . 

  // Autonomous Selector using LLEMMU
  ez::as::auton_selector.autons_add({
    {"Solo AWP\n\nStarting Position: Plat Down", SoloAWP},
    {"Score Rings on Amogo\n\nStarting Position: Plat Down", ScoreRingsPlatDown},
    {"Neutral Steal\n\nStarting Position: Plat Down", NeutralStealPlatDown},
    {"Neutral Steal\n\nStarting Position: Plat Up", NeutralStealPlatUp},
 });

 . . .
}
```