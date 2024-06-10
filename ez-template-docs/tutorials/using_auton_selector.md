---
title: Using Autonomous Selector
description: How to have multiple autonomous routines in one program
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Why?
There are two main ways to have multiple autonomous routines: 
1. One program loaded with multiple routines.
2. Multiple programs with one routine each.

I dislike 2.  If you make a change to a PID constant or make a change to user control code, you now have to redownload multiple times.  I find this tedious and easy to make a mistake while under pressure at a tournament.  

## Selecting the Autonomous Routine
The brain screen has 3 buttons.  With EZ-Template the center button does nothing.  You can navigate through the menus by selecting the left or right button.  Once autonomous mode is enabled, the page that is left open will be the autonomous routine that runs.  

## Running the Autonomous Routine
### With a Competition Switch
With a competition switch, run your code in disable.  Select the autonomous on the screen.  Change the competition switch to `Autonomous` on your competition switch, and set it to `Enable`.  Your autonomous routine will run!

### Without a Competition Switch
This code changed in the example project slightly between 3.0.x and 3.1.0, where it now requires multiple buttons to be held to prevent accidental triggers.  But, holding `B` and `DOWN` will run the selected autonomous routine during driver control.  
<Tabs
  groupId="310"
  defaultValue="proto"
  values={[
    { label: '>3.1.0',  value: 'proto', },
 { label: '<3.1.0',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
// Trigger the selected autonomous routine
if (master.get_digital_new_press(DIGITAL_B))
  autonomous();
```
</TabItem>


<TabItem value="proto">

```cpp
// Trigger the selected autonomous routine
if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN))
  autonomous();
```
</TabItem>
</Tabs>

## Micro-SD Card Support
To make things easier at tournaments, having an SD card in your brain will save the last page between cycles.  So if you select page 3 then turn off the brain, the next time you turn the brain on it'll start from page 3.  This means you can select your autonomous routine while queueing for a match and it'll hold when you get to a match.  

The micro-SD card must be formatted to FAT32.  [Here](https://www.lifewire.com/format-sd-card-to-fat32-6666166) is a tutorial on how to do that. 
