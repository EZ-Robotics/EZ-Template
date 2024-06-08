---
title: Upgrading
description: How to upgrade EZ-Template
---

:::warning

This only works for 2.x and beyond.  You cannot upgrade from 1.x to 2.x, or 2.x to 3.x without changing code.

:::


1) Download the most recent `EZ-Template@x.x.x.zip` [here](https://github.com/EZ-Robotics/EZ-Template/releases/latest).  
2) Move the file to your project.  
3) Open terminal or command prompt, and `cd` into your projects directory.    
4) Run this command from terminal `pros c fetch EZ-Template@x.x.x.zip` replacing `x.x.x` with the version number of your file.
5) Apply the library to the project `pros c apply EZ-Template`.  