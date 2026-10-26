---
layout: default
title: Util
description:  chat, util is short for utility if you're new here.  we're just using slang
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Printing

### screen_print()

<ApiFunction name="screen_print" />

### ez_template_print()

<ApiFunction name="ez_template_print" />

### to_string_with_precision()

<ApiFunction name="util::to_string_with_precision" />

## Math

### sgn()

<ApiFunction name="util::sgn" />

### clamp()

<ApiFunction name="util::clamp" args="double, double, double | double, double" />

### to_deg()

<ApiFunction name="util::to_deg" />

### to_rad()

<ApiFunction name="util::to_rad" />

### wrap_angle()

<ApiFunction name="util::wrap_angle" />

## Points

### absolute_angle_to_point()

<ApiFunction name="util::absolute_angle_to_point" />

### distance_to_point()

<ApiFunction name="util::distance_to_point" />

### vector_off_point()

<ApiFunction name="util::vector_off_point" />

## controller
The pros controller is defined globally in our library as `master`.     
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
void opcontrol() {
  while (true) {
    int l_stick = chassis.opcontrol_curve_left(master.get_analog(ANALOG_LEFT_Y));
    int r_stick = chassis.opcontrol_curve_right(master.get_analog(ANALOG_RIGHT_Y));
    
    chassis.drive_set(l_stick, r_stick);
    
    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
extern pros::Controller master;
```



</TabItem>
</Tabs>


 



 

## DELAY_TIME 
Standard delay time for loops in ms.    
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
void opcontrol() {
  while (true) {
    chassis.opcontrol_tank();

    pros::delay(ez::util::DELAY_TIME);
  }
}
```

</TabItem>


<TabItem value="proto">

```cpp
const int DELAY_TIME = 10;
```



</TabItem>
</Tabs>






 

## SD_CARD_ACTIVE
Boolean that checks if an SD card is installed.  True if there is one, false if there isn't.    
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
void initialize() {
  if (!ez::util::SD_CARD_ACTIVE) 
    printf("No SD Card Found!\n");
}
```


</TabItem>


<TabItem value="proto">

```cpp
const bool SD_CARD_ACTIVE = pros::usd::is_installed();
```



</TabItem>
</Tabs>





 

## Version macros
Macros that tell you which version of EZ-Template your code is being built against.  They're for libraries and shared code that need to work with more than one EZ-Template version.  Include `EZ-Template/api.hpp` and they're available, there's nothing to call.  

:::caution 3.x and early betas don't have these

The version macros are new in 4.0.0-beta.3, and they're not in 4.0.0-beta.1 or 4.0.0-beta.2.  On those, and on 3.x, every macro on this page is missing, so your code has to cope with that.  

:::

| Macro | What it is |
| --- | --- |
| `EZ_TEMPLATE_VERSION_MAJOR`, `EZ_TEMPLATE_VERSION_MINOR`, `EZ_TEMPLATE_VERSION_PATCH` | The version numbers |
| `EZ_TEMPLATE_VERSION_STAGE` | `EZ_TEMPLATE_STAGE_ALPHA`, `EZ_TEMPLATE_STAGE_BETA`, `EZ_TEMPLATE_STAGE_RC` or `EZ_TEMPLATE_STAGE_RELEASE` |
| `EZ_TEMPLATE_VERSION_PRERELEASE_NUM` | The `N` in `beta.N`, or `0` on a full release |
| `EZ_TEMPLATE_VERSION_PRERELEASE` | `"beta.6"`, or `""` on a full release |
| `EZ_TEMPLATE_VERSION_STRING` | `"4.0.0-beta.6"` |
| `EZ_TEMPLATE_VERSION` | One integer that sorts correctly: alpha, then beta, then rc, then release.  `4.0.0-rc.1` is less than `4.0.0` |
| `EZ_TEMPLATE_VERSION_ENCODE(major, minor, patch, stage, num)` | Builds that integer, for comparing against an exact version |
| `EZ_TEMPLATE_VERSION_AT_LEAST(major, minor, patch)` | Feature check.  Ignores the prerelease, so `4.0.0-beta.6` counts as `4.0.0` |

Minor, patch and the prerelease number must each stay under 100.  

### Checking for a version
Include EZ-Template first, then give 3.x a fallback.  3.x doesn't define the macro, so without the fallback the `#if` would fail to compile.  If the fallback comes before the include, the header's own definition still wins, but the compiler warns `macro redefined`.  

```cpp
#include "EZ-Template/api.hpp"  // before the fallback below

#ifndef EZ_TEMPLATE_VERSION_AT_LEAST
#define EZ_TEMPLATE_VERSION_AT_LEAST(major, minor, patch) 0  // 3.x
#endif

#if EZ_TEMPLATE_VERSION_AT_LEAST(4, 0, 0)
  // 4.0 or newer
#else
  // 3.x
#endif
```

Don't write `#if defined(EZ_TEMPLATE_VERSION_MAJOR) && EZ_TEMPLATE_VERSION_AT_LEAST(4, 0, 0)`.  The preprocessor still reads the second half on 3.x, where that macro doesn't exist, and stops with a syntax error.  Nest the `#if` instead.  

### Prereleases are not final
Alpha, beta and rc releases are not final.  APIs can change or be removed between them, and the final release may differ from any prerelease.  `EZ_TEMPLATE_VERSION_AT_LEAST(4, 0, 0)` is true on every 4.0.0 prerelease, so if your code depends on something that changed during the prereleases, compare against a specific stage with `EZ_TEMPLATE_VERSION_ENCODE` instead.  

```cpp
#ifdef EZ_TEMPLATE_VERSION
#if EZ_TEMPLATE_VERSION >= EZ_TEMPLATE_VERSION_ENCODE(4, 0, 0, EZ_TEMPLATE_STAGE_RC, 1)
  // 4.0.0-rc.1 or newer
#endif
#endif
```

To be told when you're building against a prerelease, add a warning:  

```cpp
#ifdef EZ_TEMPLATE_VERSION_STAGE
#if EZ_TEMPLATE_VERSION_STAGE != EZ_TEMPLATE_STAGE_RELEASE
#warning "Built against an EZ-Template prerelease, the API may change"
#endif
#endif
```

### Printing the version

```cpp
void initialize() {
  printf("EZ-Template %s\n", EZ_TEMPLATE_VERSION_STRING);  // EZ-Template 4.0.0-beta.6
}
```
