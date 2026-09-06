---
layout: default
title: Screen Rotation
description: for brains mounted sideways or upside down
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

:::note

Experimental, unverified on hardware.  This uses LVGL's software rotation on the live display driver.

:::

## Setters

### screen_rotation_set()
Rotates the brain screen in 90-degree steps, for brains mounted sideways or upside down.

Accepts 0, 90, 180, or 270 (measured clockwise); other values are rejected with a printed message.  Call this after the screen is initialized (`ez::as::initialize()` or `pros::lcd::initialize()`).

At 90 and 270 the stock LLEMU screen is swapped for EZ's portrait screen.  LLEMU builds its widgets at the resolution live when `pros::lcd::initialize()` ran (480x272), so once the canvas becomes 272x480 its text runs off the right edge.  The portrait screen re-flows the same LLEMU theme, font, print lines, and three-button bar to the rotated resolution, wrapping long lines rather than clipping them, and replays its button presses onto LLEMU's own buttons so `pros::lcd::register_btn*_cb` callbacks behave identically.  0 and 180 restore LLEMU untouched.

Touch rotates with the pixels, so on-screen buttons respond where they are drawn.  90/270 swap the screen's width and height.  The physical LLEMU buttons are unaffected.

`degrees` 0, 90, 180, or 270
<Tabs
  groupId="screen_rotation_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
void initialize() {
  ez::as::initialize();

  // Brain is mounted sideways
  ez::screen_rotation_set(90);
}
```

</TabItem>

<TabItem value="proto">

```cpp
void screen_rotation_set(int degrees);
```

</TabItem>
</Tabs>


### screen_line_set()
Writes one of the 8 print lines, to the portrait screen when it is loaded and to LLEMU otherwise.  `ez::screen_print` routes through here.

`line` 0 through 7  
`text` the text to print
<Tabs
  groupId="screen_line_set"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::screen_line_set(0, "Hello!");
```

</TabItem>

<TabItem value="proto">

```cpp
void screen_line_set(int line, std::string text);
```

</TabItem>
</Tabs>


### screen_line_clear()
Clears one of the 8 print lines, through the same routing as `screen_line_set()`.

`line` 0 through 7
<Tabs
  groupId="screen_line_clear"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::screen_line_clear(0);
```

</TabItem>

<TabItem value="proto">

```cpp
void screen_line_clear(int line);
```

</TabItem>
</Tabs>


### screen_lines_clear()
Clears all 8 print lines, through the same routing as `screen_line_set()`.
<Tabs
  groupId="screen_lines_clear"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::screen_lines_clear();
```

</TabItem>

<TabItem value="proto">

```cpp
void screen_lines_clear();
```

</TabItem>
</Tabs>


## Getters

### screen_rotation_get()
Returns the current rotation, in degrees (0, 90, 180, or 270).
<Tabs
  groupId="screen_rotation_get"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::screen_rotation_set(90);
printf("%i\n", ez::screen_rotation_get()); // Prints 90
```

</TabItem>

<TabItem value="proto">

```cpp
int screen_rotation_get();
```

</TabItem>
</Tabs>


### screen_portrait_enabled()
Returns true while the portrait screen is the loaded screen (rotation 90 or 270).
<Tabs
  groupId="screen_portrait_enabled"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
ez::screen_rotation_set(90);
printf("%i\n", ez::screen_portrait_enabled()); // Prints true
```

</TabItem>

<TabItem value="proto">

```cpp
bool screen_portrait_enabled();
```

</TabItem>
</Tabs>


## Touch Coordinates

### screen_touch_rotate()
Maps a raw panel touch into the coordinate space of a screen rotated by `degrees`.

LVGL already applies this to its own pointer input, so widgets need no help; this is only for code that reads the panel directly (`pros::screen_touch_status()`) and so bypasses LVGL.  Do not apply it to `lv_indev` data or the point is transformed twice.

`degrees` 0, 90, 180, or 270  
`panel_w` the physical, unrotated panel width (480 on the V5)  
`panel_h` the physical, unrotated panel height (272 on the V5)  
`raw_x` raw touch x from the panel  
`raw_y` raw touch y from the panel
<Tabs
  groupId="screen_touch_rotate"
  defaultValue="proto"
  values={[
    { label: 'Prototype',  value: 'proto', },
    { label: 'Example',  value: 'example', },
  ]
}>

<TabItem value="example">

```cpp
pros::screen_touch_status_s_t status = pros::screen_touch_status();
ez::screen_point p = ez::screen_touch_rotate(ez::screen_rotation_get(), 480, 272, status.x, status.y);
printf("x: %i, y: %i\n", p.x, p.y);
```

</TabItem>

<TabItem value="proto">

```cpp
struct screen_point {
  int x;
  int y;
};

screen_point screen_touch_rotate(int degrees, int panel_w, int panel_h, int raw_x, int raw_y);
```

</TabItem>
</Tabs>
