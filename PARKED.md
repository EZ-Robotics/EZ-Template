# Parked bugs

This page lists bugs we know about and have decided not to fix in code for now, each in a class from [WHAT-IS-A-BUG.md](WHAT-IS-A-BUG.md) section 3.

**The list is being rebuilt.** The waits and exit conditions are being rebuilt from 3.2.2, and most rows written so far describe code that rebuild removes. The list comes back once the rebuild lands.

Until then, report bugs the usual way (CONTRIBUTING.md), and agents judge each problem against WHAT-IS-A-BUG.md directly.

## IDs

Each row will have an ID made of an area and a number, like `WAIT-07`. An ID never changes and is never reused, even when a row moves into scope. Agents propose rows with `NEW` as the ID, and a maintainer gives it one.

| Area | Covers |
| --- | --- |
| `WAIT` | `pid_wait*`, exit conditions |
| `PID` | `ez::PID` and the PID tuner |
| `DRIVE` | The `Drive` object, setup, gearing and drive motions |
| `TURN` | Turns and swings |
| `ODOM` | Odometry, odom motions, pure pursuit, boomerang |
| `TRACK` | Tracking wheels and custom tracking |
| `SLEW` | Slew |
| `IMU` | IMUs |
| `DRIVER` | Driver control and active brake |
| `SELECT` | The auton selector |
| `SCREEN` | Brain screen and controller printing |
| `LOCK` | Tasks, locks and `ez_auto` |
| `PTO` | Power take-off |
| `BUILD` | Compiling, linking and `firmware/` |

## How this page changes

Anyone MAY propose a row in an issue. The maintainers add, move and close rows. Rows are never deleted. Agents never edit this page in a fix PR.
