# Parked bugs

This page lists bugs we know about and have decided not to fix in code for now, each in a class from [WHAT-IS-A-BUG.md](WHAT-IS-A-BUG.md) section 3.

If your problem is here and you saw it on a robot, open an issue and name the ID. That's E1, and it can move the row into scope. Agents name the ID in their report and write no code for it.

"Where" is the PR that removed code for the row, or the commit and test file that show it. Tests for parked problems aren't kept on `dev`, so check out that commit to run one.

## IDs

Each row has an ID made of an area and a number, like `WAIT-07`. An ID never changes and is never reused, even when a row moves into scope. Agents propose rows with `NEW` as the ID, and a maintainer gives it one.

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

## Could hang or crash, the maintainers decided

WHAT-IS-A-BUG.md says the maintainers decide these, and they parked them.

| ID | Problem | Class | Where |
| --- | --- | --- | --- |
| `WAIT-01` | With the odom drive velocity and mA exits both off, an odom wait on a robot hunting back and forth around its point never returns. The library already warns when both are 0 | 3.4 | 570a50c, `test/test_exit_gate_armed_backstop.cpp` |
| `WAIT-02` | A robot pinned outside big_error with the velocity exit off, whose motors never read over current, never ends `pid_wait_quick_chain()` or `pid_wait_until(distance)`. A custom tracker that sets the pose on every pass has the same shape, since xy's velocity exit doesn't count while its movement is unknown | 3.1 | 570a50c, `test/test_exit_gate_small_error_settle.cpp` and `test/test_exit_gate_wait_until_spin.cpp` |
| `WAIT-03` | A robot dragged away from its odom point while its drive stalls, reading over current only on and off, never ends `pid_wait()` | 3.1 | 570a50c, `test/test_exit_ma_dragged_robot.cpp` |
| `WAIT-04` | A robot pinned partway through a pure pursuit path and turned by another robot never ends `pid_wait()`, because its heading never holds still | 3.1 | 570a50c, `test/test_pp_wait_stuck.cpp` (3 cases) |
| `TRACK-01` | An ADI tracking wheel given one port reads past the end of its port list | 3.4 | Guard removed by #578 |

## 3.1 Only in a simulation, a test, or with impossible numbers

Reopen with E1, the same thing seen on a robot.

| ID | Problem | Where |
| --- | --- | --- |
| `WAIT-05` | A robot crawling under 5 in/s, through a corner or on a slow final leg, ends on the xy velocity exit, interfered, short of its target. A robot log decides whether the velocity exit can drop to the stop speed | 570a50c, `test/test_shove_recovery.cpp` and `test/test_n5_stuck_floor.cpp` |
| `WAIT-06` | An exit that has latched is never checked again, so a bump after it latches and before the wait returns isn't reported as interfered. 3.2.2 did the same | #576 |
| `IMU-03` | When one drive side's wheels slip for about half a second while the robot doesn't turn, `check_imu_task()` reads it as a turn the IMU missed and drops a healthy redundant IMU until the next calibration | 570a50c, `test/test_imu_wheel_slip_divergence.cpp` |
| `WAIT-08` | The mA exit counts time over current while a heavy robot is still speeding up, so it can end the wait early. 3.2.2 did the same | Removed by #576 |

## 3.2 Something PROS or VEXos doesn't do

Reopen with E1, a robot log showing it.

| ID | Problem | Where |
| --- | --- | --- |
| `LOCK-01` | Nothing checks that the PROS system daemon runs below the priority `ez_auto` raises to, or tells the team if it doesn't | Check removed by #577 |
| `LOCK-02` | If `ez_auto` stalls or stops during a wait, the wait keeps reading the last PID state it computed and nothing notices or tells the team | Starvation guards removed by #576 |

## 3.3 After something else already failed

Reopen with E1 or E2 showing teams hit the first failure.

| ID | Problem | Where |
| --- | --- | --- |
| `IMU-01` | A redundant IMU that drops out and comes back isn't used again until the next calibration | Removed by #580 |
| `IMU-02` | When every IMU looks unhealthy, nothing says so, and the primary IMU keeps being read | Warning removed by #580 |

## 3.6 New code for settings no team uses

Reopen with E2, a public team project using the setting, plus a failing test.

| ID | Problem | Where |
| --- | --- | --- |
| `WAIT-07` | With a look-ahead shorter than xy big_error (3 in by default), a velocity or mA exit partway through a path reads clean, because xy's error stays inside big_error | #576 |
| `PID-01` | The secondary (IMU) velocity exit compares one raw reading instead of the last 100 ms like the main velocity exit, and with it on, a slow steady cruise can end a motion early. It's off unless a team passes `true` for `use_imu` | Default turned off by #576 |

## How this page changes

Anyone MAY propose a row in an issue. The maintainers add, move and close rows. Rows are never deleted. Agents never edit this page in a fix PR.
