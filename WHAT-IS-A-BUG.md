# What is a bug in EZ-Template?

This page decides which problems get fixed in library code. The other process docs point here for these rules.

**TL;DR.** A bug that breaks a promise in section 1 gets fixed once it has evidence from section 2. The classes in section 3 are parked, except for their "Still in scope" lines.

Read all of it before judging a problem. A sentence found by search can mean something else out of context.

**MUST** and **MUST NOT** are hard rules. **SHOULD** and **SHOULD NOT** can be broken only with a reason written in the PR description. **MAY** means allowed. These words, and the terms below, mean the same thing in every EZ-Template process doc.

## Terms

- **Library code**, or **the library.** Every file under `include/EZ-Template/` and `src/EZ-Template/` except the generated `version.hpp`, plus the linker scripts in `firmware/`, which reach teams through the example project. Tests, tools, CI, the example files (`src/main.cpp`, `src/autons.cpp`, `include/autons.hpp`, `include/subsystems.hpp`), the vendored folders (`include/pros/`, `include/liblvgl/`, `include/EZ-Units/`) and docs aren't library code.
- **Changed lines.** Lines added plus lines removed in library code, the green and red numbers GitHub shows for the library's files. Test lines don't count. Locally, `git diff --shortstat <dev>...HEAD -- include/EZ-Template src/EZ-Template`.
- **Controller code.** Library code that computes what the motors are told during a motion or in driver control, or decides when a motion ends. That includes at least `PID`, `slew`, `travel`, `tracking_wheel` and `util` (their `.hpp` and `.cpp`), `drive/drive.hpp`, and everything in `src/EZ-Template/drive/` except `pid_tuner.cpp`. If you're unsure, treat it as controller code.
- **Public API.** Every name a team can use through `EZ-Template/api.hpp`, including public data members like `leftPID`, except names in `ez::detail` and members starting with `internal_`.
- **Wait.** Any `pid_wait*` function. A **hang** is a wait that would never return while the robot is enabled. In a test, it hasn't returned 5 seconds after every exit time and timeout it uses has run out.
- **Host tests.** `make -C test`, run on a computer. They can't build `display.cpp`, `sdcard.cpp`, `health.cpp`, `pid_tuner.cpp` or `firmware/`, so a fix there gets a robot run instead of a test, and its evidence has to be E1 or E3.
- **Robot run.** The change run on a V5 robot, in an auton or driver control that uses it, with the robot, the code and the result written on the PR.
- **Team corpus.** The [EZ-Template-Corpus](https://github.com/EZ-Robotics/EZ-Template-Corpus), a public list of links to public team projects that use EZ-Template.
- **Season.** One VEX game year, from the game's reveal to the last day of its World Championship. **The last stable release of the previous season** is the newest stable release tagged before this season's reveal.

## Words for bugs

- **Bug.** EZ-Template does something it shouldn't, or doesn't do something it should. Parked bugs are still bugs.
- **In scope.** Breaks a promise in section 1. It gets fixed once it has evidence.
- **Waiting for evidence.** Would be in scope, but has no evidence yet. No code is written for it.
- **Parked.** Fits a class in section 3. It's kept out of the code until evidence moves it into scope.
- **Behavior change.** A difference in how a robot drives from an earlier version. It goes in the release notes either way. If it keeps every promise, it's class 3.8 and there's nothing to fix.

Nobody writes "not a bug" about anything on this page. Write **parked** instead. The labels `in-scope`, `waiting-for-evidence` and `parked` mean exactly these words.

Two things aren't judged here. Something that breaks no promise and fits no class, like faster code, is an `enhancement`. Problems in tests, tools, CI or the example project are fixed like docs, as long as no library code changes.

## 1. What EZ-Template promises

1. **The robot does the same thing every time.** The same auton on the same robot drives the same way and ends in the same place, run after run. EZ-Template MUST NOT add differences of its own, like state left over from an earlier motion. When temperature, battery or the field change, EZ-Template uses what its control loops already read to still get there and finish. Adding a new way to correct for something is a feature. A miss from something EZ-Template can't measure, like wheel slip, doesn't break this. Task timing that only matters on one exact 10 ms pass is class 3.1.
2. **Never hang, never crash, always build.** Every wait returns. A bad input to a public function prints a warning once and the program keeps running. That warning needs no other evidence if it fits in 10 changed lines. Where it can, the library catches a mistake at compile time instead. A clean project builds with the versions in MAINTAINING.md.
3. **The default constructor is enough.** Ports, IMU port, wheel rpm and wheel diameter make every feature work. A feature that's off by default works once it's turned on, with no other setting.
4. **It works with real V5 hardware,** the way VEX documents it and the way it measures on robots. Known VEXos differences are tracked in [#343](https://github.com/EZ-Robotics/EZ-Template/issues/343) and [#344](https://github.com/EZ-Robotics/EZ-Template/issues/344).
5. **It works the way PROS runs,** including PROS deleting the competition task on every mode change, even while it holds an EZ-Template lock.
6. **It works for the code teams write.** These are normal use.
   - a team's own PID task at a period other than 10 ms, like 20 ms
   - a PID fed by a sensor slower than 10 ms, like a distance sensor or GPS
   - `drive_rpm_set()` while the robot is moving
   - starting a motion from a team's own task while an auton is in a wait. The new motion MUST run as asked and the wait MUST return. What the old wait reports is class 3.4.
7. **Tank drives only.**
8. **Every public function does what its name and docs say,** including what it prints on the brain and controller.
9. **Updating never breaks a working robot.** If an auton or driver control works on one stable release, it works at least as well on every later stable release with the same major version. A fix MAY change how a robot drives when the old behavior broke a promise, and the release notes say what changed. Any other change to driving waits for the next major version. When a major version changes or removes a call, old code MUST keep working with a deprecation warning or fail to compile with a message saying what to write instead. No release, including a pre-release, may make old code compile with a new meaning.

## 2. Evidence

An in-scope bug MUST have one of these before anyone writes a fix.

- **E1. Seen on a robot.** A report naming the EZ-Template version, the code that was running, and either a video or terminal log that shows it, or steps someone else can follow on their own robot.
- **E2. Team code hits it.** A project in the [EZ-Template-Corpus](https://github.com/EZ-Robotics/EZ-Template-Corpus) or code posted in an issue hits the trigger, or a team's patched copy of the library changes those lines. Plus a host test that uses EZ-Template the same way and fails on `dev`. A maintainer can write the test, in our own words. It MUST NOT use `test/sim_physics.hpp`, `test/exit_gate_rig.hpp` or any other model of how a robot moves. A count of projects that call a function isn't evidence. For a corpus project, the issue or PR only says it was found in the EZ-Template-Corpus. Send the exact project and line to a maintainer privately on the Discord.
- **E3. The code contradicts itself.** The wrong result follows from the code alone, for an input a team can pass, and you can quote the comment, docs page or math it breaks. For example a sign error, a unit mismatch, an unwrapped angle, a divide by zero, a null or out-of-range read, a wait that can never return, or a lock held across a call PROS can delete. If you need a simulation to show it, it isn't E3.

A scenario named in a promise or a "Still in scope" line counts as the trigger. It still needs evidence.

**Maintainers' call.** A maintainer MAY put any bug in scope without evidence, including a parked one or one they found and understand. Write it in the issue, or in the PR thread when there's no issue. A maintainer can also tell their own agent, and the agent writes it on its card so the other maintainers see it in the PR.

A failing test isn't evidence by itself. Simulation output isn't evidence, including the sweep harness and `test/sim_physics.hpp`, until a model's noise has been checked against a real robot log. None has been yet. A simulation MAY help understand a bug that already has evidence.

**Every fix.** A fix over 30 changed lines needs a maintainer's OK in its issue before the PR, counting every PR for that issue together. A change to controller code needs a robot run before release. It MAY merge first with the `needs-hardware-test` label. Before AI, four controller fixes caused new bugs (576eb32, d844c96, 681db82, 1aeb393), and most pre-AI fixes changed under 10 lines.

## 3. Parked classes

We fix our own code. Hardware and VEXos problems go to VEX, kernel problems go to PROS, and a team's setup, tuning and code are the team's.

If a bug fits a class, it's parked even when it seems to break a promise, unless it fits the class's "Still in scope" line. Then it's in scope, and its card names the promise.

**3.1 Only in a simulation, a test, or with impossible numbers.** For example, a push that has to land on one exact 10 ms pass, more than one control pass per tick, unchecked sensor noise, or an auton longer than a match. A sim can always find another case, and fixing them in 4.0 made the code harder to read without helping any team.
*Still in scope, once it has evidence.* The same failure from something that happens on a field, like one push, pin or wall hit, and anything in promise 6. Seen only in a sim, it stays 3.1.

**3.2 Something PROS or VEXos doesn't do.** For example, the library's own task stalling for 300 ms, which no robot log has shown. Also any release older than the last stable release of the previous season.
*Still in scope.* Promise 5.

**3.3 After something else already failed.** For example, a turn missing because the IMU was unplugged, or odom spinning because a tracking wheel was set up backwards. The first failure is the bug.
*Still in scope.* The first failure, and promise 2. A disconnected motor MUST NOT hang a wait. One failed read (`PROS_ERR`, NaN, infinity) is normal on V5, and using it as a real reading is the library's bug. A sensor that stays unplugged is the first failure.

**3.4 The team's own setup or code.** For example, wrong ports, wheel size or rpm, oscillating PID constants, or exits loosened until the robot stops early. If it goes away in a fresh example project with only the team's ports, wheel size and rpm, it's setup.
*Still in scope.* Promise 6, and the warning promise 2 allows.

**3.5 An edited library, or internals.** A team's patched copy, code using `ez::detail`, or code getting around `private`.
*Still in scope.* A team's patched copy that changes the lines with the bug is E2 once a failing test is added, until a maintainer decides otherwise in the issue.

**3.6 New code for settings no team uses.** For example, handling for a combination of settings or an order of calls no public team project we know of uses. Every line of it is one more line a student has to read.
*Still in scope.* A setting that does the opposite of its docs gets a fix of 10 changed lines or fewer. A feature isn't parked here until one season after it's released.

**3.7 The library reported what happened.** For example, `interfered` after the robot was hit. There's nothing to fix.

**3.8 A behavior change that keeps every promise.** For example, a wait settling 50 ms later than in a pre-release. It goes in the release notes.

**Hangs, crashes and runaways.** If a parked or waiting bug could hang a wait, crash the program, leave a motor running after its motion should have ended, or drive the robot somewhere it wasn't told to go, open an issue for the maintainers right away. Don't write a fix. The maintainers decide.

## 4. Worked examples

| Problem | Class | Evidence | Result |
| --- | --- | --- | --- |
| `pid_speed_max_set()` during an odom motion is undone at the next path point | Promise 8 | E1, a team found it (#536), plus a failing test | **Fixed** (#538) |
| Controller text sent every 10 ms garbles the screen | Promise 8 | Four teams' patched copies deleted the same lines. Today it would also need a failing test | **Fixed** (#495) |
| Odom moves to a point behind the robot spin in circles | Promise 8 | E3, the heading error isn't wrapped | **Fixed** in 10 lines (#497) |
| Setting any exit constant to 0 makes every wait return at once | 3.6, but does the opposite of its docs | E3 | **Fixed** in 2 lines (#91) |
| `ez_auto` stalls for 300 ms and waits misjudge | 3.2 | None. Never seen on a robot | **Parked** |
| Driver control feels slower after updating, from a 4-line change | Promise 9, a regression | E1, several teams and a maintainer watching one robot | **Fixed** in 8 lines (#204) |
| Derivative on error jerks every new target | No promise broken | None, found from control theory | **Maintainers' call** in 3.1.0 (#105). Today it would wait for a major version (promise 9). It also caused a later integral bug |

## 5. How this page changes

The maintainers decide. Anyone MAY propose a change in an issue. Agents propose changes in their report and MUST NOT edit this page in a fix PR.
