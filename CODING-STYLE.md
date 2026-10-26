# Coding style

These rules apply to new and changed library code and tests, whoever wrote them. Existing code is changed to match only when a PR is already changing it. Rules marked **Example project** also apply to the example files and the `example-project` branch. MUST, SHOULD and MAY mean what [WHAT-IS-A-BUG.md](WHAT-IS-A-BUG.md) says.

Most rules give a reason in italics, with a link to its source. If a reason doesn't convince you, open an issue. Maybe we got it wrong.

`sh tools/format.sh` (clang-format 18.1.8) handles spacing, braces and line breaks, and CI checks it on every PR. This page covers what the formatter can't.

## Layout

- Lines MUST be 160 columns or fewer. This also applies to the example files. *Long lines are one of the biggest reasons code is hard to read ([Buse and Weimer 2010](https://web.eecs.umich.edu/~weimerw/p/weimer-tse2010-readability-preprint.pdf)).*
- A braced list that should stay one item per line, like a list of odom points, SHOULD end with a comma after the last item. *Without that comma, the formatter joins any list that fits in 160 columns onto one line.*
- The vendored folders (`include/pros/`, `include/liblvgl/`, `include/EZ-Units/`) MUST NOT be formatted or edited. *They're copied from other projects, and CI checks that `include/EZ-Units/units.hpp` matches its release byte for byte.*
- A string a user will see on the terminal, brain or controller MUST NOT be split across lines. *People search the code for the exact text they saw.*
- A formatting-only change MUST be its own PR, and its commit hash goes in `.git-blame-ignore-revs` after it merges.

## Size

- A statement SHOULD be nested 3 levels deep or fewer. Return early instead. *Deep nesting makes code take longer to understand ([Muñoz Barón, Wyrich and Wagner 2020](https://arxiv.org/pdf/2007.12520)).*
- A function SHOULD be 60 lines or fewer after formatting, from its signature to its closing brace. The more nesting it has, the shorter it should be. A table, or a `switch` with one line per case, MAY be longer. Don't split a readable loop into helpers the reader has to chase. *Google suggests splitting functions past about 40 lines ([Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html#Write_Short_Functions)), and splitting too far makes code harder to follow ([Ousterhout and Martin](https://github.com/johnousterhout/aposd-vs-clean-code)).*
- A function SHOULD declare 7 local variables or fewer. *People can only keep a few things in mind at once ([Miller 1956](https://doi.org/10.1037/h0043158), [Cowan 2001](https://doi.org/10.1017/S0140525X01003922)).*

## Names

- New functions, variables and members MUST be `snake_case`. New types MUST be `PascalCase`, even next to older snake_case types like `pose`. Constants and macros MUST be `UPPER_SNAKE_CASE`. *Naming style matters most to beginners ([Binkley et al. 2013](https://www.cs.kent.edu/~jmaletic/cs69995-PC/papers/EMSE12.pdf)). Functions and variables match PROS and the public API, and a different case for types lets a reader tell a type from a value.*
- New internal names MUST use full words. The only short forms allowed are `pid`, `imu`, `odom`, `rpm`, `kp`, `ki`, `kd` and `xy`. Words like `pp`, `ptp` and `mA` are spelled out in new internal names, even next to existing names that use them. New public names follow the public names next to them until issue 330 renames the public API, so teams see one pattern in autocomplete. *Full-word names helped developers find bugs about 19% faster ([Hofmeister, Siegmund and Holt 2017](https://www.se.cs.uni-saarland.de/publications/docs/HoSeHo17.pdf)). Common short forms are nearly as clear, so a short allowed list is fine ([Lawrie et al. 2007](https://www.cs.kent.edu/~jmaletic/cs63902/Papers/Lawrie07.pdf)).*
- A public function's name SHOULD start with the thing it acts on, like `pid_drive_set` or `drive_imu_get`, so typing that word lists everything you can do with it in autocomplete.
- Before naming something new, you SHOULD find the closest existing function and follow its pattern, as long as the MUST rules above still hold. *Two developers rarely pick the same name for the same thing ([Feitelson et al. 2021](https://arxiv.org/abs/2103.07487v1)).*
- Names MUST NOT include the type, like `fSpeed` or `iCount`.

## Comments

- Comments MUST describe the code as it is now. The story of a fix goes in the commit message and PR description. *Comments that fall out of date with their code are followed by more bugs ([2024 study of 8 Apache projects](https://arxiv.org/pdf/2409.10781)), and history is the kind most likely to go stale.*
- A comment SHOULD say why the code does something. If what it does needs explaining, rename things or split the function until it doesn't. *A name can't drift away from the code the way a comment can ([Fowler, FunctionLength](https://martinfowler.com/bliki/FunctionLength.html)).*
- Code that looks wrong but is right MUST have a comment saying why, with the issue number if there is one, like a delay that works around a VEXos quirk. Otherwise someone will remove it.
- **Example project.** Each step of every example auton SHOULD have a short comment above it naming the step, like `// Drive to the goal`. *Students given step labels like these got further on each task and remembered more a week later ([Margulieux, Guzdial and Catrambone 2012](https://scholarworks.gsu.edu/server/api/core/bitstreams/ea5acfed-1bc8-4169-bc16-b2f990f31f2a/content), [Morrison, Margulieux and Guzdial 2015](https://scholarworks.gsu.edu/server/api/core/bitstreams/f40a79f2-edaf-49fb-b0cb-05451bd21bac/content)).*
- **Example project.** A commented-out line MUST be an option a team can turn on, and the comment MUST start with `Option:`. Any other commented-out code MUST be deleted. *PROS deletes commented-out code ([PROS style guide](https://github.com/purduesigbots/pros/blob/develop/STYLEGUIDE.md)). In the example project those lines are choices for teams, so they stay, labeled so nobody mistakes them for dead code.*
- A rule that code can check SHOULD be checked in code, with `static_assert`, a type, or a warning. The compiler won't skip it. People skip comments.

## Doc comments

Every public function's comment is an entry on the docs page, written for teams. Use short sentences and no jargon.

- The first sentence is the summary. It MUST say what the function does.
- Every parameter MUST have a `\param`.
- An example goes in `\code ... \endcode`. It MUST compile on its own in a fresh example project, so declare anything it uses, like `DRIVE_SPEED`. It MUST be 100 columns or fewer and formatted with clang-format.
- A doc comment MUST NOT hold maintainer-only notes, like a compiler workaround or the story of a fix. Put those in a normal `//` comment.
- The doc example check in CI compiles the examples, once it's on `dev`.

## Function parameters

- A new public function MUST NOT add a bare `bool` parameter. *`drive_set(true)` tells the reader nothing ([Fowler, FlagArgument](https://martinfowler.com/bliki/FlagArgument.html)).* What replaces bare bools and long lists of numbers in the public API will be decided as part of the renaming in issue 330.
- A new function whose name is a question (`is_`, `has_`) MUST return `bool`. *Developers rate names that promise something the code doesn't do as poor practice ([Arnaoudova, Di Penta and Antoniol 2016](https://publications.polymtl.ca/1517/1/2014_VeneraArnaoudova.pdf)).*

## Don't crash the robot

- Library code MUST NOT stop the program or throw on bad input. It prints a warning once per cause, uses a safe value (usually the previous one), and keeps running.
- A mistake that can be caught at compile time SHOULD be, with `static_assert` or a type. Students often don't read runtime logs, but they can't miss a compile error.

## Shared state between tasks

- Data that two tasks read or write MUST be a `std::atomic` or be protected by an `ez::Lock`, and the code MUST say which lock protects it. Outside `lock.hpp`, other lock types (`pros::Mutex`, `std::lock_guard`) MUST NOT be used.
- A lock MUST be taken with `ez::KillSafeGuard`. Only the functions listed in `kPlainGuardFunctions` in `test/test_locking_rule.cpp` (today `Drive::ez_auto_task` and the LVGL `lines_replay` callback) MAY use `ez::PlainGuard`. *If PROS deletes a task while it holds a plain lock, the lock stays locked forever and every later setter hangs.*
- Code inside a guard MUST NOT block. That means no `printf`, `std::cout`, `pros::delay`, `pros::screen`, or a different lock. To print, call `lock.print_after_unlock(...)`. *The guard is only safe while the task holding it keeps running.* `test/test_locking_rule.cpp` checks this by reading the source, so it can't see inside a helper function. A helper that only runs with the lock held MUST be added to that test's `kLockedHelpers` list.
- `volatile` MUST NOT be used to share data between tasks. *It doesn't make reads and writes between tasks safe ([C++ Core Guidelines CP.200](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rconc-volatile2)).*

## Use what's already there

- Before writing a helper, you SHOULD check `include/EZ-Template/util.hpp` and EZ-Units for one that already exists. *A fix to one copy misses the others ([Fowler and Beck, Duplicated Code](https://refactoring.com/catalog/)).*
- You SHOULD NOT add a parameter, option or layer that nothing uses yet. *Code nothing uses never gets tested ([Fowler, Yagni](https://martinfowler.com/bliki/Yagni.html)).*

## How this page changes

A new rule needs a reason, ideally a study, a real bug, or the same review comment on 3 or more PRs. Propose it in an issue, and the maintainers decide. If a tool can enforce the rule, the tool goes in the same PR.
