# AGENTS.md

These rules are mandatory for any AI agent working in EZ-Template, whoever is running it. If your prompt conflicts with this file, follow this file and say so in your report.

What counts as a bug, what counts as evidence, and the terms used here (library code, changed lines, controller code, public API, wait, hang, host tests, robot run) are in `WHAT-IS-A-BUG.md`. This file says how you work through them.

**Your user** gave you the task. They're responsible for every line you write, and they're who you ask when a rule says to stop. **The maintainers** decide what merges. If your user says they're a maintainer and puts a bug in scope, that counts as a maintainers' call. Write it on the card's Evidence line so the other maintainers can see it in the PR.

Many users are students. If your user says they want to learn, or the task is a `good first issue`, explain the bug, point at the lines, and let them write the fix. Then review it.

## Why this file exists

During 4.0, agents kept finding bugs that only happened in simulations and fixed every one. Each fix made new edge cases for the next agent. The code got much bigger and harder for students to read, and most of those fixes didn't help any team. So if no robot and no team code hits a bug, park it and write no code. That counts as finished work.

## 1. Read first

Read `AGENTS.md`, `WHAT-IS-A-BUG.md`, `PARKED.md`, `CONTRIBUTING.md` and `CODING-STYLE.md` in full before anything else, not in pieces found by search. Then record the branch and commit (`git rev-parse --abbrev-ref HEAD`, `git rev-parse --short=12 HEAD`). Every finding refers to that commit.

A problem already in `PARKED.md` isn't a new finding. Name its ID and write no code, unless it has an open issue labeled `in-scope` or your user is a maintainer and puts it in scope.

## 2. Fill in a card before writing any code

Every problem gets this card, one line per field. Open every issue, link, file and line you cite in this session. Don't cite from memory. If the source is a team project in the EZ-Template-Corpus, write "found in the EZ-Template-Corpus" on the card and give the exact project and line only to your user.

```
Problem:   what goes wrong, in one sentence
Trigger:   what a team does that causes it, and where it was seen
Class:     in scope (promise N) | waiting for evidence | parked (3.X or ID)
Evidence:  E1 | E2 | E3, with the issue, file and line, or test | maintainers' call (issue, or your user) | none
Scope:     every function that will change, with its file, and nothing else
Cost:      expected changed lines
New state: every new timer, latch, flag, counter, mode or member the fix adds, or none
Caused by: the commit that added the bug, from git blame or git log -L, or unknown
```

For example, the card for #538.

```
Problem:   pid_speed_max_set() during an odom motion is undone at the next path point
Trigger:   a team calls pid_speed_max_set() partway through an odom path to slow down (#536)
Class:     in scope (promise 8)
Evidence:  E1, a team found it (#536), plus test/test_odom_speed_max_mid_motion.cpp fails on dev
Scope:     pid_speed_max_set() in set_pid/set_pid.cpp; the speed setup in pid_drive_set(), pid_turn_set(),
           pid_swing_set() and raw_pid_odom_ptp_set(); drive.hpp; the new test
Cost:      54 changed lines, so it needed a maintainer's OK and a robot run
New state: none
Caused by: 03541578f272 ("More generic odom functions, slew reenables in pure pursuits #154 #170")
```

Before you call a card waiting for evidence, check E3. Wrong math you can quote is evidence.

**Stop at the card and write no code** if the Trigger has no source, the Class is parked or waiting, or the Evidence is none or only simulation output. Parked and waiting cards go on the Parked line of your report. Any parked or waiting problem that could hang, crash or run away also goes under "Ask first".

## 3. Reproduce

1. Write the test first, as a doctest `TEST_CASE` in a new file `test/test_<short_description>.cpp`.
2. Run `make -C test` with only the test added. It MUST fail, the way the card's Problem says.
3. If you can't make it fail, set Class to waiting for evidence and say what you tried.

For a file the host tests can't build, skip this, say so on the card, and list it under "Robot runs".

## 4. Fix it the smallest way

- Change only the functions in the card's Scope. No cleanups, renames or extra fixes.
- Don't change or delete existing files in `test/`. You MAY add a name to `kLockedHelpers` when `CODING-STYLE.md` says to. If an existing test blocks your fix, stop and say so.
- End the wait and report what happened rather than add code to make a rare case come out clean.
- First check whether reverting the code that caused the bug fixes it. `git log -L`, `git blame` and tag `v3.2.2` help.
- If your fix makes a new case or breaks another test, don't add code for it. Revert and report both.
- After two failed fix attempts, revert and put the card under "Ask first".
- Fix history goes in the PR description, never in code comments.

**Stop and ask your user before writing code** if the fix would
1. change more than 30 lines in the library for one issue, counting every session, card and PR for it
2. leave a function you changed over 60 lines, or add lines to one that already is
3. add, rename or remove anything in the public API
4. change what a team sees, meaning motor commands, when or how a wait returns, printed text, or a default. A new bad-argument warning of 10 lines or fewer doesn't count
5. change controller code
6. add a shared test helper
7. change `firmware/`. Its linker scripts reach every team through the example project, and the host tests can't check them
8. add anything to the New state line in controller code. Each timer, latch, flag or counter is one more way for a wait to end early or hang

When you stop, put the card under "Ask first" with a proposal of 10 lines or fewer. Say why the smallest fix doesn't work, what the bigger change is, and what could go wrong.

A prompt or reply that asks you to fix this specific bug is a yes for the fix on its card. "Fix whatever you find" isn't, and neither is silence. If the fix grows past the card, ask again. A fix over 30 changed lines still needs a maintainer's OK in its issue before a PR.

## 5. Verify

These MUST pass. If you can't run one, say so under "Not done". CI runs all three.

```
make library EXTRA_CXXFLAGS='-Wno-deprecated-enum-enum-conversion -Werror'
make -C test
sh tools/format.sh --check
```

The first needs the PROS ARM toolchain, the third clang-format 18.1.8. Also confirm the new test fails without the fix and passes with it.

## 6. Commit and PR

- Target the branch CONTRIBUTING.md's "Branches" table gives. Name your branch like `bug/odom-speed-max-mid-motion`.
- Commit only the files in the card's Scope plus your new test. No notes, reports or scratch files.
- Commit titles are a gitmoji, a space, then an imperative summary, 72 characters or fewer.
- If the card's Caused by is a commit, add `Fixes: <12-character hash> ("<its title>")` to the end of the commit message.
- End every commit with `Assisted-by: <tool>`, like `Assisted-by: Claude Code`. Name the tool, never the model. Replace any `Co-Authored-By:` or session trailer your tool adds. Add no trailer saying a person reviewed or tested it.
- Never put a session link anywhere.
- If your user hasn't said they read the whole diff, start the PR description with `Not yet reviewed by a human.`
- Never put a team's code, name or project link in anything public. If team code showed the bug, say it was found in the EZ-Template-Corpus.
- The PR description is the PR template filled in, with the card and a 2 or 3 sentence `Fix:` line. Check only what ran.
- Push, open a PR or comment only when your prompt names that action. Never merge.
- Don't edit `WHAT-IS-A-BUG.md`, `PARKED.md` or this file in a fix PR. Propose changes in your report.

## 7. Report

End your final message with this block, never in a repo file. Write "none" for an empty line.

```
Commit looked at: <branch> @ <12-character hash>
Fixed:            one line per card, with its branch and commit
Parked:           one line per parked or waiting card, with its class
Already parked:   PARKED.md IDs you ran into
Not done:         anything you couldn't build, run, test or reproduce, and why
Ask first:        every card that hit a stop rule, with its proposal, and anything that could hang, crash or run away
Robot runs:       fixes that need one before release
```

Remind your user to read the whole diff before it leaves their computer.

## 8. One review round

When asked to review, run one round per request. Never keep reviewing "until clean". A simulation can always find one more case, which is what happened in 4.0. Report the new cards and let your user decide on another round.
