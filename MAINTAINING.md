# Maintaining EZ-Template

This page is for maintainers. The rules for what gets fixed, and the terms used here, are in [WHAT-IS-A-BUG.md](WHAT-IS-A-BUG.md).

## Maintainers

| Maintainer | GitHub |
| --- | --- |
| Jess | @ssejrog |
| jordan shastalebab | @shastalebab |
| William Sellery | @nightwolf3140 |
| Jonathan Emminizer | @jemminiz |
| Hakop Zarikyan | @HakopZ |

Every maintainer can review, merge, release, and change anything in the project, including merging their own PRs.

New maintainers are invited by the current maintainers.

## Reviewing

Check each PR against these, in order, and say on the PR which one failed.

1. **Issue.** The PR follows CONTRIBUTING.md's "Fix it yourself" rules. A first-timer who skipped an issue MAY have the PR thread treated as the issue.
2. **Evidence.** A bug fix has E1, E2, E3, or a maintainers' call. If it doesn't, label it `parked` or `waiting-for-evidence` and say kindly what's missing. Ask a first-timer for the evidence before closing anything.
3. **Size.** Be wary of very large PRs, especially AI-written ones. You MAY ask for one to be split, cut to what its issue asked for, or closed if the author can't explain it.
4. **New moving parts.** Count any timer, latch, flag, counter or mode the PR adds to controller code. Each one can make a wait end early or hang, and students have to learn it. Ask whether 3.2.2 needed it. One added for a case only a test or sim shows is parked, however small. If a fix needs a second fix for a case the first one created, rethink the first one.
5. **Test.** For a contributor's bug fix, add a host test that fails without the fix. Check locally that it fails on `dev` and passes with the fix, write that on the PR, and push the test to the PR branch as its own commit. Don't rewrite the contributor's commits. An AI-written test is fine if it fails without the fix, passes with it, and the author can explain it. A test for E2 MUST NOT use a robot model.
6. **Checks.** Every GitHub check passes. If only the format check fails on a contributor's PR, you MAY run `sh tools/format.sh` and push it.
7. **Controller code** and files the host tests can't build get `needs-hardware-test`.
8. **Claims.** A PR that says a robot drives better MUST show numbers from a robot run.
9. **Behavior.** A PR that changes how a robot drives gets `behavior-change`, and the release notes MUST describe it.

Be kind, especially to first-timers. Keep comments about the code. If a first-timer's PR is close, give hints and let them make the change.

### New issues

Each new bug report gets one of these, ideally within a week.
- **Needs info.** Ask for what the form is missing. It MAY be closed after 30 days with no reply.
- **Already parked.** Link the PARKED.md row, add `parked`, and thank them. If they saw it on a robot, that's E1, so decide whether it moves into scope.
- **In scope.** Add `in-scope`. If it's small and needs no robot run, also add `good first issue` and name the function to change.
- **Waiting for evidence.** Add `waiting-for-evidence` and say what evidence would move it.
- **Parked.** Add `parked` and add a PARKED.md row in its own PR.
- **Regression.** Add `regression` the same day.

When a parked bug moves into scope, give it an open issue labeled `in-scope`. Agents only work on parked bugs that have one. Questions about using EZ-Template go to the Discord.

### AI-heavy PRs

The more of a PR a tool wrote, the closer you check it. You MAY ask the author to explain any line, ask for the prompt, ask for a robot run, review it after human-written PRs, suggest a better prompt instead of code changes, or close it without a detailed review if nobody can explain it. Don't use AI detectors. They don't work.

AI review bot comments are advisory. Tell the author which ones need an answer.

A PR starting with `Not yet reviewed by a human.` came from an agent. Ask the person who ran it to read the whole diff and delete that line before you review.

To close a PR nobody can explain, something like this works. "Thanks for this! We're closing it because the review questions couldn't be answered, and we only merge code someone here understands. You're welcome to open a smaller version you can walk us through."

### Labels

`bug` (not yet judged), `in-scope`, `waiting-for-evidence`, `parked`, `behavior-change`, `regression` (worse than an earlier stable release), `needs-hardware-test` (needs a robot run before release), `good first issue` (30 changed lines or fewer, no robot run), and `enhancement` (a feature, or a change that breaks no promise).

## Merging

PRs into `dev` are squash merged. The maintainer who merges writes the message.
- **Title.** A gitmoji, a space, then an imperative summary, 72 characters or fewer, like `🐛 Keep pid_speed_max_set() for the rest of an odom motion`. The common ones are under Gitmoji below.
- **Body.** For a bug fix, what went wrong, what triggers it, the evidence, and what changed. Paste the agent's card if there is one. Otherwise 1 to 3 sentences.
- **Trailers**, only the ones that apply.

```
Fixes: <12-character hash> ("<title of the commit that caused the bug>")
Closes: #<issue number>
Reported-by: <name or handle>
Tested-by: <someone who did a robot run of this exact change>
Co-Authored-By: <a person> <email>
Assisted-by: <AI tool, like Claude Code, one line per tool>
```

The person who opened the PR stays the author. Never include a session link.

The release PR from `dev` into `main` is a merge commit. Process doc PRs go into `main`, and after one merges, merge `main` into `dev`. Nobody force-pushes a shared branch or someone else's PR branch. Undo merged changes with `git revert`. After a rebase, the checks MUST pass again before merging.

### Gitmoji

| Gitmoji | Use it for |
| --- | --- |
| 🐛 | A bug fix |
| ✨ | A new feature |
| 📝 | Docs |
| ✏️ | A typo |
| ♻️ | Reorganizing code without changing what it does |
| ⚡️ | Making something faster |
| ✅ | Tests |
| 🎨 | Formatting |
| 🔥 | Removing code or files |
| 💥 | A change that breaks teams' code |
| ⏪️ | Undoing an earlier change |
| 🔖 | A release |
| 🔧 | Config or build files |

The full list is at [gitmoji.dev](https://gitmoji.dev).

## Releases

| Release | Example | What it's for | Can it change how a robot drives? | Can it stop code compiling? |
| --- | --- | --- | --- | --- |
| Patch | 3.2.1 to 3.2.2 | Bug fixes | Only to fix a broken promise, with release notes | No |
| Minor | 3.1.0 to 3.2.0 | New features, off by default | Same as a patch | No |
| Major | 3.2.2 to 4.0.0 | Big changes that can break teams' code | Yes, with every change in the release notes | Yes, with a message saying what to write instead |
| Pre-release | 4.0.0-beta.5 | Testing a release before it's stable. Alphas come first, then betas, then release candidates (`rc`) | Yes, with every change in the release notes | Yes, with a message saying what to write instead |

**What goes in a patch.** It's merged into `dev` (or `release/X.Y` if the code is gone from `dev`), it's 100 changed lines or fewer, it has a test and any robot run it needs, and it fixes a hang, crash, build error, regression, or wrong auton.

**Which versions get fixes.** The newest stable release and the newest pre-release. The last stable release of the previous season MAY get one patch for a regression that stops teams scoring at events. When someone asks for a fix on an older version, answer in the PR either way.

**Fixing an older stable release.** Make `release/X.Y` from its tag the first time, like `release/3.2` from `v3.2.2`. Fix on `dev` first and `git cherry-pick -x` into a PR to `release/X.Y`, or fix there directly if the code is gone from `dev`. Tag the patch on that branch, and uncheck "Set as the latest release" if a newer stable release exists.

**Timing.** As the season goes on, merge fewer risky changes. Don't release in the 7 days before a major event like States or Worlds, unless it fixes a regression blocking teams there.

**How to release**
1. Go through the checklist below.
2. Bump `VERSION` in the `Makefile`, run `sh tools/gen-version.sh`, and commit the three changed files in their own PR, like #561.
3. Tag a pre-release on `dev`. For a stable release, merge a release PR from `dev` into `main` and tag the merge commit.
4. Build the template with `pros make template` from the tag.
5. Build the example project zip from `example-project`, named exactly `EZ-Template-Example-Project.zip`.
6. Make the GitHub release with both zips and the notes. Check "Set as a pre-release" for alphas, betas and RCs.
7. Update the docs site.
   1. On `website-dev`, run `npm run api:update -- vX.Y.Z --use`. Use Doxygen 1.18.0, so snapshots only change when the headers do.
   2. Update the version label in `docusaurus.config.js`.
   3. Commit, and open a PR into `website-dev`.
   4. Promote `website-dev` into `website` with a merge commit, never a squash.
   5. At a stable release, also do the `docs:version` step in the `website-dev` README.
8. Post in the Discord.

**Before every release**
- [ ] No open `regression` issues, or a maintainer wrote on each why it can ship
- [ ] Every `needs-hardware-test` PR in it has a robot run written on it
- [ ] Every check passes, and `example-project` builds against it
- [ ] The notes thank every contributor by GitHub name
- [ ] The notes describe every behavior change in words a team can act on, like "waits end about 50 ms later on heavy robots"
- [ ] The notes list everything deprecated and what to use instead

### Fixing the live docs between releases

Check whether `website-dev` has changes that aren't released yet.
- **Nothing unreleased.** Fix it on `website-dev` and promote as usual.
- **Something unreleased.** Branch off `website` and open a PR with the fix into `website`, merged with a merge commit. Then merge `website` into `website-dev`. This is the only time `website` goes back into `website-dev`.

Wording of a function entry comes from the header comment, so it can only change on the live site with a new snapshot.

## Regressions

A regression is something that worked in an earlier stable release and works worse in a later one, on the same robot and code. For example, the robot ends 1 in or more farther from its target over 5 runs of each version, a motion stops short, a wait returns 100 ms or more later or hangs, or code stops compiling without a message saying what to write instead. "Just retune" doesn't make it not a regression.

It's not a regression when a major version or pre-release lists it as a behavior change, when the old behavior broke a promise and the fix's notes say what changed, or when the release where it got worse no longer gets fixes.

Regressions come before other work. If the commit that caused it reverts cleanly, revert first and fix properly after.

## Public API

A new public function needs an issue with an example of team code that needs it, and MUST follow these.
1. It works with only the default constructor.
2. It's simple enough that teams won't wrap it.
3. Its name starts with what it acts on (issue 330), so autocomplete lists it.
4. A length, angle, time or speed parameter gets an EZ-Units overload.

Public names are only renamed or removed in a major version. The old name SHOULD keep working with `[[deprecated("use <new name>")]]` for a full season. If it can't, it MUST fail to compile with a message saying what to write. The release notes MUST list it.

## Requirements

| Needed | Version |
| --- | --- |
| PROS kernel | 4.2.2 or newer |
| ARM GCC, for `--std=gnu++26` | 14 or newer |
| Desktop compiler for host tests | Any g++ or clang++ with C++20 |
| clang-format | 18.1.8 |

If this table and the repo's files disagree, the files are right.

## Every season

- Reread the competition AI policies linked in CONTRIBUTING.md and check the links still work.
- Reread AGENTS.md and WHAT-IS-A-BUG.md against the past season's PRs, and change what kept going wrong, each in its own PR.
- Close or relabel stale `waiting-for-evidence` issues.
- Remove a project from the [EZ-Template-Corpus](https://github.com/EZ-Robotics/EZ-Template-Corpus) when someone asks. It adds new projects and drops private or deleted ones by itself.
- Check that every label exists and the docs site's "Edit this page" link opens `website-dev`.
