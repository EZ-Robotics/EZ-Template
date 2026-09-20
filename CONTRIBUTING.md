# Contributing to EZ-Template

- Code PRs target `dev`.
- Branch names: `<type>/<short-description>` (`bug/`, `feature/`, `docs/`, `test/`, `build/`, `cleanup/`, `safety/`).
- `src/main.cpp` and `src/autons.cpp` are the dev robot config, excluded from the packaged library.
- The example project lives on the `example-project` branch, which never merges into `dev` or `main`. PRs that change it target `example-project`, not `dev`.

## Commits & PR titles

Lead with a [gitmoji](https://gitmoji.dev):

| | |
| --- | --- |
| 🐛 | Bug fix |
| ✨ | New feature |
| 💥 | Breaking change |
| ♻️ | Refactor |
| 🔥 | Remove code |
| 🚨 | Fix warnings |
| 🧪 | Tests |
| 📝 | Docs |
| 👷 | CI/build |
| 🔧 | Config |
| ⬆️ | Upgrade dependency |
| ⏪️ | Revert |

## Locking

PROS deletes the running competition task on every mode change, wherever it happens to be, and deleting a task does
not release a mutex it was holding. A task that dies inside a chassis setter leaves the chassis lock taken for good,
and the background drive task, and with it odometry and every PID motion, waits forever. So every lock in the library
follows two rules (`include/EZ-Template/lock.hpp`):

- Take every lock with an `ez::KillSafeGuard`. It raises the task above the PROS system daemon for as long as it holds
  the lock, and the daemon cannot preempt a higher priority task, so it cannot delete the task while it holds the lock.
  The two places PROS never deletes the task (`ez_auto_task` and the display's LVGL callback) use an `ez::PlainGuard`,
  which locks the same way without changing the priority. Never use a bare `pros::Mutex`, `pros::RecursiveMutex`,
  `std::lock_guard`, or `take()`/`give()`.
- Nothing inside a guard may block: no `printf`, `std::cout`, `pros::delay`, `pros::screen`, screen printing, or another
  lock. A task that blocks lets the daemon run, and the daemon may then delete it while it holds the lock. To print from
  inside a guard, use `drive_mutex.print_after_unlock(...)`: the text is printed once the outermost guard has released.

`make -C test` fails if either rule is broken (`test/test_locking_rule.cpp`). It reads the source, so it cannot see a slow
call made by a function that a locked function calls. If you write a helper that only ever runs with the lock held and
prints or waits, add it to `kLockedHelpers` at the top of that test.

What this cannot do:

- It depends on the PROS system daemon staying below the priority a guard raises to. `ez_auto_task` prints a warning at
  startup if it does not.
- A task that is running on an inherited priority cannot be shielded, because `task_set_priority` only changes the
  priority a task is using when it is not inheriting one. The guard notices, puts the priority back, counts it
  (`ez::detail::stats.unshielded_raises`), and takes the lock anyway. The public API cannot read a task's base priority,
  so putting it back leaves the task with the inherited priority afterwards. This happens when a task calls a chassis
  function while holding its own mutex that a higher priority task is waiting for. Do not do that.
- Reading a sensor inside a guard can block on that sensor's port mutex, and the daemon takes every port mutex each time
  it runs. A setter that blocks there during the same 2 ms as a mode change is still exposed.
- PROS's own `printf` and `pros::screen` mutexes can be orphaned by a mode change. No library can fix that.

## Before opening a PR

- `pros make` builds the full project.
- `make -C test` runs the host-only unit tests (no PROS toolchain needed).
- Match `.clang-format`.
- CI runs `Build`, `warnings-clean` (`-Werror`), and `host-tests` on every PR.

## Merging

- Regular `dev` PRs: use **Squash and merge**, so `dev`'s history stays one
  gitmoji-prefixed commit per PR.
- The `dev` → `main` release PR: use a regular merge, to keep `dev`'s full
  commit history intact on `main`.
