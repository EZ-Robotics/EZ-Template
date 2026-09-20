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

PROS deletes the running competition task on every mode change, without releasing any mutex it was holding.
A lock that waits forever can leave the background drive task, and with it odometry and every PID motion, dead
until the brain restarts. So every lock in the library follows two rules (`include/EZ-Template/lock.hpp`):

- Use an `ez::RecoverableMutex` taken with an `ez::LockGuard`. Never a bare `pros::Mutex`, `pros::RecursiveMutex`,
  `std::lock_guard`, or `take()`/`give()`.
- Do nothing slow while holding one: no `printf`, no `delay`, no waiting on hardware. To print from inside a lock,
  use `guard.print_after_unlock(...)`.

`make -C test` fails if either rule is broken (`test/test_locking_rule.cpp`).

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
