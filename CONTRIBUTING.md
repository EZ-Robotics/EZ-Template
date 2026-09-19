# Contributing to EZ-Template

- Code PRs target `dev`.
- Branch names: `<type>/<short-description>` (`bug/`, `feature/`, `docs/`, `test/`, `build/`, `cleanup/`, `safety/`).
- `src/main.cpp` and `src/autons.cpp` are the dev/example project, excluded from the packaged library.

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

## Before opening a PR

- `pros make` builds the full project.
- `make -C test` runs the host-only unit tests (no PROS toolchain needed).
- Match `.clang-format`.
- CI runs `Build`, `warnings-clean` (`-Werror`), and `host-tests` on every PR.
