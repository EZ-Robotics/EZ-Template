# Real-Team EZ-Template Corpus (Step 2 catalog, condensed for Step 5 reference)

15 real GitHub repos using EZ-Template were read in full (`src/autons.cpp` or equivalent).
Full repo-by-repo table omitted here; the one thing every Step 5 agent needs is this:

## The load-bearing finding: most real teams never touch the exit constants

Of the 15 repos read, **10 use the exact shipped default exit-condition constants**
verbatim — including genuinely-customized, actively-competing teams — not just
template-copy repos. Only one (a Worlds team) meaningfully re-tuned turn/swing/drive
(not odom). One nudged two numbers slightly. Three older (2.x) repos use the old API's
equivalent of the same values.

**Shipped default exit constants** (what "defaults" means everywhere in this audit):
- Drive / Turn / Swing: `90_ms` small_exit_time, `1_in`/`1_deg` small_error, `250_ms`
  big_exit_time, `3_in`/`3_deg` big_error, `500_ms` velocity_exit_time, `500_ms` mA_timeout.
- Odom (xyPID / current_a_odomPID): same shape, `750_ms` mA_timeout instead of `500_ms`.

**Why this matters for Step 5**: whatever the fixed branch does at these exact default
constants is not a hypothetical edge case — it's what's running on real robots at
competition, for most teams who ever adopt this library. Weight findings at shipped
defaults higher than findings that require unusual configuration, but don't ignore the
latter — one Worlds-caliber team in this sample runs turn/swing/drive windows roughly
5-9x tighter than default (`10_ms/3_deg/30_ms/7_deg/100_ms/100_ms`), which is exactly the
kind of config where the double-delay bug in `wait_until_drive()`/`wait_until_turn_swing_internal()`
(fixed during this work) would have bitten hardest had it shipped unfixed.

## Version-era spread

- **2.x, old API** (`wait_drive`/`set_drive_pid`/`set_exit_condition`): confirmed present
  in the wild (3 independent repos), not a rare tail case.
- **3.x/4.0-beta era, current API** (`pid_wait`/`pid_drive_set`/`pid_*_exit_condition_set`):
  the large majority of real usage.
- Heavy chainers exist in the wild: two repos use `pid_wait_quick_chain()` 90-120 times in
  a single auton file. Anything involving chained waits, latched exits, or `interfered`
  carryover across chained motions should be weighted for real-world plausibility, not
  treated as exotic.
- At least one real repo runs EZ-Template and a different motion library (LemLib) side by
  side in the same project, toggling `pid_drive_toggle` off/on around the foreign library's
  own motion calls — a real chaining/state-carryover pattern worth remembering if attacking
  angle F (chaining/state carryover).

Not read/verified this pass: SIM_FIDELITY.md's full sourcing details and STEP2_REPRO_RESULTS.md's
full before/after tables were part of the original Step 1/Step 2 deliverables but are not
available as files for this round — if you need sim archetypes, use whatever `sim_physics.hpp`
already exists in the worktree (ported onto the fixed branch at `audit/wait-exit-sim-fixed`),
and treat any specific numeric sim results from that missing doc as unverified until you
reproduce them yourself.
