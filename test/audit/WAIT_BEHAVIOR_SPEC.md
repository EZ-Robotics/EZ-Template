# Wait Behavior Spec (Step 0, draft v4 — final before presenting)

Status: **draft, not signed off.** Nothing beyond this document has started. This went
through four passes; each of the first three had errors caught before being shown to you.
v4 fixes the load-bearing facts (setter reset behavior, boomerang's actual mode, what the
left/right PIDs in odom actually aim at, StuckWatch's real vs. carrot target) by reading
the setter bodies directly rather than inferring, and gives every judgment call a proposed
default as requested. I'm presenting this one — further correction is welcome from you, not
from another internal review pass; diminishing returns on doing that to myself.

Grounded against `bug/odom-wait-stuck` @ `8aed78a` (confirmed byte-identical to PR #504's
actual head `daa5bd6` for the three core files). Also read in full for this pass:
`set_turn_pid.cpp`, `set_swing_pid.cpp`, `set_odom_pid.cpp` (all of it, including
`raw_pid_odom_ptp_set`'s body), `drive.cpp`'s sensor getters, `purepursuit_math.cpp`.

---

## 1. Vocabulary

- **Exit types**: `RUNNING`, `SMALL_EXIT`, `BIG_EXIT`, `VELOCITY_EXIT`, `mA_EXIT`,
  `ERROR_NO_CONSTANTS`.
- **`timers_reset()`**: runs inside `PID::exit_condition()` before returning any
  non-`RUNNING` value, including a `VELOCITY_EXIT` an odom caller discards via
  `without_velocity()` (F5, §6).
- **What actually resets at the start of a new motion** (grepped across all four setter
  files, not assumed): every top-level `pid_*_set()` — `pid_drive_set`, `pid_turn_set`
  (`TURN`), the turn-to-point setter (`TURN_TO_POINT`, same file), `pid_swing_set`,
  `pid_odom_set`/`pid_odom_ptp_set`/`raw_pid_odom_pp_set`/the pure-pursuit path-injection
  setter — clears `interfered`, calls `timers_reset()` + `motion_reset()` on the PIDs that
  motion uses, and (odom pure-pursuit only) repopulates `injected_pp_index` and resets
  `pp_index = 0`. **`raw_pid_odom_ptp_set()` — the function `boomerang_task()` calls every
  pass as the carrot moves, and `pp_task()` calls on every waypoint advance — does *not*
  call any of that.** It only updates targets/constants (confirmed by reading its full body:
  `xyPID.constants_set(...)`, `odom_target.x/y = ...`, `leftPID.target_set(...)`, no
  `timers_reset`/`motion_reset` calls anywhere in it). So a boomerang's continuous carrot
  updates, and pure pursuit's index advances, do **not** reset `xyPID`/`current_a_odomPID`'s
  exit timers or `velocity_armed` mid-motion — only the one-time top-level setter does, at
  the very start. This is good news for `StuckWatch`'s integrity (its underlying PID state
  isn't getting wiped every pass) and confirms the JC-5 latch finding is real, not an
  artifact of frequent resets.
- **`velocity_armed`**: inert until derivative exceeds `velocity_zero_main` once, or
  1000 ms since the last `timers_reset()`.
- **`velocity_exit_hold`**: `ptp_task()` freezes both velocity timers while turn bias has
  zeroed `xy_out`. Capped at 2000 ms.
- **`without_velocity()`**: odom's `pid_wait()`/`pid_wait_until_point()` main loops map
  `VELOCITY_EXIT` back to `RUNNING`. **`wait_until_drive()`'s `on_last_point` failsafe check
  does not** — it reads `xyPID.exit_condition()` raw (line 365), so a velocity exit *can*
  end that specific wait where the others wouldn't.
- **`StuckWatch`**: fires when distance-to-target, heading error, and pure-pursuit index
  have all failed to improve by a full step within `window_` ms (`velocity_exit_time` if
  nonzero else `mA_timeout` else permanently inert). Confirmed this pass: its
  `target_distance()`/`point_distance()` inputs (in `pid_wait()`, `pid_wait_until_point()`,
  `pid_wait_until_index_started()`) all read `pp_movements[pp_index].target` or the
  explicit `target`/point argument — **the real waypoint, never the boomerang carrot** (the
  carrot is a local variable in `boomerang_task()` that only ever reaches `odom_target`, not
  `pp_movements[pp_index].target`). So `StuckWatch` and the `pid_wait()` "settled" check
  (§4) are always judging against the real target. **Superseded 2026-09-24 (see §8.2):**
  this bullet originally claimed only `xyPID`'s own small/big exit was carrot-relative during
  boomerang. Round 3 of the audit found `current_a_odomPID`'s own small/big exit is
  carrot-relative too (`raw_pid_odom_ptp_set()` retargets both `odom_target` and
  `point_to_face` from the carrot every pass) — **both axes are carrot-relative**, not just
  xy. This is now accepted, out-of-scope behavior; see §8.2 for the full correction and why.
- **`interfered`**: cleared by every top-level `pid_*_set()` (confirmed all four setter
  files this pass), never by a wait returning cleanly. **P8.**
- **Modes**: `DISABLE`, `SWING`, `TURN`, `TURN_TO_POINT`, `DRIVE`, `POINT_TO_POINT`,
  `PURE_PURSUIT`. **Boomerang is `PURE_PURSUIT`-only**, confirmed this pass:
  `is_boomerang = true` is only ever passed by `boomerang_task()` (itself only reachable
  from `pp_task()`, which only runs in `PURE_PURSUIT`); `pid_odom_ptp_set()` (the
  `POINT_TO_POINT` setter) hardcodes `is_boomerang = false`. v1-v3's "POINT_TO_POINT/
  boomerang" combined row was wrong — corrected in §4 below.
- **Odom's left/right PID targets — confirmed, not "one look-ahead ahead" as a guess**:
  `raw_pid_odom_ptp_set()` line: `leftPID.target_set(l_start + (odom_look_ahead_get() *
  dir))` (and same for right), plus `leftPID.exit = xyPID.exit` — `leftPID`/`rightPID` are
  explicitly retargeted to exactly one look-ahead distance past the motion's start, using
  `xyPID`'s exit constants, every time this function runs (motion start, every carrot move,
  every waypoint advance). This is what `wait_until_drive()` reads in `POINT_TO_POINT`/
  `PURE_PURSUIT`, and it's the literal mechanism behind N3.
- **Sensor source**: `drive_sensor_left()`/`_right()` read a tracking wheel/rotation sensor
  if configured, otherwise the drive motors' own encoders directly.

---

## 2. Scope-inventory note

Deferred to the actual Scope step. v1's branch-name-based claim was wrong; `--no-merged`
is also unreliable (misreports squash-merged branches as unmerged). The Scope step should
build its list from file history (`git log --follow -p`) as the task already specifies, and
diff any unmerged-looking branch against `origin/dev` on the specific files before treating
it as new scope. Not resolved further here.

---

## 3. Wait × mode: DRIVE / TURN / SWING / TURN_TO_POINT

| Wait | Mode(s) | Must return | `interfered` | Bound at defaults | Gap |
|---|---|---|---|---|---|
| `pid_wait()` | `DRIVE` | Both `leftPID`/`rightPID` exit. | `true` iff velocity/mA. | Settle: ~90-100 ms. Pinned (armed then stopped): ~1000+500=1500 ms. mA: 500 ms. **Spin: unbounded (JC-1).** | Motor-encoder **lift**: false success (`SMALL_EXIT`, `interfered=false`) — encoders reach target with no load. **Corrected from v3**: tracking-wheel/rotation-sensor lift is the **pinned** shape, not spin — derivative reads ~0 off the ground, so it's released via `VELOCITY_EXIT` (`interfered=true`) at the same ~1500 ms bound. |
| `pid_wait()` | `TURN`, `TURN_TO_POINT` | `turnPID` exits. | Same pattern. | Same bounds as DRIVE row. | Same JC-1 (spin unbounded); no lift-analog failure mode for turning. |
| `pid_wait()` | `SWING` | `swingPID` exits (swinging side). | Same pattern. | Same bounds. | Same JC-1. |
| `pid_wait_until(distance)` | `DRIVE` | Sign-of-error flip (crossed), or a side's own failsafe exit. | Per failsafe type; untouched on a clean cross. | **JC-4**: `wait_until_drive()` calls `pros::delay(DELAY_TIME)` twice per still-running iteration (confirmed, both call sites reread) but `exit_condition()` once — every failsafe timer here takes **~2×** its configured value in real time (pinned ~3000 ms, mA ~1000 ms). `pid_wait()` doesn't have this bug. | Proposed default: fix it (drop the redundant delay) unless there's a reason the doubled slack is wanted. |
| `pid_wait_until(angle)` | `TURN`, `SWING`, `TURN_TO_POINT` | Crossed, or failsafe. | Same pattern. | Same JC-4 doubling. | Same. |
| `pid_wait_until(distance)` | `TURN`/`SWING`/`TURN_TO_POINT` | N/A — prints, returns immediately, no wait. | Untouched. | Instant. | Silent no-op if misused. |
| `pid_wait_until(angle)` | `DRIVE`/odom | N/A — prints, returns immediately. | Untouched. | Instant. | Same. |
| `pid_wait_quick()` / `pid_wait_quick_chain()` | any | Dispatches into the matching row above/below (§4) against the chain target. | Inherits. | Inherits. | Chain mutation is mutex-atomic against `ez_auto_task` (that task holds `drive_mutex` for its whole pass). |

**JC-1 proposed default**: add a `StuckWatch`-equivalent progress backstop here, matching
odom. This is the most-used wait family and currently the least protected — I'd lean toward
"yes, add it" as the default, but flagging as the single highest-leverage call in this
whole document rather than assuming.

---

## 4. Wait × mode: odom family

| Wait | Mode | Must return | `interfered` | Bound at defaults | Gap |
|---|---|---|---|---|---|
| `pid_wait()` | `PURE_PURSUIT` (incl. boomerang) | Pre-last-point: `pp_index` reaches last point, or `mA_EXIT`/`StuckWatch` on `xyPID`. Post: both `xyPID`+`current_a_odomPID` exit (small/big/mA; velocity filtered) or `StuckWatch`. Loop requires **both** non-`RUNNING`. | `stalled` or `mA_EXIT` → `true`. | Settle: ~90-100 ms per PID. `StuckWatch` window: 500 ms (or 750 via mA fallback) from the last real step of progress — but "real step" and "last progress" are formula-driven, not a flat bound (see N5 below). | **JC-5 (confirmed real):** heading exit latched at an earlier point (line 257) is reused unchanged at the final point (line 275) without re-checking there — xy's per-point exits are explicitly not kept (per the code's own comment); angle's is. **JC-2/N1, ACCEPTED (2026-09-24, see §8.2)**: both `xyPID`'s AND `current_a_odomPID`'s own small/big exits are carrot-relative during boomerang, not just xy's as previously stated — the "needs both non-`RUNNING`" loop condition is not the real-target backstop it was thought to be. Accepted as out-of-scope; boomerang is being replaced by issue #449. **Do not re-report.** |
| `pid_wait()` | `POINT_TO_POINT` | Same as above, minus the pre-last-point loop (no boomerang carrot here — `odom_target` **is** the real target the whole time, confirmed `is_boomerang=false` hardcoded). | Same. | Same. | No carrot ambiguity in this mode at all — JC-2/N1 is `PURE_PURSUIT`/boomerang-only. |
| `pid_wait_until(distance)` | `POINT_TO_POINT`/`PURE_PURSUIT` | **Where N2/N3 actually live.** Routes through `wait_until_drive()`. Pre-last-point: only `leftPID`/`rightPID`'s own exits, which are retargeted every pass to exactly one look-ahead past the motion's start (confirmed, §1) — not the true target, so their small/big windows can fire well before the robot is actually there (**N3**). On the last point: `on_last_point` reads `xyPID.exit_condition()` **raw** (not `without_velocity`-filtered) — a pivot reading as "stopped" translationally ends this wait where plain `pid_wait()` on the same motion wouldn't. **No `StuckWatch` anywhere in this function** — a spin here has zero odom-side backstop (**N2**'s literal shape). JC-4's doubling also applies (same function). | mA/velocity failsafe → `true`; left/right's own pre-last-point exits → `true` on velocity/mA. | Pre-last-point: bounded by left/right's own small/big/velocity/mA against the look-ahead target — could be fast (N3) or, for velocity/mA, ~2× via JC-4. Last point, spun: **unbounded**, no backstop (N2). | Both N2 and N3 are structurally confirmed here, not just claimed by the task prompt. Magnitude unverified — Step 3. |
| `pid_wait_until_point(target)` / `pid_wait_until(pose)` | any mode, **no guard** | Crossed (`is_past_target` sign flip, live odometry), or failsafe/`StuckWatch` on `xyPID`/`current_a_odomPID`. | Failsafe per type; `StuckWatch` unconditional. | Same `StuckWatch` shape as `pid_wait()`. | **Confirmed, no guard at all** — unlike every `wait_until_*`. Runs fine on the crossed-check (live odometry) even outside odom modes, but the exit/`StuckWatch` machinery reads stale `xyPID`/`current_a_odomPID` state left over from whatever the last odom motion was. **JC-6.** |
| `pid_wait_until_index_started(index)` / `pid_wait_until_index(index)` | intended for `PURE_PURSUIT` | Same shape as above. | Same. | Same. | **Confirmed this pass**: `injected_pp_index` is only populated by the pure-pursuit path-injection setter — `pid_odom_ptp_set()` (`POINT_TO_POINT`) never touches it. Calling this in `POINT_TO_POINT` mode reads a stale/leftover array from whatever pure-pursuit motion last ran (or an empty one) — same missing-guard family as the row above. **JC-6 covers this too.** |
| `pid_wait_quick()` / `pid_wait_quick_chain()` | any odom | Dispatches into the rows above against the extended target. | Inherits. | Inherits. | Matches. |

**`StuckWatch`'s real shape, stated plainly (this is what N5 is actually about)**: it makes
odom waits *finite*, not *bounded within an auton's time budget*. At default constants
(`small_error` 1 in, `velocity_exit_time` 500 ms → step ≈ 1 in, window = 500 ms), a leg that
crawls at just over step/window = **2 in/s** never triggers it — a 48 in final approach at
that speed survives roughly 24 seconds, which is most of a full autonomous period, for one
motion. That's **N5**: the same number is both "the floor below which a healthy slow motion
gets falsely flagged stuck" and "above which there's effectively no time bound at all."

---

## 5. Mode changes / disable mid-wait

Every wait's own loop polls its PID's `error`/`derivative` every `DELAY_TIME` regardless of
whether `ez_auto_task`'s per-mode function is still updating them. `ez_auto_task()` forces
`drive_mode_set(DISABLE, true)` on a field disable or once when autonomous ends, after which
the per-mode function stops running and `error`/`derivative` freeze while the wait keeps
reading them.

**Corrected from v3, using hardware evidence already on file** (project memory:
`stuck-lock-on-mode-change-hardware-results.md`, confirmed on-brain in both bench and
real-switch testing — "replacement competition task inherits the lock so driver control
works but odometry dies"): a competition state change **does replace the running
competition task** (autonomous → the next state runs as a new task), which means a
`pid_wait()` call sitting inside `autonomous()` itself does not hang — the task it's running
on stops existing. **The frozen-value scenario below only applies to a wait running in a
context that survives the transition — a persistent user-created task, not `autonomous()`
or `opcontrol()` themselves** (both are competition-linked and get the same replacement
treatment, by the same evidence).

With that scoped down: a wait in a persistent user task, mid-motion, whose mode's per-frame
function stops updating (e.g. `drive_mode_set(DISABLE, ...)` runs while that user task's
wait loop is still going) sees the same frozen-error shape as before — a frozen in-tolerance
error can still fire `SMALL_EXIT`/`interfered=false` (false success); a frozen out-of-
tolerance error with nonzero derivative can hang (`VELOCITY_EXIT` never arms-as-stopped,
`mA_EXIT` reads live current which likely isn't there once stopped). Narrower blast radius
than v3 stated, but the shape is real for that narrower case.

One addition: any odom wait's `target_distance()`/`point_distance()` lambdas take
`drive_mutex` every call (`ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex)`) — a
wait holding that lock at the moment its task gets torn down is a concurrency question for
Step 3's angle H, not resolved here.

---

## 6. Findings table

| ID | Gap | Confidence |
|---|---|---|
| F5 | Velocity exit firing internally inside an odom wait still runs `timers_reset()` (wiping mA/current accumulator, disarming `velocity_armed`) even though the caller only sees `RUNNING`. | Confirmed by code read. |
| P8 | `interfered` never clears on a clean wait return, only on the next top-level `pid_*_set()`. Confirmed across all four setter files this pass. | Confirmed. |
| JC-1 / N4 | No progress backstop in DRIVE/TURN/SWING/TURN_TO_POINT. Pinned: bounded (~1.5 s). Spun: unbounded. Motor-encoder lift: false success. Tracking-wheel lift: same bound as pinned (corrected this pass). | Confirmed. |
| JC-4 | `wait_until_drive`/`wait_until_turn_swing_internal` double-delay real-time doubling of every failsafe. `pid_wait()` unaffected. | Confirmed by rereading both loop bodies. |
| JC-5 | Heading exit latched at an earlier pure-pursuit point reused unchanged at the final point. | Confirmed. |
| N2 / N3 | Both live in `pid_wait_until(distance)` on odom modes (§4), not in `pid_wait()`. N3: left/right retarget to one-look-ahead-past-start (confirmed via `raw_pid_odom_ptp_set`'s literal `leftPID.target_set` line). N2: this function has zero `StuckWatch` coverage, unlike `pid_wait()`. | Confirmed structurally; magnitude unverified (Step 3). |
| JC-2 / N1 | **ACCEPTED (2026-09-24, see §8.2).** Both `xyPID` and `current_a_odomPID`'s own exits are carrot-relative during boomerang, in `pid_wait()` and in `pid_wait_until_index_started()` on non-final legs — the "needs both non-`RUNNING`" condition is not a real-target backstop. Out of scope; boomerang is being replaced by issue #449. | Confirmed (Round 3), corrects this doc's earlier narrower claim. |
| JC-6 | `pid_wait_until_point`/`pid_wait_until(pose)` has no mode guard at all. `pid_wait_until_index[_started]` in `POINT_TO_POINT` reads a stale/never-populated `injected_pp_index`. | Confirmed. |
| §5 disable-mid-wait | Real, but scoped to persistent user tasks only — `autonomous()`/`opcontrol()` themselves get replaced on a state change per existing hardware evidence, not hung. | Confirmed via cited prior hardware testing, not re-tested this pass. |
| N5 | Not a simple floor — the same ~2 in/s number at default constants is both the false-stuck floor and the point past which `StuckWatch` stops being a meaningful time bound. | Confirmed via formula (step/window), worked example given. |
| `StuckWatch` step = 0 | Only reachable with `small_error==0` AND `velocity_exit_time==0` AND `mA_timeout!=0`. Only strictly-smaller readings count as progress (not every reading). | Confirmed. |
| N6, slow-hairpin `velocity_zero_main` | Runtime/data claims. | Needs sim/hardware — Step 3. |

---

## 7. Judgment calls, each with a proposed default

1. **JC-1**: add a `StuckWatch`-equivalent to DRIVE/TURN/SWING/TURN_TO_POINT? **Proposed: yes** — highest-leverage gap, most-used wait family, currently least protected.
2. **JC-2 / N1** (boomerang carrot vs. real target): **DECIDED 2026-09-24 — see §8.2.** Both axes' carrot-relative exits are accepted and out of scope; boomerang is being replaced by issue #449.
3. **JC-3** (disable-mid-wait): scoped to persistent user tasks only per §5. **Proposed: low priority** given the narrow blast radius — worth a note in docs ("don't run a blocking wait for a motion inside a task that outlives the competition state") rather than a code change, unless you disagree with the narrowing.
4. **JC-4** (double-delay in the two `wait_until_*` functions): **Proposed: fix it** — drop the redundant `pros::delay()` so failsafes fire at their configured value, not ~2×. Looks like an unintentional bug, not a design choice.
5. **JC-5** (latched angle exit carrying to the final point): **Proposed: reset it on `pp_index` advance**, matching what the code comment already claims is the design intent for xy. Symmetric with how xy is explicitly handled.
6. **JC-6** (`pid_wait_until_point`/`pid_wait_until_index[_started]` missing mode guards): **Proposed: add the same mode-check-and-refuse pattern** the other `wait_until_*` functions already have, for consistency and to fail loud instead of silently reading stale state.
7. **BIG_EXIT as success**: today `BIG_EXIT` leaves `interfered=false`, same as `SMALL_EXIT`, and `pid_wait()`'s odom "settled" check relies on this. **Proposed: keep it** — changing this would ripple into the settled check and likely surprise existing autons; flagging only because the task's own N6 depends on this convention.
8. **N5 floor**: **No default proposed** — this is a real tuning tradeoff (lower floor = more false "stuck" flags on genuinely slow-but-healthy motions; leaving it = the 24 s-survives-a-crawl shape). Needs sim evidence on real archetypes before picking a number, per the task's Step 1-2.
9. **P8** (`interfered` auto-clear): **Proposed: leave as-is** (only the next `pid_*_set()` clears it) but document it explicitly — auto-clearing on every wait return would silently erase a signal that a prior `pid_wait_until()` in the same motion had a problem, which seems like the wrong direction.
10. **Zeroed exit constants / hard ceiling**: **Proposed: no new hard ceiling** — "the team's choice to make" matches EZ-Template's existing philosophy of exposing raw constants rather than guarding them, but flagging since a hard ceiling would still fit the "nothing gated" constraint if you'd rather have one.
11. **Scope inventory method** (§2): no sign-off needed, just noting the method for the next step.

Jess's sign-off: proceed with proposed defaults on all of them.

---

## 8. Accepted behaviors from Round 2/3 (2026-09-24)

By this point the audit is on its third fresh Step 5 attack round, against a branch that has
absorbed two rounds of Step 4 fixes. §1-§7 above are the original Step 0 spec and are left
historically intact except for the two corrections above; this section is where later,
post-signoff decisions live. **Attack agents: everything in this section is decided. Do not
report it again.**

### 8.1 A pivot counts as distance driven

`wait_until_drive()` on an odom move (`POINT_TO_POINT`/`PURE_PURSUIT`) ends `pid_wait_until
(distance)` successfully the moment **either** side's driven distance crosses the requested
value — including a pure in-place pivot with zero net translation, where one side's encoder
travel is entirely pivot arc length. This is intentional, not a bug: Jess's own framing for
this wait was "I just want to know when the left or right side has driven the wait until
distance," and that is a literal, either-side, per-side reading with no net-translation
requirement layered on top. **Do not change this. Do not report it as a regression** (an
earlier internal framing of this behavior as "finding #2c, a regression" was wrong and is
retracted).

### 8.2 Boomerang's carrot-relative exits are accepted, out of scope

During a boomerang leg, `raw_pid_odom_ptp_set()` retargets **both** `xyPID` (via `odom_target`)
**and** `current_a_odomPID` (via `point_to_face`) to the moving carrot, every pass, not just
xy as §1/§4/§6/§7 originally claimed. Concretely, in `pid_wait()`'s `PURE_PURSUIT` last-point
loop and in `pid_wait_until_index_started()` on a non-final leg, both axes' own small/big exit
windows are measured against the carrot, not the real waypoint — the "settled"/`StuckWatch`
check against the real target only runs on the stuck-detection path, not on an ordinary clean
double-exit. A robot can end a boomerang leg with `interfered=false` noticeably short of the
real target (confirmed empirically: ~7.2 of a 12in leg on a plain, healthy, never-paused
cruise at shipped default constants and the shipped default `dlead` of 0.625).

**This is accepted and out of scope.** Boomerang is being replaced by issue #449. Do not fix
this mechanism, do not report it again in any future attack round.

### 8.3 Channel's one-shot rebound latch stays as it is

`exit_conditions.cpp`'s `Channel::made()` (the shared progress test behind `StuckWatch` and
`SingleStuckWatch`) only ever credits a recovered overshoot/shove **once** for the life of a
Channel instance — `rebounded` arms on the first sign-flip and never resets. This was already
decided once (Round 2) and stands after being reconfirmed and sharpened in Round 3: a
**second** recovered disturbance within the same wait (e.g. two ordinary taps from a
defender, or a second encoder-noise spike after an earlier one) can be misread as "no
progress" and false-stop the wait — `interfered=true` while the robot was genuinely still
closing on the target. Reproduces on the most-used wait family (DRIVE, and by code-trace
TURN/SWING/odom) at shipped default constants.

**Do not change the latch.** A verifier proved by mutation that naively removing it reopens
the original unbounded spin/pin hang the latch exists to prevent — any real fix needs smarter
re-arming than a blanket removal, and that redesign is not being taken on right now. The known,
accepted consequence — a second recovered disturbance in one wait can end that wait early via
a false "stuck" — is intentional for now. **Do keep** any test that only adds coverage without
changing behavior (e.g. `Channel`'s `step==0` boundary case) — that's welcome, just don't touch
the rebound-latch mechanism itself.

The velocity-exit debounce question (a repeated/flaky sensor sample defeating a *different*
backstop, not `Channel`) is a separate, real bug — see the Step 4 round-3 fix pass, not this
section.

### 8.4 Process rules for every future attack/verify/fix round

**Never kill processes by name.** No `Stop-Process -Name`, no `taskkill /IM`, no `pkill`.
Parallel agents share this machine — killing by name has already taken out sibling agents'
in-progress builds mid-round. Only ever kill a PID your own worktree/build actually started.

**Never use `git stash` when other agents may be running in sibling worktrees of the same
repo.** `.git/stash` is shared across every worktree of a repo, not per-worktree — concurrent
agents stashing at the same time have already collided, with one agent's `stash pop` pulling
back a *different* agent's unrelated WIP (caught and recovered by luck, not by design, in the
Step 4 round-3 fix pass). Use a throwaway branch or a patch file (`git diff > x.patch`) to set
work aside instead.

### 8.5 Redundant-IMU primary handoff has no resync on promotion, and stays that way

`Drive::check_imu_task()` in `maintenance.cpp`: on ejection, the promotion path (where the
next `good_imus` entry becomes primary) does **not** resync against the outgoing IMU's last
reading, unlike a *returning* IMU on the recovery path, which does get resynced. **This is a
deliberate decision, not an oversight: do not change the code.**

The known consequence, stated explicitly: two IMUs that are each individually "good" the
whole time (neither frozen, neither ejected) can still carry an ordinary standing calibration
offset between them — ordinary inter-sensor drift, not a fault. The instant one is ejected for
any reason and the other promoted, `drive_angle_get()`/`odom_theta_get()` jumps by that offset
in one tick with no transition. If that jump lands inside a turn's exit window, it produces a
clean `interfered=false` exit while the robot's real heading is still outside `big_error`.
Confirmed in Round 4 at real `TURN` defaults (3°/7°, not the 1°/3° an earlier pass mis-cited):
a 12° offset — well outside the real 7° `big_error` window — still produced a clean
`SMALL_EXIT` the instant the promoted IMU took over. Confirmed by direct repro for `TURN`;
`TURN_TO_POINT`/`POINT_TO_POINT`-odom/`SWING` by code trace, same `drive_angle_get()`/
`odom_theta_get()` mechanism.

**Attack agents: this is decided. Do not re-report it.**

### 8.6 Velocity exit is disabled for Drive's own drive, turn, and swing waits

`pid_wait()` in `DRIVE`, `TURN`, `TURN_TO_POINT`, and `SWING` gets the same `without_velocity()`
treatment odom's `pid_wait()` already has, so a `VELOCITY_EXIT` can no longer by itself end any
of these waits — matching the fix odom already received rather than leaving Drive's own waits
on the older, unfiltered behavior. This closes the shipped-default gap Round 4 confirmed at
`exit_conditions.cpp:362` (a realistic 200rpm/2.75in drivetrain at speed 20 cruising at
~2.9-3.4in/s — under the fixed 5in/s velocity floor — ended a DRIVE `pid_wait()` after ~5in of
a 60in leg, `interfered=true`, and survived realistic sensor noise). **Supersedes §3's stated
~1500ms pinned bound for `pid_wait()` in these modes**, which described the raw,
`VELOCITY_EXIT`-terminated behavior this entry replaces — §3's table text is left as originally
written per this document's own historical-intact convention (see the note at the top of §8);
this entry is the correction.

`StuckWatch`/`SingleStuckWatch` remain the stall backstop for all of these waits exactly as
before — they track PID *error*, not velocity/derivative, so removing the velocity channel's
ability to end these waits does not remove their only backstop. A permanently pinned or spun
robot is still released by the stuck watch, not by velocity.

**Attack agents: this is decided. Do not re-report "drive/turn/swing waits ignore velocity" as
a missing exit check.**

### 8.7 `wait_until_*()`'s settled-inside-big-error exemption, extended and conditioned

`pid_wait()` already treats a stall where the robot has settled inside its `big_error` window
as "settled", not "stuck" (§3, §4's settled carve-outs). That same exemption now also applies
to `wait_until_drive()` and `wait_until_turn_swing_internal()` — but **only** when the
`wait_until()` target passed in is numerically equal (within the library's existing exit
tolerance) to the motion's final target.

Both halves apply explicitly:

- A `wait_until()` target equal to the final target gets the exemption: stopping inside
  `big_error` short of that target is an ordinary settle, not a stuck condition, and does not
  set `interfered=true` on its own.
- A `wait_until()` target that is a waypoint short of the final target does **not** get the
  exemption: stopping short of a point the robot is meant to drive *through* on its way to
  somewhere further must still report `interfered=true`. Extending the settled carve-out to a
  mid-route waypoint would silently let the robot stop early on the way to its actual
  destination, which is a real early exit, not a settle.

This closes the asymmetry Round 4 confirmed at `exit_conditions.cpp:675/756/786` — the fix
commit that added the exemption to `pid_wait()` (`3198d2f`) had deliberately scoped it there
only; extending it to the two `wait_until_*` functions, gated on target-equals-final, is the
consistency decision that commit left open.

**Attack agents: this is decided. Do not re-report the settled-exemption asymmetry between
`pid_wait()` and the `wait_until_*` functions.**
