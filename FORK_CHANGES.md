# Turn-Motion Klipper fork: change ledger and update guide

This repository is upstream Klipper (https://github.com/Klipper3d/klipper)
plus a small set of changes for the Turn V2 machine: a cartesian printer
whose bed is carried by three independent vertical motors (Z, A, B).

This file is the single place that records **what** the fork changes,
**why**, and **how to re-verify it** after pulling a new upstream
version. Keep it up to date whenever a fork change is added, removed,
or moved. It is written for both humans and coding agents.

## 1. Branch and update strategy

- `upstream` remote: https://github.com/Klipper3d/klipper.git
- The fork lives as a short stack of commits on top of `upstream/master`.
  Never merge upstream into the fork; rebase the fork onto upstream so the
  stack stays small and reviewable.
- Update procedure:
  1. `git fetch upstream`
  2. `git rebase upstream/master` on the fork branch; resolve conflicts
     using the rules in section 3.
  3. Run the "new upstream code" checklist in section 6.
  4. Run the verification in section 7.
  5. Verify on the machine (homing, probing, z_tilt, a short print).
- The pre-port state of the old fork is tagged `pre-upstream-port-2026-09`.

## 2. Axis layout (the core invariant)

Upstream Klipper plans three kinematic axes (x, y, z) followed by the
extruder and any "extra axes". This fork plans **five** kinematic axes:

```
index:  0  1  2  3  4  5   6...
axis:   x  y  z  a  b  e   extra axes (manual_stepper GCODE_AXIS, ...)
```

A and B are the two additional bed motors. They are first-class kinematic
axes: planned in the same trapezoid as XYZ, part of the move distance,
homed with endstops through the normal homing module, and stepped by the
itersolve C code from the same trapq. They are deliberately **not**
upstream "extra axes" (those cannot be homed through `homing.py`, are
excluded from the move distance, and force a full stop around pure-tilt
moves).

Consequences that appear all over the code, and that every upstream
update must be checked against:

| Upstream pattern                          | Fork equivalent                              |
|-------------------------------------------|----------------------------------------------|
| `axes_d[:3]`, `pos[:3]`, `pos[3:]`        | `[:5]`, `[5:]`                               |
| `range(3)`, `(0, 1, 2)`, `[0, 1, 2]`      | `range(5)`, `[0, 1, 2, 3, 4]`                |
| `"xyz"`, `'XYZ'`, `'XYZE'`                | `"xyzab"`, `'XYZAB'`, `'XYZABE'`             |
| extruder index `3`, `e_index + 3`         | `5`, `e_index + 5`                           |
| `[0., 0., 0., 0.]` positions              | `[0., 0., 0., 0., 0., 0.]`                   |
| `trapq_append(..., x, y, z, rx, ry, rz,)` | `(..., x, y, z, a, b, rx, ry, rz, ra, rb,)`  |
| `trapq_set_position(tq, t, x, y, z)`      | `(tq, t, x, y, z, a, b)`                     |
| `itersolve_set_position(sk, x, y, z)`     | `(sk, x, y, z, a, b)`                        |
| `itersolve_calc_position_from_coord(...)` | five coordinates                             |
| `gcode.Coord` with x, y, z, e             | x, y, z, a, b, e (pads to 6)                 |

Stock kinematics (cartesian, corexy, delta, ...) still hand three- or
four-element lists to the core. The core tolerates that:
`stepper.pad_coord`, `toolhead.set_position`, `homing.home_rails` and
`HomingMove.calc_toolhead_pos` pad short lists, and `G28` homes only the
axes named by `kin.axis_names` (default `"xyz"`).

## 3. Conflict resolution rules

- In a conflict inside a file listed in section 4, keep upstream's
  structure and re-apply the fork's semantic change (usually one of the
  substitutions in the table above). Do not resurrect old fork code.
- If upstream adds a new place that indexes positions with `3`, `[:3]`,
  `"xyz"` or similar, decide whether it refers to the kinematic axes
  (then convert) or to something else (then leave it). Section 6 has the
  grep that finds these.
- If upstream changes a C signature that carries coordinates
  (`trapq_append`, `trapq_set_position`, `itersolve_*`), update
  `klippy/chelper/__init__.py` and **every** Python caller.
  `grep -rn "trapq_append(\|itersolve_set_position(\|calc_position_from_coord(" klippy`
  lists them.

## 4. Files changed relative to upstream

Grouped by the commits of the original port. The rationale column is the
thing to preserve; the exact lines may move as upstream evolves.

### 4.1 C helper (`klippy/chelper/`)

| File | Change | Why |
|------|--------|-----|
| `trapq.h`, `trapq.c` | `struct coord` has `x, y, z, a, b` (`axis[5]`); `move_get_coord`, `trapq_append`, `trapq_set_position`, `pull_move`/`copy_pull_move` carry a and b | A and B travel in the same trapq as XYZ |
| `itersolve.h`, `itersolve.c` | `AF_A`, `AF_B` active flags; `check_active`, `itersolve_is_active_axis('a'/'b')`, `itersolve_calc_position_from_coord`, `itersolve_set_position` take a and b | Steppers on A/B are recognised and stepped |
| `kin_idex.c` | copies `a`/`b` into the transformed move | keep the dual-carriage wrapper coordinate-complete |
| `kin_cartesianxy3z.c` | **fork-only** | one-coordinate stepper kinematics for x, y, z, a, b |
| `__init__.py` | CFFI declarations for all of the above; `kin_cartesianxy3z.c` in `SOURCE_FILES` | |
| `klippy/stepper.py` | `pad_coord`; `set_position`/`calc_position_from_coord` pass five coords | stock kinematics pass three |

### 4.2 Motion core and g-code

| File | Change | Why |
|------|--------|-----|
| `klippy/toolhead.py` | six-element `commanded_pos`; `Move` distance = `max(xy, |z|, |a|, |b|)`; junction cosine includes a, b; `e_index + 5`; `trapq_append` with five coords; `set_position` pads and passes a, b; `move()` flushes lookahead when `check_move` rejects a move; `drip_move` keeps A/B targets; `max_z_velocity`/`max_z_accel` on `[printer]` with `get_max_z_velocity()`, `set_max_velocities(..., max_z_velocity, max_z_accel)`, status fields, `SET_VELOCITY_LIMIT Z_VELOCITY= Z_ACCEL=` | first-class A/B; shared vertical speed limits |
| `klippy/gcode.py` | `Coord` pads to 6 and exposes `.a`, `.b`, `.e` (index 5) | |
| `klippy/extras/gcode_move.py` | six-element positions; `axis_map` with A, B; E at index 5 in G92, M114, M221, SET_GCODE_OFFSET, SAVE/RESTORE_GCODE_STATE, GET_POSITION | G1 A.. B.. and offsets on A/B |
| `klippy/extras/homing.py` | five-axis distances and `"xyzab"`; pads short positions; `G28` uses `kin.axis_names`; `_probing_home` applies the probed Z correction to A and B; `probing_move` calls `kin.set_probing(True/False)` around the move | homing with A/B; safety checks off while probing |
| `klippy/kinematics/extruder.py` | five-coord `set_position` and `trapq_append`; **maximum extrusion cross-section check is not enforced** (only logged) | move distance includes bed travel, so the ratio is not meaningful on this machine |
| `klippy/kinematics/none.py` | `calc_position` returns five zeros | |
| `klippy/kinematics/rotary_delta.py` | five coords to `itersolve_calc_position_from_coord` | |
| `klippy/extras/manual_stepper.py`, `force_move.py` | five-coord trapq/itersolve calls; `SET_KINEMATIC_POSITION A= B=`, `SET_HOMED`/`CLEAR_HOMED` accept a, b | |
| `klippy/extras/motion_report.py` | trapq dumps and live position include a, b; extruder velocity at index 5 | |
| `klippy/extras/exclude_object.py`, `tuning_tower.py`, `hall_filament_width_sensor.py`, `tsl1401cl_filament_width_sensor.py` | E index 5, six-element positions | |
| `klippy/extras/gcode_arcs.py` | A and B interpolated linearly along G2/G3 | |
| `klippy/extras/homing_override.py` | `set_position_a/b`, axes XYZAB | |
| `klippy/extras/stepper_enable.py` | `clear_homing_state("xyzab")` on motor off | |
| `klippy/extras/safe_z_home.py` | Z hop and Z homing move Z, A, B together | |
| `klippy/extras/display/display.cfg` | position line shows A and B | |

### 4.3 Kinematics (fork-only files)

| File | Purpose |
|------|---------|
| `klippy/kinematics/cartesianxy3z.py` | `kinematics: cartesianxy3z`. Rails `stepper_x/y/z/a/b` (+ `dual_carriage`), `axis_names = "xyzab"`. Homing: coordinated fast approach of Z/A/B (`HomingMove` on all three endstops, back off 2 mm) then individual homing per axis; X/Y after the vertical axes. `check_move` limits speed/accel so the slowest axis group (XY at `max_velocity`, vertical at `max_z_velocity`) sets the move duration. Calls the safety module except while homing/probing (`is_homing`, `set_probing`). |
| `klippy/kinematics/cartesianxy3z_safety.py` | Bed joint geometry checks (`[CartesianXY3ZSafety]` section, `CHECK_CARTESIANXY3Z_SAFETY`, `PREVIEW_MOVE`). Pure fork code; no upstream dependency beyond `Move` attributes and `gcode_move` status. |
| `config/example-Turn-V2.cfg`, `config/example-cartesianxy3z-full.cfg`, `printer.cfg` | machine configs |

### 4.3b Derived horizontal carriages (`kinematics: cartesianxy3z2h`)

| File | Purpose |
|------|---------|
| `klippy/kinematics/cartesianxy3z2h.py` | Subclass of `CartesianXY3Z`. Adds `[stepper_u]`/`[stepper_v]`: steppers whose position is **derived** from (Z, A, B) by plate rigidity, not G-code axes (the position tuple is unchanged). Reads `[cartesianxy3z2h]` (`a_joint_home_x`, `b_joint_home_x`, `max_h_velocity`, `max_h_accel`, `enable_horizontal_carriages`). `check_move` samples U/V along the move (`UV_SAMPLE_MM`, `UV_MAX_SAMPLES`), rejects moves that take a carriage past its required `position_min`/`position_max`, and caps speed/accel from the peak carriage rate. `home()` detaches U/V from the trapq (homing start positions are physically impossible poses) and re-derives them afterwards. `get_derived_steppers()` tells probe/z_tilt to ignore them. |
| `klippy/chelper/kin_cartesianxy3z2h.c` | `cartesianxy3z2h_h_stepper_alloc(a_home_x, b_home_x, side)`: the closed-form slide position; active on `AF_Z\|AF_A\|AF_B`. |
| `klippy/extras/probe.py` `lookup_bed_steppers()` | Z/A/B-active steppers minus `kin.get_derived_steppers()`; used by `LookupZSteppers` and `z_tilt.ZAdjustHelper`. |
| `test/test_uv_derived.py` | C solver (and its Python transcription) vs the vendored reference solver `test/reference/printerkin` (byte-identical copy of the main application's kinematics; excluded from the whitespace check). |
| `test/test_uv_motion.py` | `check_move` behaviour with the C bindings stubbed: limiter, range rejection, interior extremum, homing bypass. Pure Python, runs on any machine. |
| `sim/printer.sim.uvderived.cfg`, `gcode/bed_pose_smoke.gcode`, `sim/verify-axes.sh` | simulator profile, smoke g-code (X Y Z A B only, runs on both profiles), and the container-side gate that builds, runs both unit tests with the real C solver and batch-runs both profiles. `sim/printer.sim.cfg` carries a `[z_tilt]` section as a runtime assertion that the derived steppers stay out of the bed-stepper set. |
| `config/example-cartesianxy3z2h.cfg` | example fragment |

### 4.4 Probing and bed levelling

| File | Change | Why |
|------|--------|-----|
| `klippy/extras/probe.py` | `bed_lift_coord()` / `bed_descend_position()` move A and B by the same **delta** as Z (used by PROBE_CALIBRATE, PROBE_ACCURACY, sample retract, `ProbePointsHelper._raise_tool`, `DescendToEndstopHelper`); `LookupZSteppers` also returns A and B steppers; `probe_travel_limit` option (default `max(-1.5, position_min)`); `[:5]` comparisons in `ProbeEndstopWrapper` | the three motors carry the bed together; a z_tilt correction between them must survive probe lifts; the probe trigger must halt all three |
| `klippy/extras/load_cell_probe.py` | uses `probe.bed_descend_position` and `probe_travel_limit`; first-home correction and ascent lift applied to A and B | same |
| `klippy/extras/z_tilt.py` | `ZAdjustHelper` collects z, a, b steppers; when steppers map to different axes it uses `_adjust_axes`: one coordinated move (mean-relative corrections) and session g-code offsets so Z, A, B report the same coordinate. Stock multi-Z path kept for other kinematics | per-axis levelling of the bed |
| `klippy/extras/bed_mesh.py`, `bed_tilt.py`, `z_thermal_adjust.py` | Z adjustments applied to A and B as well (`bed_tilt` has `a_adjust`/`b_adjust`) | |

### 4.5 Print recovery and estimation

| File | Change |
|------|--------|
| `klippy/extras/virtual_sdcard.py` | tracks the last SD line whose moves reached the step generator (`last_flushed_line_num`, `last_flushed_line` in status; logged on shutdown), reports the resume point on M26 and print start |
| `klippy/extras/print_estimator.py` (fork-only) | API endpoint `estimate_file`: runs a batch klippy with `[move_capture]` to estimate print time and per-move trapezoids |
| `klippy/extras/move_capture.py` (fork-only) | wraps `toolhead.lookahead.flush` and writes one JSON line per move. **Depends on the private `LookAheadQueue.flush` API; re-check after every upstream toolhead change.** |

### 4.6 Simulator and firmware

| File | Change |
|------|--------|
| `Dockerfile.sim`, `docker-compose.sim.yml`, `sim/` | Linux host MCU simulator (see `sim/entrypoint.sh`) |
| `src/linux/gpio_sim.c` (fork-only) | simulated GPIO, swapped in for `src/linux/gpio.c` by the sim entrypoint only |
| `src/Kconfig`, `src/linux/Kconfig`, `src/sched.c`, `src/stepper.c` | `STEPPER_PAST_THRESHOLD_US` / `SCHED_TIMER_PAST_THRESHOLD_US` (1000/0 us on real MCUs, 500 ms on the Linux process) replace the hard-coded "too far in past" limits |
| `.gitattributes`, `.gitignore` | LF for scripts; ignore IDE folders |

## 5. Intentional behaviour differences from upstream

- Move distance is the largest of the XY distance and each vertical
  axis distance, not the XYZ Euclidean distance.
- The extrusion cross-section check (`max_extrude_cross_section`) is
  not enforced. G-code files must declare `M83`/`M82` explicitly; the
  default is upstream's (absolute, M82).
- Probe descents stop at `probe_travel_limit` (default -1.5, never
  below `position_min`).
- After `Z_TILT_ADJUST` the Z, A and B motors sit at different physical
  heights and g-code offsets make them report the same coordinate. The
  offsets are session-only and reset on the next G28.
- The Linux host MCU tolerates 500 ms scheduling slips (simulation only).

Things that were in the old fork and are **deliberately not** here:
the pre-merge load cell probe snapshot and its MCU protocol, the
"adaptive horizontal_move_z" lift, `PROBE_ACCURACY` average delta, the
`bypass_z_home_together` / `post_homing_z_lift` options, per-axis homing
that swallowed errors, and all debug logging.

## 6. Checklist after rebasing onto a new upstream

1. Grep for new three-axis assumptions in Python and decide per hit:
   ```
   grep -rnE "\[:3\]|\[3:\]|range\(3\)|'xyz'|\"xyz\"|'XYZ'|'XYZE'|\(0, 1, 2\)|\[0, 1, 2\]|axes_d\[3\]|axes_r\[3\]|pos\[3\]|e_index \+ 3" klippy --include=*.py
   ```
   Known harmless hits: `resonance_tester.py` (moves XYZ and keeps the
   rest), `idex_modes.py`, `generic_cartesian.py`, `kinematic_stepper.py`,
   `bltouch.py` retry loops, `probe_eddy_current.py` matrix maths,
   `temperature_probe.py`, `garbage_collection.py`, `console.py`.
2. Grep C code for new users of `struct coord` / `axes_r.z` that should
   also handle `.a`/`.b`: `grep -rn "axes_r\.z\|start_pos\.z" klippy/chelper`.
3. Check the private APIs the fork hooks: `LookAheadQueue.flush`
   (move_capture), `toolhead.register_lookahead_callback`
   (virtual_sdcard), `gcode_move.base_position/homing_position` (z_tilt),
   `kin.set_probing` (homing), `probe.LookupZSteppers`,
   `MCU_stepper.set_trapq(None)` (cartesianxy3z2h homing detach).
4. Run `./scripts/check_whitespace.sh`.
5. Run section 7.

## 7. Verification

All of this runs in Docker (no toolchain needed on the host). From the
repository root:

```
docker run --rm -v "$PWD:/klipper:ro" -v "$PWD/../klipper-verify:/out" python:3.11-slim \
    bash -c "tr -d '\r' < /klipper/sim/run-regression.sh > /tmp/r.sh && bash /tmp/r.sh /klipper /out"
```

`sim/run-regression.sh` installs gcc and gcc-avr, copies the tree,
builds the AVR and Linux MCU dictionaries, builds the C helper, and runs
every upstream regression test whose dictionary is available (34 tests).
Expected result: `All 34 test cases passed`.

For the derived carriages additionally run `python test/test_uv_motion.py`
(anywhere) and, inside the simulator container,
`docker compose -f docker-compose.sim.yml run --rm --entrypoint /klipper/sim/verify-axes.sh klipper-sim`
which also exercises the compiled C solver and batch-runs both simulator
profiles on `gcode/bed_pose_smoke.gcode`.

For a motion-level comparison against a previous version, run both
trees in batch mode against the same g-code with `[move_capture]`
enabled and diff the JSONL output; identical positions and extrusion
per move are expected, and velocity differences should be explainable.
Note that `klippy -i` reads the file in 4096-byte chunks and cannot
decode multi-byte UTF-8 characters split across a chunk; use ASCII-only
g-code for batch runs.

Finally verify on the machine: `G28`, `PROBE_ACCURACY`, `Z_TILT_ADJUST`
followed by a second `Z_TILT_ADJUST` (should converge), and a short
print.
