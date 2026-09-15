# Turn-Motion Klipper fork: change ledger and update guide

This repository is upstream Klipper (https://github.com/Klipper3d/klipper)
plus a small set of changes for the Turn machines: a cartesian printer
whose bed is carried by three independent vertical motors (Z, A, B),
optionally with a rotary axis and with the derived horizontal bed joint
carriages.  One repository, one build, one firmware: which machine a
build behaves like is decided entirely by `printer.cfg`.

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
extruder and any "extra axes".  **This fork does not hard-code a number
of axes at all.**  The kinematics class owns the roster and the core
reads it:

```
index:  0  1  2  ...  K-1   K    K+1...
axis:   <the kinematics' axis letters>   e    extra axes
```

- `kin.axis_names` is the roster, one lower-case letter per kinematic
  axis, in order.  A kinematics that declares nothing gets upstream's
  `"xyz"`, so every stock kinematics runs with exactly three axes and
  upstream's behaviour.
- The extruder is at index `K = len(axis_names)`; any "extra axes"
  (`manual_stepper GCODE_AXIS`) follow it.
- `toolhead.get_axis_names()`, `get_axis_count()` and
  `get_extruder_index()` are the only supported way to ask.  Never write
  a literal axis index.
- `klippy/axis_roster.py` answers the machine-shaped questions: which
  axes and which steppers carry the bed, and which steppers are derived
  rather than commanded.  Probing and bed levelling go through it.
- The compiled C code carries `KIN_AXES` (8) axes in `struct coord` as
  an array; `trapq_get_kin_axes()` reports that maximum and the toolhead
  refuses a roster larger than it.

The Turn machines therefore have:

| kinematics       | roster    | extruder index |
|------------------|-----------|----------------|
| `cartesianxy3z`  | `xyzab`   | 5              |
| `cartesianxy3zr` | `xyzabr`  | 6              |
| stock (cartesian, corexy, delta, ...) | `xyz` | 3 |

### Move distance

Upstream measures a move by the euclidean distance over x, y and z.  The
kinematics may instead publish `distance_groups`, a list of axis-index
tuples.  Each group contributes the euclidean norm of its axes and the
move distance is the **largest** group, so axes in different groups each
get the whole move time rather than sharing it.

- One group over every axis reproduces upstream exactly, and that is the
  default for a kinematics that publishes nothing.
- The Turn machines use `[(0,1), (2,), (3,), (4,)]`: the gantry in one
  group and each bed motor in its own.
- An axis in **no** group is carried along by the other axes and never
  makes a move longer; when it is the only axis moving, the largest
  kinematic axis distance is the fallback so that it still moves.  This
  is what `motion_mode: slaved` on a rotary axis does.

Consequences that every upstream update must be checked against:

| Upstream pattern                          | Fork equivalent                              |
|-------------------------------------------|----------------------------------------------|
| `axes_d[:3]`, `pos[:3]`, `pos[3:]`        | `[:naxes]`, `[naxes:]`                       |
| `range(3)`, `(0, 1, 2)`, `[0, 1, 2]`      | `range(naxes)`                               |
| `"xyz"`, `'XYZ'`, `'XYZE'`                | `toolhead.get_axis_names()` (+ `'E'`)        |
| extruder index `3`, `e_index + 3`         | `toolhead.get_extruder_index()`              |
| `[0., 0., 0., 0.]` positions              | `[0.] * (naxes + 1)`                         |
| "the Z axis carries the bed"              | `axis_roster.get_bed_axes(printer)`          |
| "these are the Z steppers"                | `axis_roster.get_bed_steppers(printer)`      |
| `trapq_append(..., x, y, z, ...)`         | `(..., naxes, start_pos, axes_r, ...)`       |
| `trapq_set_position(tq, t, x, y, z)`      | `(tq, t, naxes, pos)`                        |
| `itersolve_set_position(sk, x, y, z)`     | `(sk, naxes, coord)`                         |
| `itersolve_calc_position_from_coord(...)` | `(sk, naxes, coord)`                         |
| `itersolve_is_active_axis(sk, 'z')`       | axis **index**, not a letter                 |
| `gcode.Coord` with x, y, z, e             | one attribute per roster letter, plus `e`    |

`gcode.Coord` is a single class that is re-shaped in place by
`gcode.set_axis_names()` when the kinematics loads.  It must stay one
class object: config sections are loaded before the toolhead, so modules
capture a reference to it before the roster is known.

Stock kinematics still hand three- or four-element lists to the core.
The core tolerates that: `stepper.pad_coord`, `toolhead.set_position`,
`homing.home_rails` and `HomingMove.calc_toolhead_pos` pad short lists.

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
| `trapq.h`, `trapq.c` | `struct coord` is `double axis[KIN_AXES]` with `x, y, z` kept as names; `trapq_append`, `trapq_set_position` take a count and an array; `pull_move` carries arrays; `trapq_get_kin_axes()` and `trapq_fill_coord()` added | the number of kinematic axes comes from the kinematics at runtime |
| `itersolve.h`, `itersolve.c` | `AF_AXIS(i)` active flags; `check_active` loops; `itersolve_is_active_axis` takes an index; `itersolve_calc_position_from_coord` and `itersolve_set_position` take a count and an array | same |
| `kin_idex.c` | copies every axis into the transformed move | keep the dual-carriage wrapper coordinate-complete |
| `kin_axis.c` | **fork-only** | one pass-through stepper solver for any axis index (replaces `kin_cartesianxy3z.c`) |
| `kin_bedjoint.c` | **fork-only** | the derived bed joint carriage solver |
| `__init__.py` | CFFI declarations for all of the above; `TRAPQ_KIN_AXES` is checked against the compiled `KIN_AXES` at startup | a silent mismatch would corrupt every motion report |
| `klippy/stepper.py` | `KIN_AXIS_NAMES` / `set_axis_names()` / `pad_coord`; `is_active_axis` accepts a letter or an index | stock kinematics pass three coordinates |

### 4.2 Motion core and g-code

| File | Change | Why |
|------|--------|-----|
| `klippy/toolhead.py` | the axis roster and `distance_groups` come from the kinematics (`_setup_axes`); `calc_move_distance()`; junction cosine over the roster; the extruder index is `axis_count`; `trapq_append`/`trapq_set_position` take arrays; `move()` flushes lookahead when `check_move` rejects a move; `max_z_velocity`/`max_z_accel` on `[printer]` with `get_max_z_velocity()`, `set_max_velocities(..., max_z_velocity, max_z_accel)`, status fields, `SET_VELOCITY_LIMIT Z_VELOCITY= Z_ACCEL=` | any axis count; shared vertical speed limits |
| `klippy/axis_roster.py` | **fork-only** helper: axis names/count, extruder index, bed axes, bed steppers, derived steppers, bed-relative position helpers | one place asks the kinematics instead of many literals |
| `klippy/gcode.py` | `Coord` is re-shaped in place by `set_axis_names()`; one attribute per roster letter plus `e` | a reference captured before the toolhead loads must stay valid |
| `klippy/extras/gcode_move.py` | positions, `axis_map` and every axis-naming command (G1, G92, M114, M221, SET_GCODE_OFFSET, SAVE/RESTORE_GCODE_STATE, GET_POSITION) built from the roster | `G1 A.. B.. R..` and offsets on any axis |
| `klippy/extras/homing.py` | roster-driven distances, axis letters and padding; `G28` axes from the roster; `_probing_home` applies the probed Z correction to every bed axis; `probing_move` calls `kin.set_probing(True/False)` around the move | homing with extra axes; safety checks off while probing |
| `klippy/kinematics/extruder.py` | array `set_position` and `trapq_append`; **maximum extrusion cross-section check is not enforced** (only logged) | move distance includes bed travel, so the ratio is not meaningful on this machine |
| `klippy/kinematics/rotary_delta.py`, `none.py` | array-form C calls | |
| `klippy/extras/manual_stepper.py`, `force_move.py` | single-axis private trapqs; `SET_KINEMATIC_POSITION` and `SET_HOMED`/`CLEAR_HOMED` follow the roster | |
| `klippy/extras/motion_report.py` | trapq dumps and live position sized from the roster | |
| `klippy/extras/exclude_object.py`, `tuning_tower.py`, `hall_filament_width_sensor.py`, `tsl1401cl_filament_width_sensor.py` | extruder index from the toolhead | |
| `klippy/extras/gcode_arcs.py` | every kinematic axis outside the arc plane is interpolated linearly along G2/G3; `R` is only rejected as a radius when the machine has no R axis | |
| `klippy/extras/homing_override.py` | `set_position_<axis>` options discovered from the config, mapped through the roster | |
| `klippy/extras/stepper_enable.py` | `clear_homing_state()` over the roster on motor off | |
| `klippy/extras/safe_z_home.py` | the hop and the bed homing use `axis_roster.get_bed_axes` | |
| `klippy/extras/display/display.cfg` | position line shows A and B | |

### 4.3 Kinematics (fork-only files)

One engine, a module per machine flavour.  A flavour declares its axis
roster and nothing else; everything else is shared.

| File | Purpose |
|------|---------|
| `klippy/kinematics/axis_kinematics.py` | The engine.  Builds a rail per axis from an `AxisDef` list (letter, config section, motion group), plus optional `[dual_carriage]` and `[bed_joint_carriages]`.  Motion groups: `PLANAR` (one euclidean group at `max_velocity`/`max_accel`), `VERTICAL` (one group each at `max_z_velocity`/`max_z_accel`, and these are the bed axes), `ROTARY` (own limits from `max_<axis>_velocity`/`max_<axis>_accel` on `[printer]`).  Homing: coordinated fast approach of all the vertical axes (`HomingMove` on their endstops, back off 2 mm) then individual homing per axis, then the planar axes, then anything else with an endstop.  `check_move` range-checks every axis and limits speed and acceleration so the group that needs the most time sets the move duration.  `BareAxisRail` serves an axis configured without an endstop. |
| `klippy/kinematics/cartesianxy3z.py` | `kinematics: cartesianxy3z` - the roster `x y` (planar) + `z a b` (vertical). |
| `klippy/kinematics/cartesianxy3zr.py` | `kinematics: cartesianxy3zr` - the same plus `r` (rotary). |
| `klippy/kinematics/bed_joint_carriages.py` | `[bed_joint_carriages]`: the two horizontal bed joint slides, derived from the bed pose rather than commanded.  `travel_check: sampled|endpoints` and `homing_mode: detached|attached` select between the two implementations we have run. |
| `klippy/kinematics/cartesianxy3z_safety.py` | Bed joint geometry checks (`[CartesianXY3ZSafety]` section, `CHECK_CARTESIANXY3Z_SAFETY`, `PREVIEW_MOVE`).  Pure fork code.  It is specific to the three-vertical-axis bed and indexes z, a and b at 2, 3 and 4, which holds for every flavour that enables it. |
| `config/example-Turn-V2.cfg`, `config/example-cartesianxy3z-full.cfg`, `config/example-cartesianxy3zr.cfg`, `config/example-bed-joint-carriages.cfg`, `printer.cfg` | machine configs |

Adding a flavour is a new module with a roster and a `load_kinematics`
that calls `AxisKinematics`.  Do not subclass an existing flavour.

### 4.4 Probing and bed levelling

| File | Change | Why |
|------|--------|-----|
| `klippy/extras/probe.py` | `bed_lift_coord()` / `bed_descend_position()` move every bed axis by the same **delta** as Z (used by PROBE_CALIBRATE, PROBE_ACCURACY, sample retract, `ProbePointsHelper._raise_tool`, `DescendToEndstopHelper`); `LookupZSteppers` takes `axis_roster.get_bed_steppers`; `probe_travel_limit` option (default `max(-1.5, position_min)`) | the bed motors carry the bed together; a z_tilt correction between them must survive probe lifts; the probe trigger must halt all of them |
| `klippy/extras/load_cell_probe.py` | uses `probe.bed_descend_position`, `probe_travel_limit` and `axis_roster.bed_delta_position` | same |
| `klippy/extras/z_tilt.py` | `ZAdjustHelper` takes its steppers from `axis_roster.get_bed_steppers`; when they map to different axes it uses `_adjust_axes`: one coordinated move (mean-relative corrections) and session g-code offsets so every bed axis reports the same coordinate.  Stock multi-Z path kept for other kinematics | per-axis levelling of the bed |
| `klippy/extras/bed_mesh.py`, `bed_tilt.py`, `z_thermal_adjust.py` | Z adjustments applied to every bed axis (`bed_tilt` reads a `<axis>_adjust` option per extra bed axis) | |

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

- Move distance is the largest distance group, not the XYZ euclidean
  distance.  On the Turn machines that is the largest of the XY distance
  and each vertical axis distance.  A stock kinematics is unaffected.
- The extrusion cross-section check (`max_extrude_cross_section`) is
  not enforced. G-code files must declare `M83`/`M82` explicitly; the
  default is upstream's (absolute, M82).
- Probe descents stop at `probe_travel_limit` (default -1.5, never
  below `position_min`).
- After `Z_TILT_ADJUST` the Z, A and B motors sit at different physical
  heights and g-code offsets make them report the same coordinate. The
  offsets are session-only and reset on the next G28.
- The Linux host MCU tolerates 500 ms scheduling slips (simulation only).
- A rotary axis without an `endstop_pin` is reported as homed from
  startup and is skipped by `G28`.
- The derived bed joint carriages are never homed; their position is
  recomputed from the bed pose, and their configured `position_min` /
  `position_max` are enforced by rejecting the move.

Things that were in the old fork and are **deliberately not** here:
the pre-merge load cell probe snapshot and its MCU protocol, the
"adaptive horizontal_move_z" lift, `PROBE_ACCURACY` average delta, the
`bypass_z_home_together` / `post_homing_z_lift` options, per-axis homing
that swallowed errors, and all debug logging.

## 6. Checklist after rebasing onto a new upstream

1. Grep for new fixed-axis assumptions in Python and decide per hit -
   does it mean the kinematic axes (convert it) or something else
   (leave it)?
   ```
   grep -rnE "\[:3\]|\[3:\]|range\(3\)|'xyz'|\"xyz\"|'XYZ'|'XYZE'|\(0, 1, 2\)|\[0, 1, 2\]|axes_d\[3\]|axes_r\[3\]|pos\[3\]|e_index \+ 3" klippy --include=*.py
   ```
   Known harmless hits: `resonance_tester.py` (moves XYZ and keeps the
   rest), `idex_modes.py`, `generic_cartesian.py`, `kinematic_stepper.py`,
   `bltouch.py` retry loops, `probe_eddy_current.py` matrix maths,
   `temperature_probe.py`, `garbage_collection.py`, `console.py`,
   `hybrid_corexy.py` / `hybrid_corexz.py` (rail indexes, not axes).
2. Grep for new "Z is the bed" assumptions - anything that adjusts or
   looks up `pos[2]` in a probing or levelling path should go through
   `axis_roster`.
3. Grep C code for new users of `struct coord` that name `.x`, `.y` or
   `.z` where they mean "every axis":
   `grep -rn "start_pos\.\|axes_r\." klippy/chelper`.
4. Check the private APIs the fork hooks: `LookAheadQueue.flush`
   (move_capture), `toolhead.register_lookahead_callback`
   (virtual_sdcard), `gcode_move.base_position/homing_position` (z_tilt),
   `kin.set_probing` (homing), `probe.LookupZSteppers`,
   `idex_modes` calling `kin.update_limits` and
   `kin.home_axis(state, axis_index, rail)`.
5. If upstream changes a C signature that carries coordinates, update
   `klippy/chelper/__init__.py` and every Python caller;
   `grep -rn "trapq_append(\|itersolve_set_position(\|calc_position_from_coord(" klippy`
   lists them.  If upstream adds a kinematics, it needs no changes - it
   simply runs with three axes.
6. Run `./scripts/check_whitespace.sh`.
7. Run section 7.

## 7. Verification

All of this runs in Docker (no toolchain needed on the host).

### 7.1 Upstream regression suite

From the repository root:

```
docker run --rm -v "$PWD:/klipper:ro" -v "$PWD/../klipper-verify:/out" python:3.11-slim \
    bash -c "tr -d '\r' < /klipper/sim/run-regression.sh > /tmp/r.sh && bash /tmp/r.sh /klipper /out"
```

`sim/run-regression.sh` installs gcc and gcc-avr, copies the tree,
builds the AVR and Linux MCU dictionaries, builds the C helper, and runs
every upstream regression test whose dictionary is available (34 tests).
Expected result: `All 34 test cases passed`.

This is the check that the axis roster generalisation did not change
upstream behaviour: those configs all run with three kinematic axes.

### 7.2 The machine flavours

`sim/verify-axes.sh` builds once and exercises every flavour:

```
docker compose -f docker-compose.sim.yml run --rm \
    --entrypoint /klipper/sim/verify-axes.sh klipper-sim
```

It runs the unit tests (`test/test_axis_motion.py` for the roster,
distance groups, move checking and the rotary modes;
`test/test_bed_transforms.py` for the bed-adjusting g-code transforms,
which have no upstream test of their own; `test/test_uv_derived.py` for
the derived carriage equations against the vendored reference solver,
with the compiled C solver required), then
boots klippy in batch mode against `sim/printer.sim.cfg`,
`sim/printer.sim.uvderived.cfg` and `sim/printer.sim.rotary.cfg`.
The first two run the *same* g-code, which is the point: adding the
derived carriages must not change the g-code.

The unit tests also run standalone with plain Python, no compiler:

```
python test/test_axis_motion.py
python test/test_bed_transforms.py
python test/test_uv_derived.py
```

### 7.3 Differential motion check

For a motion-level comparison against a previous version, run both trees
in batch mode against the same g-code with `[move_capture]` enabled and
diff the JSONL output with `sim/compare_moves.py`:

```
python sim/compare_moves.py baseline.jsonl candidate.jsonl
```

Identical positions and extrusion per move are expected, and velocity
differences should be explainable.  Note that `klippy -i` reads the file
in 4096-byte chunks and cannot decode multi-byte UTF-8 characters split
across a chunk; use ASCII-only g-code for batch runs.

`gcode/bed_pose_smoke.gcode` exercises the bed pose on any flavour and
`gcode/rotary_smoke.gcode` exercises the rotary axis.

### 7.4 On the machine

Finally verify on the machine: `G28`, `PROBE_ACCURACY`, `Z_TILT_ADJUST`
followed by a second `Z_TILT_ADJUST` (should converge), and a short
print.  With the derived carriages fitted, also compare `travel_check`
and `homing_mode` settings against each other before choosing.
