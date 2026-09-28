# Turn multi-axis kinematics

**Software architecture document**

| | |
|---|---|
| Status | Draft |
| Owner | Turn-Motion firmware |
| Last updated | 17 September 2026 |
| Applies to | `cartesianxy3z`, `cartesianxy3zr`, and the optional derived bed joint carriages |
| Baseline | Klipper3d/klipper @ `2d7717e3` |
| Related | [Turn_Kinematics_Architecture.pdf](Turn_Kinematics_Architecture.pdf) (same material with diagrams), [FORK_CHANGES.md](../FORK_CHANGES.md) (maintenance ledger) |

## Purpose and scope

This document describes the motion architecture of the Turn Klipper fork:
how many kinematic axes a printer has, which component decides that, and
how the host software is written so the answer can differ between
machines. It is the reference for changes to motion, homing, probing and
g-code handling, and for configuring a machine against them.

### Why the fork is based on current upstream

The previous fork was based on Klipper from April 2025. Upstream moved 705
commits beyond it, rewriting the lookahead queue and moving step generation
into a new `motion_queuing` module. The fork's `toolhead.py` was by then an
old upstream file carrying five-axis edits; it could not be rebased
mechanically and was re-ported by hand onto current upstream.

Rebasing onto upstream is therefore an architectural constraint rather than
housekeeping:

- **Divergence compounds.** Upstream restructures the motion path every few
  releases. A fork that carries edits inside those files pays for each
  restructuring it skipped, and eventually cannot rebase at all.
- **Upstream's tests apply to this code.** Because the stock kinematics
  still run through this motion core, upstream's own regression
  configurations exercise it rather than bypassing it.
- **Fixes and hardware support arrive without work**, provided the fork is
  a small stack on top rather than a snapshot beside.
- **Changes can be proposed upstream.** That is the only route by which the
  fork's footprint shrinks instead of growing.

### Why the axis count is generalised rather than raised

Three machines are in scope: a gantry over a three-motor bed, the same bed
with derived horizontal joint carriages, and the same machine with a rotary
axis. Raising the hard-coded three to five, and then to six, makes each
machine a separate sweep through the motion core, and the variants cannot
be combined because each owns its own copy of that core.

Generalising instead puts the machine-specific part in a declaration and
leaves the shared part upstream-shaped, working for three axes as well as
for six. One repository, one build and one firmware serve every machine,
and `printer.cfg` selects between them. Stock Klipper kinematics keep
running from the same build, which is what makes the point above about
upstream's tests true.

### Scope

**In scope.** The axis roster and the interfaces built on it, the
kinematics engine and the machine variants it serves, the runtime path a
move takes, the configuration surface of each variant, and the conventions
that keep the fork rebaseable.

**Out of scope.** The MCU firmware, the serial protocol and step
compression, none of which this architecture changes. Upstream Klipper's
own kinematics, which continue to work unmodified. Machine mechanics and
electrical design.

## 1. The one idea

Upstream Klipper plans exactly three kinematic axes - x, y and z -
followed by the extruder and any "extra axes". That `3` is written into
the motion core, the g-code layer, the homing code and roughly twenty
other modules.

This fork removes the number. **The kinematics class owns the axis
roster and everything else reads it.**

```
index:   0   1   2  ...  K-1    K       K+1 ...
axis:    <the kinematics' axis letters>  e    extra axes
```

A kinematics publishes `axis_names`, one lower-case letter per axis, in
order. One that publishes nothing gets upstream's `"xyz"`, so every
stock kinematics runs at three axes with upstream's behaviour. The
extruder sits at index `K = len(axis_names)`.

| kinematics       | roster   | axes | extruder index |
|------------------|----------|------|----------------|
| `cartesianxy3z`  | `xyzab`  | 5    | 5              |
| `cartesianxy3zr` | `xyzabr` | 6    | 6              |
| `cartesian`, `corexy`, `delta`, ... | `xyz` | 3 | 3 |

Nothing in the host code may write a literal axis index. Ask instead:

```python
toolhead.get_axis_names()      # "xyzab"
toolhead.get_axis_count()      # 5
toolhead.get_extruder_index()  # 5
```

## 2. Machines

The Turn V2 is a cartesian gantry over a bed carried by **three**
independent vertical motors: Z at the front, A and B at the rear. All
three are first-class kinematic axes - planned in the same trapezoid as
X and Y, part of the move distance, homed through the normal homing
module, stepped from the same motion queue. They are deliberately *not*
upstream "extra axes", which cannot be homed by `G28`, are excluded from
the move distance, and force a full stop around a move that only tilts
the bed.

Three flavours exist today:

| Flavour | What it adds |
|---|---|
| `cartesianxy3z` | the baseline: X, Y and the three bed motors |
| `cartesianxy3z` + `[bed_joint_carriages]` | two horizontal bed-joint slides (U and V), *derived* from the bed pose rather than commanded |
| `cartesianxy3zr` | the baseline plus a rotary axis R, in degrees |

`[bed_joint_carriages]` is an optional section, so it combines with
either flavour.

## 3. Architecture

```
   G-code
     |
     v
   gcode_move  --- axis map, positions and every axis-naming
     |              command are built from the roster
     v
   move transforms   (bed_mesh, bed_tilt, z_thermal_adjust,
     |                exclude_object, ...)
     v
   ToolHead.move()  ->  Move + LookAheadQueue
     |
     v
   kin.check_move()   range checks, per-group speed limits
     |
     v
   trapq (C)          one motion queue, KIN_AXES coordinates
     |
     v
   itersolve          one solver callback per stepper
```

### The core

- **`klippy/toolhead.py`** reads the roster once the kinematics is
  loaded (`_setup_axes`), sizes the position tuple, and publishes
  `get_axis_names()`, `get_axis_count()` and `get_extruder_index()`.
- **`klippy/gcode.py`** holds one `Coord` class that is re-shaped in
  place when the roster is known, giving it one attribute per axis
  letter plus `e`. It must stay a single class object: config sections
  load before the toolhead, so modules capture a reference to it before
  the roster exists.
- **`klippy/axis_roster.py`** (fork-only) answers the machine-shaped
  questions - which axes carry the bed, which steppers those are, which
  steppers are derived rather than commanded, and the helpers that move
  a whole bed by a delta or to a height. Probing and bed levelling go
  through it instead of assuming Z.
- **`klippy/chelper/`** carries up to `KIN_AXES` (8) axes in
  `struct coord` as an array. `trapq_append`, `trapq_set_position`,
  `itersolve_set_position` and `itersolve_calc_position_from_coord` all
  take a count and an array.

### Move distance

Upstream measures a move by the euclidean distance over x, y and z. A
kinematics may instead publish `distance_groups`, a list of axis-index
tuples. Each group contributes the euclidean norm of its axes, and the
move distance is the **largest** group - so axes in different groups
each get the whole move time instead of sharing it.

- One group over every axis reproduces upstream exactly, and is the
  default.
- The Turn machines use `[(0,1), (2,), (3,), (4,)]`: the gantry in one
  group, each bed motor in its own. A move that lifts the bed 10 mm
  while traversing 100 mm in XY is as slow as the *slower* of those two,
  not as slow as their combination.
- An axis in **no** group is carried along and never lengthens a move.
  When it is the only axis moving, the largest single axis distance is
  the fallback, so it still moves and is still limited. This is what a
  `slaved` rotary axis does.

### The kinematics engine

`klippy/kinematics/axis_kinematics.py` is the engine; a flavour module
declares a roster and nothing else.

```python
# klippy/kinematics/cartesianxy3z.py
from . import axis_kinematics as ak

AXES = [ak.AxisDef('x', ak.PLANAR), ak.AxisDef('y', ak.PLANAR),
        ak.AxisDef('z', ak.VERTICAL), ak.AxisDef('a', ak.VERTICAL),
        ak.AxisDef('b', ak.VERTICAL)]

def load_kinematics(toolhead, config):
    return ak.AxisKinematics(toolhead, config, AXES,
                             kin_name='cartesianxy3z', bed_safety=True)
```

An `AxisDef` is a letter, a motion group and (optionally) a config
section name, defaulting to `stepper_<letter>`. The three motion groups:

| Group | Limits | Distance | Homing |
|---|---|---|---|
| `PLANAR` | `max_velocity` / `max_accel` | all planar axes in one euclidean group | individually, after the bed |
| `VERTICAL` | `max_z_velocity` / `max_z_accel` | one group each | coordinated fast approach of all of them, then individually; these are the bed axes |
| `ROTARY` | `max_<letter>_velocity` / `max_<letter>_accel` | one group each, or none if slaved | only if it has an endstop |

The engine builds the rails, homes, range-checks, and limits speed and
acceleration so that the group needing the most time sets the duration
of the move. An axis configured without an `endstop_pin` gets a
`BareAxisRail`: usable over its configured range from startup, skipped
by `G28`.

### Derived steppers

`klippy/kinematics/bed_joint_carriages.py` implements the U and V
slides. The bed is a rigid plate on three ball joints; as it tilts, the
A and B joints slide horizontally by an amount that plate rigidity fixes
completely. So U and V are **not** degrees of freedom. They are modelled
the way delta towers are - a stepper with a non-trivial position
function, solved in C by `kin_bedjoint.c` - rather than as g-code axes.

The consequences are the point of the design:

- G-code is unchanged. There is no U or V word, and the same file runs
  with or without the carriages fitted.
- The carriages follow every bed transform (z_tilt, `SET_GCODE_OFFSET`,
  bed_mesh, probe lifts) automatically, because they are recomputed from
  whatever the bed axes actually are.
- Homing them is unnecessary: `set_position` re-derives them.

They are excluded from the bed stepper set, so z_tilt and probing never
mistake them for bed motors.

## 4. Configuration

### Baseline: `cartesianxy3z`

```ini
[printer]
kinematics: cartesianxy3z
max_velocity: 100
max_accel: 3000
max_z_velocity: 8.4       # shared by Z, A and B
max_z_accel: 300

[stepper_x]               # and stepper_y, stepper_z, stepper_a, stepper_b
...
```

`max_z_velocity` and `max_z_accel` live on `[printer]` in this fork, not
on the kinematics section, and apply to every vertical axis.
`[dual_carriage]` works as upstream, on X or Y.

Full example: [config/example-cartesianxy3z-full.cfg](../config/example-cartesianxy3z-full.cfg).
The production machine is [config/example-Turn-V2.cfg](../config/example-Turn-V2.cfg).

### Rotary axis: `cartesianxy3zr`

```ini
[printer]
kinematics: cartesianxy3zr
max_r_velocity: 90        # degrees/s
max_r_accel: 800          # degrees/s^2

[stepper_r]
step_pin: PXX
dir_pin: !PXX
enable_pin: !PXX
microsteps: 16
rotation_distance: 360    # with gear_ratio, one G-code unit = one degree
gear_ratio: 6:1
position_min: -100000
position_max: 100000
motion_mode: coordinated
```

Degrees come from the config, not from code: `rotation_distance: 360`
with a `gear_ratio` makes one G-code unit one degree. The limit option
names follow the axis letter, so an axis named `c` would use
`max_c_velocity`.

Two choices, both configurable:

| Option | Values | Effect |
|---|---|---|
| `endstop_pin` on `[stepper_r]` | present / absent | Present: `G28` homes R like any axis, and `position_endstop`, `homing_speed` and the rest apply. Absent: R has no home reference, is usable over its range from startup, is skipped by `G28`, and `SET_KINEMATIC_POSITION R=<angle>` redefines its zero. |
| `motion_mode` on `[stepper_r]` | `coordinated` (default) / `slaved` | `coordinated`: R has its own share of the move time, so a combined move is as slow as its slowest part. `slaved`: the linear axes carry R along and it never lengthens a combined move. R moving on its own works in both modes, and is held to `max_r_velocity` / `max_r_accel` in both. |

Example: [config/example-cartesianxy3zr.cfg](../config/example-cartesianxy3zr.cfg).

```gcode
G1 R90            ; turn 90 degrees
G1 X100 R45       ; rotate and translate together
```

### Derived bed-joint carriages

Add to either flavour:

```ini
[bed_joint_carriages]
a_joint_home_x: -23       # world X of each ball joint with the bed level
b_joint_home_x: 623
travel_check: sampled
homing_mode: detached
#max_velocity: 8.4        # defaults to max_z_velocity
#max_accel: 300           # defaults to max_z_accel
#carriages: stepper_u, stepper_v

[stepper_u]
...
position_min: -120        # REQUIRED - see below
position_max: 120

[stepper_v]
...
position_min: -120
position_max: 120
```

`position_min` and `position_max` are required deliberately. The
carriages have no endstop and no encoder, so that range is the only
thing standing between a commanded bed pose and a ball joint being
driven off the end of its rail by the bed actuators. Zero is the
level-bed home position.

Two behaviours are selectable so both can be measured on the machine:

| Option | Values | Effect |
|---|---|---|
| `travel_check` | `sampled` (default) / `endpoints` | The carriage position is nonlinear in the bed pose. `sampled` evaluates the carriage path along the move, which catches a move whose two endpoints are in range but whose middle is not, and finds the true peak rate - at the end of a long roll sweep that is about twice the chord average. `endpoints` evaluates only the two ends, which is cheaper. |
| `homing_mode` | `detached` (default) / `attached` | Klipper homes by first forcing the start position well beyond the axis range, which for a single bed motor is a pose the rigid plate cannot occupy. `detached` takes the carriages off the motion queue for the duration; the plate back-drives them mechanically and they are re-derived from the true pose afterwards, emitting no steps. `attached` leaves them on the queue, following the commanded pose through the homing moves. |

Example: [config/example-bed-joint-carriages.cfg](../config/example-bed-joint-carriages.cfg).

### Bed geometry safety checks

Both flavours run a bed-joint clearance model that rejects a move which
would collide the extruder with the bed frame. It is **on by default** -
no config section is needed, and the built-in geometry defaults apply.
Add `[CartesianXY3ZSafety]` to override the geometry, or to switch the
checks off with `disable_safety_checks: True`. It adds the
`CHECK_CARTESIANXY3Z_SAFETY` and `PREVIEW_MOVE` commands.

The checks are suspended while homing and probing, because the end
position of such a move is a travel limit rather than a point the bed
will actually reach.

## 5. What changes for the operator

| Area | Difference from upstream |
|---|---|
| `G1` | Accepts a word per kinematic axis: `G1 X10 Y10 Z5 A5 B5` (and `R45` on the rotary flavour). Same for `G92`, `SET_GCODE_OFFSET`, `G2`/`G3`. |
| `M114`, `GET_POSITION` | Report every axis in the roster. |
| `SET_VELOCITY_LIMIT` | Adds `Z_VELOCITY=` and `Z_ACCEL=` for the shared vertical limits. |
| `SET_KINEMATIC_POSITION` | Takes a parameter per axis; `SET_HOMED` / `CLEAR_HOMED` follow the roster. |
| `G28` | Homes the bed axes first (coordinated fast approach, then each individually), then X and Y, then anything else with an endstop. |
| `Z_TILT_ADJUST` | Corrects the three bed motors in one coordinated move and applies session g-code offsets so they all report the same coordinate. Offsets reset on the next `G28`. |
| Probing | Lifts and descents move every bed motor by the same delta, so a tilt correction survives them. `probe_travel_limit` (default `max(-1.5, position_min)`) bounds a descent. |
| Extrusion | `max_extrude_cross_section` is **not enforced**, only logged: the move distance includes bed travel, so the ratio is not meaningful on this machine. |
| Status | `toolhead.position`, `axis_minimum` and `axis_maximum` carry every axis; `homed_axes` uses the roster letters. With the carriages fitted, `bed_joint_carriages` reports their positions. |

## 6. Known limitations

**Some upstream probing helpers still move the z axis alone.** A bed on
several motors is raised by moving all of them together, so a helper that
commands only z tips the plate rather than lifting it. The paths this fork
uses go through `axis_roster.bed_lift_coord`, but `probe_eddy_current.py`
and `temperature_probe.py` do not, and neither is used on the current
machine. Anything enabling them needs the same treatment.

**`SET_GCODE_OFFSET Z=` moves one motor of several.** A z-only offset tips
the bed, which is what a babystep control sends. Macros work around it by
naming every bed axis. Propagating a z offset to the other bed axes at the
`gcode_move` level would fix babystepping, propagating only to axes the
command did not name so that macros setting all of them stay correct.

## 7. Extending it

**A new machine flavour.** Add a module in `klippy/kinematics/` with an
`AXES` list and a `load_kinematics` that calls `AxisKinematics`. Do not
subclass an existing flavour, and add a worked example in `config/`.

**A new derived stepper.** Write a C solver in `klippy/chelper/` that
reads the axes it depends on from `move_get_coord()`, register it in
`chelper/__init__.py`, and add a feature module in the shape of
`bed_joint_carriages.py`: allocate the steppers, re-derive them in
`set_position`, range-check and rate-limit them in `check_move`, and
report them through `get_derived_steppers()` so probing and levelling
ignore them.

**A new axis on an existing flavour.** Add an `AxisDef`. If it needs
limits of its own, it is a `ROTARY`-style group; if it moves with the
bed, it is `VERTICAL` and it becomes a bed axis automatically.
