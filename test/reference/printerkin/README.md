# printerkin

Self-contained kinematics for the 3-axis tilting-table printer: **passive-joint
solve** and **obstacle clearance**, with the machine geometry baked in.

- Pure standard library — no numpy, no file I/O required.
- The math matches the main app's `kinematics.solver` / `application.collision`
  exactly (verified to ~1e-14), so results agree with the visualizer.
- Drop-in as a submodule; the rest of the kinematics can migrate here later.

## Install / import

It's a plain package folder — put `printerkin/` on the path and `import printerkin`.

## What it computes

For a commanded `(Z, A, B)` the three carriages sit at known heights and the
rigid table forces the three ball joints to slide. Those slides are the passive
joints (all `0` at home):

| Field | Joint | Meaning |
|---|---|---|
| `A_slide` (λ_A) | A (front-left) | ball-joint slide in **X** |
| `B_slide` (λ_B) | B (front-right) | ball-joint slide in **X** |
| `Z_slide` (μ_Z) | Z (back) | ball-joint slide in **Y** |

Clearance is reported per rail as **two** numbers:

- **`clearance`** — operational safe gap (mm). `>0` safe, `<0` collision, `0`
  touching. Sign normalised so positive is always good.
- **`obstacle_relative`** — the arm tip in the **obstacle's own local frame**
  (`arm_end_x - obstacle_x`, world +X). `0` = contact; the sign says which side.
  For rail A this equals `+clearance`; for rail B it equals `-clearance`.

`X`/`Y` (the nozzle) do not affect passive joints or clearance.

## API

```python
import printerkin

# 1) Passive joints from a G-code line (axes not on the line carry from home)
ps = printerkin.solve_passive_line("G1 X10 Y20 Z460 A470 B465")
ps.A_slide, ps.B_slide, ps.Z_slide      # passive joint values (mm)
ps.A, ps.B, ps.Z                        # world ball-joint points (x, y, z)

# ...or from explicit axes
ps = printerkin.solve_passive(z=460, a=470, b=465)

# 2) Clearance from a G-code line
cl = printerkin.clearance_line("G1 Z460 A470 B465")
cl.A.clearance          # +safe / -collision
cl.A.obstacle_relative  # arm tip relative to the obstacle (obstacle-local)
cl.A.arm_end_x, cl.A.obstacle_x, cl.A.collides
cl.collides             # any rail colliding?
cl.min_clearance        # worst-case across rails

# ...or from explicit axes
cl = printerkin.compute_clearance(z=460, a=470, b=465)
```

### Modal parsing across a whole program

```python
from printerkin import default, parse_line

state = default.home_axes()
for line in program:
    state = parse_line(line, state)
    cl = default.compute_clearance(state.z, state.a, state.b)
    ...
```

### Custom machine

The default geometry lives in `config.DEFAULT_MACHINE`. Override it in code or
load the app's `machine.json` (same shape):

```python
from printerkin import Kinematics, MachineConfig, RailConfig

k = Kinematics(MachineConfig.from_json("machine.json"))
k = Kinematics(MachineConfig(
    A=RailConfig(origin=(0,0,0), offset=(-23,18.4,-6.2625),
                 arm_length=53.07, obstacle_offset_x=-64.71),
    B=RailConfig(origin=(600,0,0), offset=(23,18.4,-6.2625),
                 arm_length=53.07, obstacle_offset_x=61.61),
    Z=RailConfig(origin=(300,300,0), offset=(0,23,-6.2625), arm_length=53.07),
    home=(-32.5, -97, 468.6, 468.6, 468.6),
))
```

## Errors

- `InfeasibleConfig` — the commanded `(Z, A, B)` can't be assembled (a rigid link
  can't reach). Same condition the app rejects.
- `SingularConfig` — the A-B projected separation is ~0 (a mechanism singularity).

## Assumptions

- The closed-form trilateration requires the A and B ball joints to share a Y
  coordinate (`Ay == By`); `Kinematics` checks this at construction.
- Only A/B carry X-axis obstacles in this model; the back rail (Z) has none.
- Passive branch choices (A-B sign from home layout, smaller-magnitude μ_Z) match
  the app — the physical assembly mode.

## Layout

```
printerkin/
  __init__.py     public API + default-machine convenience functions
  config.py       RailConfig / MachineConfig + baked-in DEFAULT_MACHINE
  gcode.py        single-line modal G-code axis parsing
  solver.py       Kinematics: passive solve + clearance
  clearance.py    result types + the per-rail clearance formula
```
