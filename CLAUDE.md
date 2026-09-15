# Working in this repository

This is upstream Klipper plus the Turn-Motion changes for machines whose
bed is carried by three vertical motors, with an optional rotary axis and
optional derived bed-joint carriages. The number of kinematic axes is not
hard-coded: the kinematics class owns the axis roster and the core reads
it.

Before changing motion, homing, probing or g-code handling code, read:

- `docs/Turn_Kinematics.md` - the architecture and configuration guide:
  the axis roster, the kinematics engine, and how each machine flavour is
  configured.
- `FORK_CHANGES.md` - the maintenance ledger: every fork change file by
  file, the conflict-resolution rules, and the verification steps to run
  after rebasing onto a new upstream version.

Never write a literal kinematic axis index; ask the toolhead
(`get_axis_names`, `get_axis_count`, `get_extruder_index`) or
`klippy/axis_roster.py`.
