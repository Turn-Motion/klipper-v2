"""
printerkin — self-contained tilting-table kinematics
=================================================
Passive-joint solve and obstacle clearance for the 3-axis tilting-table printer,
with the machine geometry baked in.  Pure standard library (no numpy, no I/O
required).  The math matches the main app's ``kinematics.solver`` /
``application.collision`` so results agree with the visualizer.

Quick start (default machine)
-----------------------------
>>> import printerkin
>>> ps = printerkin.solve_passive_line("G1 X10 Y20 Z460 A470 B465")
>>> round(ps.A_slide, 3), round(ps.B_slide, 3), round(ps.Z_slide, 3)   # doctest: +SKIP
>>> cl = printerkin.clearance_line("G1 Z460 A470 B465")
>>> cl.A.clearance, cl.A.obstacle_relative                              # doctest: +SKIP

Or with explicit axes:
>>> printerkin.compute_clearance(z=460, a=470, b=465).A.clearance          # doctest: +SKIP

Custom machine
--------------
>>> from printerkin import Kinematics, MachineConfig
>>> k = Kinematics(MachineConfig.from_json("machine.json"))
>>> k.solve_passive(z=460, a=470, b=465)                                # doctest: +SKIP

Public API
----------
Config   : MachineConfig, RailConfig, DEFAULT_MACHINE
Parsing  : Axes, parse_line
Solver   : Kinematics, PassiveState, InfeasibleConfig, SingularConfig
Clearance: ClearanceState, RailClearance
Default-machine functions: solve_passive, solve_passive_line,
                           compute_clearance, clearance_line
"""

from __future__ import annotations

from .clearance import ClearanceState, RailClearance
from .config import DEFAULT_MACHINE, MachineConfig, RailConfig
from .gcode import Axes, parse_line
from .solver import (
    InfeasibleConfig,
    Kinematics,
    PassiveState,
    SingularConfig,
)

__all__ = [
    "MachineConfig",
    "RailConfig",
    "DEFAULT_MACHINE",
    "Axes",
    "parse_line",
    "Kinematics",
    "PassiveState",
    "ClearanceState",
    "RailClearance",
    "InfeasibleConfig",
    "SingularConfig",
    "solve_passive",
    "solve_passive_line",
    "compute_clearance",
    "clearance_line",
]

# A ready-to-use solver on the default machine, shared by the module-level
# convenience functions below.
default = Kinematics(DEFAULT_MACHINE)


def solve_passive(z: float, a: float, b: float) -> PassiveState:
    """Passive solve for commanded ``(Z, A, B)`` on the default machine."""
    return default.solve_passive(z, a, b)


def solve_passive_line(line: str, current: Axes | None = None) -> PassiveState:
    """Passive solve for one G-code *line* on the default machine (modal on *current*)."""
    return default.solve_passive_line(line, current)


def compute_clearance(z: float, a: float, b: float) -> ClearanceState:
    """Clearance for commanded ``(Z, A, B)`` on the default machine."""
    return default.compute_clearance(z, a, b)


def clearance_line(line: str, current: Axes | None = None) -> ClearanceState:
    """Clearance for one G-code *line* on the default machine (modal on *current*)."""
    return default.clearance_line(line, current)
