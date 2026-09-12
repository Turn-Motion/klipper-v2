"""
printerkin.solver
==============
The passive-joint solve and clearance, wrapped in a :class:`Kinematics` object.

For a commanded ``(Z, A, B)`` the three carriages sit at known heights; the rigid
table forces the three ball joints to slide horizontally.  Those slides are the
*passive joints*:

    A_slide (lambda_A) : A ball-joint slide in X
    B_slide (lambda_B) : B ball-joint slide in X
    Z_slide (mu_Z)     : Z ball-joint slide in Y

all zero in the home pose.  The math (sign flip, sqrt branches) matches the main
app's ``kinematics.solver`` exactly, so results agree with the visualizer.

Assumption: the closed-form ``X`` step requires the A and B ball joints to share a
Y coordinate (``Ay == By``), which holds for this machine.  ``Kinematics`` checks
it at construction and raises if violated.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from .clearance import ClearanceState, RailClearance, rail_clearance
from .config import DEFAULT_MACHINE, MachineConfig
from .gcode import Axes, parse_line

_EPS = 1e-9   # near-zero guard for the singular A-B separation
_TOL = 1e-7   # negative-radicand tolerance before declaring infeasible


class InfeasibleConfig(ValueError):
    """The commanded (Z, A, B) cannot be assembled (a rigid link can't reach)."""


class SingularConfig(ValueError):
    """The mechanism is at a singularity (A-B projected separation ~ 0)."""


@dataclass(frozen=True)
class PassiveState:
    """Result of the passive solve for one commanded position."""

    # Passive joint values (mm) — the horizontal ball-joint slides, 0 at home.
    A_slide: float
    B_slide: float
    Z_slide: float

    # World ball-joint attachment points (x, y, z).
    A: tuple[float, float, float]
    B: tuple[float, float, float]
    Z: tuple[float, float, float]

    # The commanded axes (G-code units) this was solved for.
    z: float
    a: float
    b: float


class Kinematics:
    """Solver bound to a :class:`MachineConfig`; precomputes the constants once."""

    def __init__(self, machine: MachineConfig = DEFAULT_MACHINE) -> None:
        self.machine = machine

        self.At = machine.A.base
        self.Bt = machine.B.base
        self.Zt = machine.Z.base

        if abs(self.At[1] - self.Bt[1]) > 1e-6:
            raise ValueError(
                "printerkin closed-form requires the A and B ball joints to share a Y "
                f"coordinate (Ay={self.At[1]}, By={self.Bt[1]})."
            )

        self.dAB2 = _dist2(self.Bt, self.At)
        self.dAZ2 = _dist2(self.Zt, self.At)
        self.dBZ2 = _dist2(self.Zt, self.Bt)

        # Absolute obstacle X = home ball-joint X + offset (slides are 0 at home).
        self.A_obstacle_x = _abs_obstacle_x(machine.A)
        self.B_obstacle_x = _abs_obstacle_x(machine.B)

    # ── Passive solve ────────────────────────────────────────────────────────

    def solve_passive(self, z: float, a: float, b: float) -> PassiveState:
        """Solve the passive joints for a commanded ``(Z, A, B)`` (G-code units)."""
        Ax, Ay, Az = self.At
        Bx, By, Bz = self.Bt
        Zx, Zy, Zz = self.Zt
        dAB2, dAZ2, dBZ2 = self.dAB2, self.dAZ2, self.dBZ2

        # Firmware uses Z-down for the linear joints; world is Z-up.
        z_w, a_w, b_w = -z, -a, -b

        # A-B: solve the projected X separation s_ab from the rigid A-B distance.
        dzAB = (Bz + b_w) - (Az + a_w)
        rad_s = dAB2 - (By - Ay) ** 2 - dzAB ** 2
        if rad_s < -_TOL:
            raise InfeasibleConfig(
                f"A-B link cannot span the commanded heights (rad_s={rad_s:.6g})"
            )
        rad_s = max(0.0, rad_s)

        s_sign = 1.0 if (Bx - Ax) >= 0.0 else -1.0   # assembly branch fixed by home layout
        s_ab = s_sign * sqrt(rad_s)
        if abs(s_ab) < _EPS:
            raise SingularConfig("A-B projected separation is ~0 (singular)")

        # Trilaterate Z along the A-B axis (requires Ay == By, checked in __init__).
        u = (Zz + z_w) - (Az + a_w)
        v = (Zz + z_w) - (Bz + b_w)
        X = (s_ab ** 2 + v ** 2 - u ** 2 - (dBZ2 - dAZ2)) / (2.0 * s_ab)

        A_slide = Zx - Ax - X
        B_slide = A_slide + s_ab - (Bx - Ax)

        # Z's perpendicular (Y) slide; pick the branch nearest home.
        rad_m = dAZ2 - u ** 2 - X ** 2
        if rad_m < -_TOL:
            raise InfeasibleConfig(f"Z link cannot reach (rad_m={rad_m:.6g})")
        rad_m = max(0.0, rad_m)

        qy = Zy - Ay
        root_m = sqrt(rad_m)
        mu1, mu2 = -qy + root_m, -qy - root_m
        Z_slide = mu1 if abs(mu1) <= abs(mu2) else mu2

        Aw = (Ax + A_slide, Ay, Az + a_w)
        Bw = (Bx + B_slide, By, Bz + b_w)
        Zw = (Zx, Zy + Z_slide, Zz + z_w)
        return PassiveState(A_slide, B_slide, Z_slide, Aw, Bw, Zw, z, a, b)

    def solve_passive_line(self, line: str, current: Axes | None = None) -> PassiveState:
        """Parse one G-code *line* (modal on *current*, default home) and solve."""
        ax = parse_line(line, self._current_or_home(current))
        return self.solve_passive(ax.z, ax.a, ax.b)

    # ── Clearance ────────────────────────────────────────────────────────────

    def clearance_from_passive(self, passive: PassiveState) -> ClearanceState:
        """Clearance for an already-solved :class:`PassiveState`."""
        return ClearanceState(
            A=self._rail(self.machine.A, passive.A[0], self.A_obstacle_x),
            B=self._rail(self.machine.B, passive.B[0], self.B_obstacle_x),
            Z=None,   # back rail has no X-axis obstacle in this model
        )

    def compute_clearance(self, z: float, a: float, b: float) -> ClearanceState:
        """Solve the passive joints for ``(Z, A, B)`` and return the clearance."""
        return self.clearance_from_passive(self.solve_passive(z, a, b))

    def clearance_line(self, line: str, current: Axes | None = None) -> ClearanceState:
        """Parse one G-code *line* (modal on *current*, default home) and score it."""
        ax = parse_line(line, self._current_or_home(current))
        return self.compute_clearance(ax.z, ax.a, ax.b)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def home_axes(self) -> Axes:
        """The home command as an :class:`Axes`."""
        return Axes(*self.machine.home)

    def _current_or_home(self, current: Axes | None) -> Axes:
        return self.home_axes() if current is None else current

    @staticmethod
    def _rail(rail, ball_x: float, obstacle_x: float | None) -> RailClearance | None:
        if obstacle_x is None or rail.obstacle_offset_x is None:
            return None
        return rail_clearance(
            ball_x=ball_x,
            arm_length=rail.arm_length,
            obstacle_x=obstacle_x,
            obstacle_offset_x=rail.obstacle_offset_x,
        )


# ── Module-private math ──────────────────────────────────────────────────────

def _dist2(p: tuple[float, float, float], q: tuple[float, float, float]) -> float:
    return (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2


def _abs_obstacle_x(rail) -> float | None:
    """Absolute world X of a rail's obstacle, or None (slides are 0 at home)."""
    if rail.obstacle_offset_x is None:
        return None
    return rail.base[0] + rail.obstacle_offset_x
