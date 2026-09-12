"""
printerkin.clearance
=================
Result types and the per-rail clearance formula.

Two numbers are reported per rail, and they mean different things:

``clearance``
    The *operational* safe gap in mm.  ``> 0`` = safe, ``< 0`` = the arm has
    crossed the obstacle (collision), ``0`` = just touching.  Sign is normalised
    per rail so "positive is always good", regardless of which side the obstacle
    is on.

``obstacle_relative``
    The arm tip's position expressed in the **obstacle's own local frame** — the
    obstacle is the origin and the axis is world ``+X``.  So it is simply
    ``arm_end_x - obstacle_x``.  ``0`` = contact; the sign tells you which side of
    the obstacle the tip is on.  For rail A (obstacle on -X, safe side +X) this is
    ``+clearance``; for rail B (obstacle on +X, safe side -X) it is ``-clearance``.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RailClearance:
    """Clearance of one rail's arm against its obstacle."""

    clearance: float
    """Operational safe gap (mm); >0 safe, <0 collision."""

    obstacle_relative: float
    """Arm tip position in the obstacle-local frame (mm, world +X); 0 = contact."""

    arm_end_x: float
    """World X of the arm tip (the point closest to the obstacle)."""

    obstacle_x: float
    """World X of the obstacle."""

    collides: bool
    """True when ``clearance < 0``."""


@dataclass(frozen=True)
class ClearanceState:
    """Clearance for every rail that has an obstacle configured."""

    A: RailClearance | None
    B: RailClearance | None
    Z: RailClearance | None = None

    @property
    def collides(self) -> bool:
        """True if any configured rail is in collision."""
        return any(r is not None and r.collides for r in (self.A, self.B, self.Z))

    @property
    def min_clearance(self) -> float | None:
        """Smallest (worst-case) clearance across configured rails, or None."""
        vals = [r.clearance for r in (self.A, self.B, self.Z) if r is not None]
        return min(vals) if vals else None


def rail_clearance(
    ball_x: float,
    arm_length: float,
    obstacle_x: float,
    obstacle_offset_x: float,
) -> RailClearance:
    """
    Clearance for a single X-axis rail against its flat obstacle.

    ``obstacle_offset_x`` only supplies the *direction* from ball joint to
    obstacle (its sign): the arm reaches ``toward`` the obstacle and the safe
    side is the opposite one.  The obstacle is a flat X-plane, so this is the
    plain point-to-plane gap along X.
    """
    toward = 1.0 if obstacle_offset_x >= 0.0 else -1.0  # +1: obstacle at +X, -1: at -X
    arm_end_x = ball_x + toward * arm_length
    obstacle_relative = arm_end_x - obstacle_x
    clearance = -toward * obstacle_relative
    return RailClearance(
        clearance=clearance,
        obstacle_relative=obstacle_relative,
        arm_end_x=arm_end_x,
        obstacle_x=obstacle_x,
        collides=clearance < 0.0,
    )
