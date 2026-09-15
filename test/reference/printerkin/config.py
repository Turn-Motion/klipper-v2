"""
printerkin.config
=================
Machine geometry for the tilting-table kinematics.

The default machine is defined by the bundled :data:`MACHINE_JSON` file that ships
*inside* this package (``printerkin/machine.json``).  It is loaded at import time
into :data:`DEFAULT_MACHINE`.  If that file is ever missing or unreadable (e.g. a
stripped/frozen build), the package falls back to :data:`_BAKED_MACHINE`, an
identical copy hard-coded in Python — so the package is always self-contained.

You can also load any other machine from a dict or a ``machine.json`` (the same
shape the main app uses) via :meth:`MachineConfig.from_dict` /
:meth:`MachineConfig.from_json`.

Geometry recap
--------------
Each of the three carriages (A = front-left, B = front-right, Z = back) is a
vertical actuator whose base sits at ``origin`` (world).  ``offset`` is the fixed
vector from that base to the ball-joint centre in the home pose, so::

    base (home ball-joint centre, world) = origin + offset

``arm_length`` is how far the rigid arm reaches from the ball-joint centre toward
its obstacle (A: -X, B: +X).  ``obstacle_offset_x`` is the obstacle's X measured
*relative to the home ball-joint centre* (so the absolute world X is
``base.x + obstacle_offset_x``); ``None`` means that rail has no obstacle.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from importlib import resources
from pathlib import Path

# Name of the config file bundled inside this package.
MACHINE_JSON = "machine.json"


@dataclass(frozen=True)
class RailConfig:
    """Geometry of one carriage/rail."""

    origin: tuple[float, float, float]
    """World position of the actuator base."""

    offset: tuple[float, float, float]
    """Fixed vector base -> ball-joint centre at home."""

    arm_length: float = 0.0
    """Arm reach from ball-joint centre toward the obstacle (mm)."""

    obstacle_offset_x: float | None = None
    """Obstacle X relative to the home ball-joint centre; ``None`` = no obstacle."""

    @property
    def base(self) -> tuple[float, float, float]:
        """Home ball-joint centre in world coordinates (``origin + offset``)."""
        ox, oy, oz = self.origin
        fx, fy, fz = self.offset
        return (ox + fx, oy + fy, oz + fz)


@dataclass(frozen=True)
class MachineConfig:
    """Full machine geometry (three rails + the home command)."""

    A: RailConfig
    B: RailConfig
    Z: RailConfig
    home: tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)
    """Home command (X, Y, Z, A, B) in G-code units."""

    # ── Constructors ─────────────────────────────────────────────────────────

    @classmethod
    def from_dict(cls, data: dict) -> "MachineConfig":
        """
        Build from a mapping shaped like the app's ``machine.json``:

            {"rails": {"A": {...}, "B": {...}, "Z": {...}}, "home": {"Z":..,"A":..,"B":..}}

        A bare ``{"A": {...}, "B": {...}, "Z": {...}}`` (no ``rails`` key) also works.
        """
        rails = data.get("rails", data)

        def rail(name: str) -> RailConfig:
            r = rails[name]
            obs = r.get("obstacle_offset_x")
            return RailConfig(
                origin=tuple(float(v) for v in r["origin"]),          # type: ignore[arg-type]
                offset=tuple(float(v) for v in r["offset"]),          # type: ignore[arg-type]
                arm_length=float(r.get("arm_length", 0.0)),
                obstacle_offset_x=(None if obs is None else float(obs)),
            )

        h = data.get("home", {})
        home = (
            float(h.get("X", 0.0)),
            float(h.get("Y", 0.0)),
            float(h.get("Z", 0.0)),
            float(h.get("A", 0.0)),
            float(h.get("B", 0.0)),
        )
        return cls(A=rail("A"), B=rail("B"), Z=rail("Z"), home=home)

    @classmethod
    def from_json(cls, path: str | Path) -> "MachineConfig":
        """Load a machine from a ``machine.json`` file."""
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


# ── Baked-in fallback (identical to the bundled machine.json) ─────────────────
# Used only if the bundled JSON can't be read, so the package still works when
# the data file is stripped from a frozen/zipped build.

_BAKED_MACHINE = MachineConfig(
    A=RailConfig(
        origin=(0.0, 0.0, 0.0),
        offset=(-23.0, 18.4, -6.2625),
        arm_length=53.07,
        obstacle_offset_x=-64.71,
    ),
    B=RailConfig(
        origin=(600.0, 0.0, 0.0),
        offset=(23.0, 18.4, -6.2625),
        arm_length=53.07,
        obstacle_offset_x=61.61,
    ),
    Z=RailConfig(
        origin=(300.0, 300.0, 0.0),
        offset=(0.0, 23.0, -6.2625),
        arm_length=53.07,
        obstacle_offset_x=None,   # no obstacle on the back rail
    ),
    home=(-32.5, -97.0, 468.6, 468.6, 468.6),
)


def _load_default_machine() -> MachineConfig:
    """Load the bundled ``machine.json``; fall back to the baked-in copy."""
    try:
        text = resources.files(__package__).joinpath(MACHINE_JSON).read_text(encoding="utf-8")
        return MachineConfig.from_dict(json.loads(text))
    except Exception:
        return _BAKED_MACHINE


# The default machine, sourced from the bundled printerkin/machine.json.
DEFAULT_MACHINE = _load_default_machine()
