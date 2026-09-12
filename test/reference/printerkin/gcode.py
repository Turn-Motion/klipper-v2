"""
printerkin.gcode
=============
Minimal, dependency-free parsing of a single G-code line into axis values.

Only the five machine axes (X, Y, Z, A, B) are read.  Parsing is *modal*: axes
not mentioned on the line keep their value from ``current`` (carry-forward), so
you can thread state across a program::

    state = Axes(*machine.home)
    for line in program:
        state = parse_line(line, state)
        ...
"""

from __future__ import annotations

import re
from dataclasses import dataclass

# One axis word: a letter in {X,Y,Z,A,B} followed by a (possibly signed/float/exp) number.
_AXIS_RE = re.compile(r"([XYZAB])\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", re.IGNORECASE)


@dataclass(frozen=True)
class Axes:
    """One machine position in G-code units."""

    x: float
    y: float
    z: float
    a: float
    b: float

    def as_tuple(self) -> tuple[float, float, float, float, float]:
        return (self.x, self.y, self.z, self.a, self.b)


def parse_line(line: str, current: Axes) -> Axes:
    """
    Return the machine position after applying one G-code *line*.

    Comments (after ``;``) are stripped.  Any of X/Y/Z/A/B present on the line
    overwrite the corresponding value in *current*; the rest carry forward.  A
    line with no axis words (e.g. ``G28``, a bare comment) returns *current*
    unchanged.
    """
    text = line.split(";", 1)[0]
    vals = {"X": current.x, "Y": current.y, "Z": current.z, "A": current.a, "B": current.b}
    for axis, num in _AXIS_RE.findall(text):
        vals[axis.upper()] = float(num)
    return Axes(vals["X"], vals["Y"], vals["Z"], vals["A"], vals["B"])
