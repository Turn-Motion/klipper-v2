# Cartesian XY kinematics with three independent vertical axes (Z, A, B)
#
# The bed is carried by three motors: Z at the front and the A and B
# "bed joint" motors at the rear.  All three are first-class kinematic
# axes, so a move may command any combination of x, y, z, a and b.
#
# The machinery lives in axis_kinematics; this module only declares the
# axis roster.  Optional features come from config sections:
#   [dual_carriage]          a second X (or Y) carriage
#   [bed_joint_carriages]    the derived horizontal bed joint slides
#   [CartesianXY3ZSafety]    bed geometry safety checks
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2024-2026  Turn-Motion
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import axis_kinematics as ak

AXES = [
    ak.AxisDef('x', ak.PLANAR),
    ak.AxisDef('y', ak.PLANAR),
    ak.AxisDef('z', ak.VERTICAL),
    ak.AxisDef('a', ak.VERTICAL),
    ak.AxisDef('b', ak.VERTICAL),
]

def load_kinematics(toolhead, config):
    return ak.AxisKinematics(toolhead, config, AXES,
                             kin_name='cartesianxy3z', bed_safety=True)
