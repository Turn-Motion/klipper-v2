# Cartesian XY3Z kinematics with an additional rotary axis (R)
#
# The same machine as cartesianxy3z plus a rotary axis commanded in
# degrees, planned in the same trapezoid as the linear axes.  "G1 R90"
# turns it 90 degrees; "G1 X100 R45" rotates and translates together.
#
# The R axis is configured entirely from [stepper_r]:
#   degrees come from the config, not the code - rotation_distance 360
#   with a gear_ratio makes one G-code unit one degree;
#   an endstop_pin makes R homeable with G28, and without one R is
#   usable over its configured range from startup and
#   SET_KINEMATIC_POSITION R=<angle> redefines its zero;
#   motion_mode selects whether R has its own share of the move time
#   ("coordinated", the default) or is carried along by the linear axes
#   without ever extending the move ("slaved").
# max_r_velocity and max_r_accel on [printer] bound it, in degrees.
#
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
    ak.AxisDef('r', ak.ROTARY),
]

def load_kinematics(toolhead, config):
    return ak.AxisKinematics(toolhead, config, AXES,
                             kin_name='cartesianxy3zr', bed_safety=True)
