# adds support fro ARC commands via G2/G3
#
# Copyright (C) 2019  Aleksej Vasiljkovic <achmed21@gmail.com>
#
# function planArc() originates from https://github.com/MarlinFirmware/Marlin
# Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

# Coordinates created by this are converted into G1 commands.
#
# supports XY, XZ & YZ planes with remaining axis as helical

# Enum
ARC_PLANE_X_Y = 0
ARC_PLANE_X_Z = 1
ARC_PLANE_Y_Z = 2

# Enum
X_AXIS = 0
Y_AXIS = 1
Z_AXIS = 2


class ArcSupport:

    def __init__(self, config):
        self.printer = config.get_printer()
        self.mm_per_arc_segment = config.getfloat('resolution', 1., above=0.0)

        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("G2", self.cmd_G2)
        self.gcode.register_command("G3", self.cmd_G3)

        self.gcode.register_command("G17", self.cmd_G17)
        self.gcode.register_command("G18", self.cmd_G18)
        self.gcode.register_command("G19", self.cmd_G19)

        # backwards compatibility, prior implementation only supported XY
        self.plane = ARC_PLANE_X_Y

    def cmd_G2(self, gcmd):
        self._cmd_inner(gcmd, True)

    def cmd_G3(self, gcmd):
        self._cmd_inner(gcmd, False)

    def cmd_G17(self, gcmd):
        self.plane = ARC_PLANE_X_Y

    def cmd_G18(self, gcmd):
        self.plane = ARC_PLANE_X_Z

    def cmd_G19(self, gcmd):
        self.plane = ARC_PLANE_Y_Z

    def _cmd_inner(self, gcmd, clockwise):
        gcodestatus = self.gcode_move.get_status()
        if not gcodestatus['absolute_coordinates']:
            raise gcmd.error("G2/G3 does not support relative move mode")
        currentPos = gcodestatus['gcode_position']
        absolut_extrude = gcodestatus['absolute_extrude']

        # The kinematic axes of this printer, in order
        toolhead = self.printer.lookup_object('toolhead')
        axis_names = toolhead.get_axis_names()
        e_index = toolhead.get_extruder_index()
        # Deliberate 'xyz' literal: a G17/G18/G19 arc PLANE is defined
        # over x, y and z, so those three must exist and be in that order.
        # Every other kinematic axis is interpolated linearly along the
        # arc and does come from the roster.  See FORK_CHANGES.md
        # section 6.
        if len(axis_names) < 3 or axis_names[:3] != 'xyz':
            raise gcmd.error("G2/G3 requires x, y and z axes")

        # Parse parameters - every kinematic axis may be given
        asTarget = [gcmd.get_float(name.upper(), currentPos[index])
                    for index, name in enumerate(axis_names)]

        if 'r' not in axis_names and gcmd.get_float("R", None) is not None:
            raise gcmd.error("G2/G3 does not support R moves")

        # determine the plane coordinates and the helical axis
        I = gcmd.get_float('I', 0.)
        J = gcmd.get_float('J', 0.)
        asPlanar = (I, J)
        axes = (X_AXIS, Y_AXIS, Z_AXIS)
        if self.plane == ARC_PLANE_X_Z:
            K = gcmd.get_float('K', 0.)
            asPlanar = (I, K)
            axes = (X_AXIS, Z_AXIS, Y_AXIS)
        elif self.plane == ARC_PLANE_Y_Z:
            K = gcmd.get_float('K', 0.)
            asPlanar = (J, K)
            axes = (Y_AXIS, Z_AXIS, X_AXIS)

        if not (asPlanar[0] or asPlanar[1]):
            raise gcmd.error("G2/G3 requires IJ, IK or JK parameters")

        # Every kinematic axis that is not part of the arc plane and
        # not the helical axis travels linearly along the arc
        linear_axes = [i for i in range(len(axis_names)) if i not in axes]

        # Build linear coordinates to move
        self.planArc(currentPos, asTarget, asPlanar, clockwise,
                     gcmd, absolut_extrude, axis_names, e_index, linear_axes,
                     *axes)

    # function planArc() originates from marlin plan_arc()
    # https://github.com/MarlinFirmware/Marlin
    #
    # The arc is approximated by generating many small linear segments.
    # The length of each segment is configured in MM_PER_ARC_SEGMENT
    # Arcs smaller then this value, will be a Line only
    #
    # alpha and beta axes are the current plane, helical axis is linear travel
    def planArc(self, currentPos, targetPos, offset, clockwise,
                gcmd, absolut_extrude, axis_names, e_index, linear_axes,
                alpha_axis, beta_axis, helical_axis):
        # todo: sometimes produces full circles

        # Radius vector from center to current location
        r_P = -offset[0]
        r_Q = -offset[1]

        # Determine angular travel
        center_P = currentPos[alpha_axis] - r_P
        center_Q = currentPos[beta_axis] - r_Q
        rt_Alpha = targetPos[alpha_axis] - center_P
        rt_Beta = targetPos[beta_axis] - center_Q
        angular_travel = math.atan2(r_P * rt_Beta - r_Q * rt_Alpha,
                                    r_P * rt_Alpha + r_Q * rt_Beta)
        if angular_travel < 0.:
            angular_travel += 2. * math.pi
        if clockwise:
            angular_travel -= 2. * math.pi

        if (angular_travel == 0.
            and currentPos[alpha_axis] == targetPos[alpha_axis]
            and currentPos[beta_axis] == targetPos[beta_axis]):
            # Make a circle if the angular rotation is 0 and the
            # target is current position
            angular_travel = 2. * math.pi

        # Determine number of segments
        linear_travel = targetPos[helical_axis] - currentPos[helical_axis]
        radius = math.hypot(r_P, r_Q)
        flat_mm = radius * angular_travel
        if linear_travel:
            mm_of_travel = math.hypot(flat_mm, linear_travel)
        else:
            mm_of_travel = math.fabs(flat_mm)
        segments = max(1., math.floor(mm_of_travel / self.mm_per_arc_segment))

        # Generate coordinates
        theta_per_segment = angular_travel / segments
        linear_per_segment = linear_travel / segments

        asE = gcmd.get_float("E", None)
        asF = gcmd.get_float("F", None)

        e_per_move = e_base = 0.
        if asE is not None:
            if absolut_extrude:
                e_base = currentPos[e_index]
            e_per_move = (asE - e_base) / segments
        # Any remaining kinematic axis travels linearly along the arc
        axis_per_segment = {
            axis: (targetPos[axis] - currentPos[axis]) / segments
            for axis in linear_axes}

        for i in range(1, int(segments) + 1):
            dist_Helical = i * linear_per_segment
            c_theta = i * theta_per_segment
            cos_Ti = math.cos(c_theta)
            sin_Ti = math.sin(c_theta)
            r_P = -offset[0] * cos_Ti + offset[1] * sin_Ti
            r_Q = -offset[0] * sin_Ti - offset[1] * cos_Ti

            c = [None] * len(axis_names)
            c[alpha_axis] = center_P + r_P
            c[beta_axis] = center_Q + r_Q
            c[helical_axis] = currentPos[helical_axis] + dist_Helical
            for axis, per_segment in axis_per_segment.items():
                c[axis] = currentPos[axis] + i * per_segment

            if i == segments:
                c = targetPos
            # Convert coords into G1 commands
            g1_params = {name.upper(): c[index]
                         for index, name in enumerate(axis_names)}
            if e_per_move:
                g1_params['E'] = e_base + e_per_move
                if absolut_extrude:
                    e_base += e_per_move
            if asF is not None:
                g1_params['F'] = asF
            g1_gcmd = self.gcode.create_gcode_command("G1", "G1", g1_params)
            self.gcode_move.cmd_G1(g1_gcmd)

def load_config(config):
    return ArcSupport(config)
