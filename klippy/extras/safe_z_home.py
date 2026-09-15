# Perform Z Homing at specific XY coordinates.
#
# Copyright (C) 2019 Florian Heilmann <Florian.Heilmann@gmx.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import axis_roster
from . import manual_probe

class SafeZHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        x_pos, y_pos = config.getfloatlist("home_xy_position", count=2)
        self.home_x_pos, self.home_y_pos = x_pos, y_pos
        self.z_hop = config.getfloat("z_hop", default=0.0)
        self.z_hop_speed = config.getfloat('z_hop_speed', 15., above=0.)
        zconfig = manual_probe.lookup_z_endstop_config(config)
        if zconfig is None:
            raise config.error('Missing Z endstop config for safe_z_homing')
        self.max_z = zconfig.getfloat('position_max', note_valid=False)
        self.speed = config.getfloat('speed', 50.0, above=0.)
        self.move_to_previous = config.getboolean('move_to_previous', False)
        self.printer.load_object(config, 'homing')
        self.gcode = self.printer.lookup_object('gcode')
        self.prev_G28 = self.gcode.register_command("G28", None)
        self.gcode.register_command("G28", self.cmd_G28)

        if config.has_section("homing_override"):
            raise config.error("homing_override and safe_z_homing cannot"
                               +" be used simultaneously")

    def _bed_hop_coord(self, toolhead, height):
        # A partial coordinate that lifts every bed axis to 'height'
        coord = [None] * toolhead.get_axis_count()
        for axis in axis_roster.get_bed_axes(self.printer):
            coord[axis] = height
        return coord
    def cmd_G28(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        axis_names = toolhead.get_axis_names()
        bed_axes = axis_roster.get_bed_axes(self.printer)
        bed_axis_names = "".join([axis_names[a] for a in bed_axes])

        # Perform Z Hop if necessary
        if self.z_hop != 0.0:
            # Check if Z axis is homed and its last known position
            curtime = self.printer.get_reactor().monotonic()
            kin_status = toolhead.get_kinematics().get_status(curtime)
            pos = toolhead.get_position()

            # Every bed axis carries the bed together
            if 'z' not in kin_status['homed_axes']:
                # Always perform the z_hop if the Z axis is not homed
                for axis in bed_axes:
                    pos[axis] = 0
                toolhead.set_position(pos, homing_axes=bed_axis_names)
                toolhead.manual_move(self._bed_hop_coord(toolhead, self.z_hop),
                                     self.z_hop_speed)
                toolhead.get_kinematics().clear_homing_state(bed_axis_names)
            elif pos[2] < self.z_hop:
                # If the Z axis is homed, and below z_hop, lift it to z_hop
                toolhead.manual_move(self._bed_hop_coord(toolhead, self.z_hop),
                                     self.z_hop_speed)

        # Determine which axes we need to home
        requested = [gcmd.get(name.upper(), None) is not None
                     for name in axis_names]
        if not any(requested):
            requested = [True] * len(axis_names)
        need_x = requested[0]
        need_y = requested[1]
        need_bed = any([requested[a] for a in bed_axes])

        # Home XY axes if necessary
        new_params = {}
        if need_x:
            new_params['X'] = '0'
        if need_y:
            new_params['Y'] = '0'
        if new_params:
            g28_gcmd = self.gcode.create_gcode_command("G28", "G28", new_params)
            self.prev_G28(g28_gcmd)

        # Home the bed axes if necessary (they are homed together)
        if need_bed:
            # Throw an error if X or Y are not homed
            curtime = self.printer.get_reactor().monotonic()
            kin_status = toolhead.get_kinematics().get_status(curtime)
            if ('x' not in kin_status['homed_axes'] or
                'y' not in kin_status['homed_axes']):
                raise gcmd.error("Must home X and Y axes first")
            # Move to safe XY homing position
            prevpos = toolhead.get_position()
            toolhead.manual_move([self.home_x_pos, self.home_y_pos], self.speed)
            # Home every bed axis
            bed_params = {axis_names[a].upper(): '0' for a in bed_axes}
            g28_gcmd = self.gcode.create_gcode_command("G28", "G28",
                                                       bed_params)
            self.prev_G28(g28_gcmd)
            # Perform Z Hop again for pressure-based probes
            if self.z_hop:
                pos = toolhead.get_position()
                if pos[2] < self.z_hop:
                    toolhead.manual_move(
                        self._bed_hop_coord(toolhead, self.z_hop),
                        self.z_hop_speed)
            # Move XY back to previous positions
            if self.move_to_previous:
                toolhead.manual_move(prevpos[:2], self.speed)

def load_config(config):
    return SafeZHoming(config)
