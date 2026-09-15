# Generic multi-axis cartesian-style kinematics engine
#
# One engine, several machines.  A "flavour" module (cartesianxy3z,
# cartesianxy3zr, ...) declares the axis roster - which axis letters the
# machine has, what config section drives each one, and which motion
# group it belongs to - and this engine does the rest: rails, homing,
# limits, move checking and status.
#
# Motion groups
#   planar    The nozzle gantry.  All planar axes share one euclidean
#             distance and the printer's max_velocity / max_accel.
#   vertical  The motors that carry the bed.  Each one is its own
#             distance group, so a move that tilts the bed gets the full
#             move time for each motor rather than sharing it, and they
#             all run at max_z_velocity / max_z_accel.
#   rotary    An additional actuated axis in degrees.  Its own limits,
#             and its own distance group unless it is configured to be
#             slaved to the linear axes.
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2024-2026  Turn-Motion
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper
from extras.homing import HomingMove
from . import idex_modes

PLANAR = 'planar'
VERTICAL = 'vertical'
ROTARY = 'rotary'

# Speed of the coordinated fast approach of the bed motors, and how far
# they back off from the endstops before each one is homed on its own
COORDINATED_HOME_SPEED = 15.0
COORDINATED_HOME_RETRACT = 2.0

# Description of one kinematic axis, declared by a flavour module
class AxisDef:
    def __init__(self, name, group, section=None):
        self.name = name
        self.group = group
        self.section = section or ('stepper_' + name)

# A rail-like wrapper for an axis that has no endstop.  Such an axis
# cannot be homed; it is usable over its configured range from startup
# and SET_KINEMATIC_POSITION redefines its zero.
class BareAxisRail:
    def __init__(self, section_config):
        self.name = section_config.get_name()
        self.stepper = stepper.PrinterStepper(section_config)
        self.position_min = section_config.getfloat('position_min')
        self.position_max = section_config.getfloat('position_max',
                                                    above=self.position_min)
    def get_name(self, short=False):
        if short and self.name.startswith('stepper'):
            return self.name[8:]
        return self.name
    def get_range(self):
        return self.position_min, self.position_max
    def get_steppers(self):
        return [self.stepper]
    def get_endstops(self):
        return []
    def get_homing_info(self):
        return None
    def setup_itersolve(self, alloc_func, *params):
        self.stepper.setup_itersolve(alloc_func, *params)
    def set_trapq(self, trapq):
        self.stepper.set_trapq(trapq)
    def set_position(self, coord):
        self.stepper.set_position(coord)

# One kinematic axis: its rail, its limits and its motion group
class Axis:
    def __init__(self, axis_def, index, rail, can_home):
        self.name = axis_def.name
        self.group = axis_def.group
        self.section = axis_def.section
        self.motion_mode = 'coordinated'
        self.index = index
        self.rail = rail
        self.can_home = can_home
        # An axis without an endstop is always usable over its range
        self.always_homed = not can_home
        self.limits = rail.get_range() if self.always_homed else (1.0, -1.0)
        # Velocity and acceleration limits; planar and vertical axes take
        # theirs from the toolhead so that SET_VELOCITY_LIMIT applies
        self.max_velocity = self.max_accel = None
    def is_homed(self):
        return self.limits[0] <= self.limits[1]

class AxisKinematics:
    def __init__(self, toolhead, config, axis_defs, kin_name,
                 bed_safety=False):
        self.printer = config.get_printer()
        self.toolhead = toolhead
        self.kin_name = kin_name
        self.axis_names = "".join([ad.name for ad in axis_defs])
        # Build one rail per axis
        self.axes = []
        for index, axis_def in enumerate(axis_defs):
            section_config = config.getsection(axis_def.section)
            if axis_def.group == ROTARY and not section_config.get(
                    'endstop_pin', None):
                # A rotary chuck usually has no home reference
                rail = BareAxisRail(section_config)
                can_home = False
            else:
                rail = stepper.LookupMultiRail(section_config)
                can_home = True
            rail.setup_itersolve('axis_stepper_alloc', index)
            self.axes.append(Axis(axis_def, index, rail, can_home))
        self.rails = [axis.rail for axis in self.axes]
        self.axis_by_name = {axis.name: axis for axis in self.axes}
        # Rotary axes get their own velocity limits, in degrees
        for axis in self.axes:
            if axis.group != ROTARY:
                continue
            max_velocity, max_accel = toolhead.get_max_velocity()
            axis.max_velocity = config.getfloat(
                'max_%s_velocity' % (axis.name,), max_velocity, above=0.)
            axis.max_accel = config.getfloat(
                'max_%s_accel' % (axis.name,), max_accel, above=0.)
        # Optional dual carriage on a planar axis
        self.dual_carriage_axis = None
        self.dc_module = None
        if config.has_section('dual_carriage'):
            self._setup_dual_carriage(config)
        # Distance groups: the planar axes share one, every vertical axis
        # gets its own, and a rotary axis gets one unless it is slaved
        self.distance_groups = self._build_distance_groups(config)
        self.slaved_axes = self._slaved_axes()
        # Optional derived (non-axis) steppers, e.g. the bed joint slides
        self.derived = None
        if config.has_section('bed_joint_carriages'):
            from . import bed_joint_carriages
            self.derived = bed_joint_carriages.BedJointCarriages(self, config)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
        # Reported axis ranges
        ranges = [axis.rail.get_range() for axis in self.axes]
        self.axes_min = toolhead.Coord([r[0] for r in ranges])
        self.axes_max = toolhead.Coord([r[1] for r in ranges])
        # Homing / probing state - the safety checks are suspended in both
        self.is_homing = False
        self.is_probing = False
        # Optional bed geometry safety checks
        self.safety_module = None
        if bed_safety:
            from . import cartesianxy3z_safety
            self.safety_module = cartesianxy3z_safety.CartesianXY3ZSafety(
                config)
        max_z_velocity, max_z_accel = toolhead.get_max_z_velocity()
        logging.info("%s: axes=%s max_z_velocity=%.1f max_z_accel=%.1f"
                     " distance_groups=%s", kin_name, self.axis_names,
                     max_z_velocity, max_z_accel, self.distance_groups)
    def _setup_dual_carriage(self, config):
        dc_config = config.getsection('dual_carriage')
        dc_axis = dc_config.getchoice('axis', ['x', 'y'])
        if dc_axis not in self.axis_by_name:
            raise config.error("dual_carriage axis '%s' is not an axis of the"
                               " %s kinematics" % (dc_axis, self.kin_name))
        self.dual_carriage_axis = dc_index = self.axis_by_name[dc_axis].index
        dc_rail = stepper.LookupMultiRail(dc_config)
        dc_rail.setup_itersolve('axis_stepper_alloc', dc_index)
        self.dc_rail = dc_rail
        self.dc_module = idex_modes.DualCarriages(
            self.printer, [self.rails[dc_index]], [dc_rail], axes=[dc_index],
            safe_dist=[dc_config.getfloat('safe_distance', None, minval=0.)])
    def _build_distance_groups(self, config):
        groups = []
        planar = tuple([axis.index for axis in self.axes
                        if axis.group == PLANAR])
        if planar:
            groups.append(planar)
        for axis in self.axes:
            if axis.group == VERTICAL:
                groups.append((axis.index,))
            elif axis.group == ROTARY:
                # "coordinated": the axis has its own share of the move
                # time, so a combined move is as slow as its slowest
                # part.  "slaved": the axis is carried along by the
                # linear axes and never extends the move.
                section = config.getsection(axis.section)
                mode = section.getchoice('motion_mode',
                                         ['coordinated', 'slaved'],
                                         'coordinated')
                axis.motion_mode = mode
                if mode == 'coordinated':
                    groups.append((axis.index,))
        return groups
    def _slaved_axes(self):
        # Axes that are in no distance group: they are carried along by
        # the other axes and never make a move longer
        grouped = set([i for g in self.distance_groups for i in g])
        return [axis for axis in self.axes if axis.index not in grouped]
    # Stepper and position plumbing
    def get_steppers(self):
        res = [s for rail in self.rails for s in rail.get_steppers()]
        if self.dc_module is not None:
            res.extend(self.dc_rail.get_steppers())
        if self.derived is not None:
            res.extend(self.derived.get_steppers())
        return res
    def get_derived_steppers(self):
        if self.derived is None:
            return []
        return self.derived.get_steppers()
    def get_bed_axes(self):
        return [axis.index for axis in self.axes if axis.group == VERTICAL]
    def calc_position(self, stepper_positions):
        # Only the axis rails contribute - the derived steppers are
        # outputs, never inputs
        rails = list(self.rails)
        if self.dc_module is not None:
            dc_index = self.dual_carriage_axis
            rails[dc_index] = (self.dc_module.get_primary_rail(dc_index)
                               or self.rails[dc_index])
        return [stepper_positions[rail.get_name()] for rail in rails]
    def update_limits(self, i, range):
        l, h = self.axes[i].limits
        # Only update limits if this axis was already homed, otherwise
        # leave it in the un-homed state
        if l <= h:
            self.axes[i].limits = range
    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        if self.dc_module is not None:
            self.dc_rail.set_position(newpos)
        if self.derived is not None:
            self.derived.set_position(newpos)
        for axis_name in homing_axes:
            axis = self.axis_by_name.get(axis_name)
            if axis is None:
                continue
            rail = None
            if self.dc_module is not None and axis.index == \
                    self.dual_carriage_axis:
                rail = self.dc_module.get_primary_rail(self.dual_carriage_axis)
            if rail is None:
                rail = axis.rail
            axis.limits = rail.get_range()
    def clear_homing_state(self, clear_axes):
        for axis in self.axes:
            if axis.name in clear_axes and not axis.always_homed:
                axis.limits = (1.0, -1.0)
    def set_probing(self, state):
        self.is_probing = state
    # Homing
    def home_axis(self, homing_state, axis, rail=None):
        # 'axis' is an Axis or an axis index; idex_modes passes an index
        # together with the carriage rail to home
        if not isinstance(axis, Axis):
            axis = self.axes[axis]
        if rail is None:
            rail = axis.rail
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None] * len(self.axes)
        homepos[axis.index] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis.index] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis.index] += 1.5 * (position_max - hi.position_endstop)
        homing_state.home_rails([rail], forcepos, homepos)
    def home_vertical_axes_together(self, homing_state, vertical):
        # Fast phase: drive every bed motor towards its endstop in one
        # coordinated move.  The move stops as soon as any endstop
        # triggers; the axes then back off slightly.  Each one is homed
        # individually afterwards for precision.
        gcode = self.printer.lookup_object('gcode')
        toolhead = self.toolhead
        try:
            # Force the kinematics to a known start position
            startpos = list(toolhead.get_position())
            for axis in vertical:
                startpos[axis.index] = axis.rail.get_range()[0]
            toolhead.set_position(startpos, homing_axes="".join(
                [axis.name for axis in vertical]))
            # Move every vertical axis towards its endstop
            movepos = list(startpos)
            all_endstops = []
            for axis in vertical:
                position_min, position_max = axis.rail.get_range()
                if axis.rail.get_homing_info().positive_dir:
                    movepos[axis.index] = position_max
                else:
                    movepos[axis.index] = position_min
                all_endstops.extend(axis.rail.get_endstops())
            hmove = HomingMove(self.printer, all_endstops, toolhead=toolhead)
            hmove.homing_move(movepos, COORDINATED_HOME_SPEED,
                              check_triggered=True)
            # Back off from the endstops
            retractpos = list(toolhead.get_position())
            for axis in vertical:
                if axis.rail.get_homing_info().positive_dir:
                    retractpos[axis.index] -= COORDINATED_HOME_RETRACT
                else:
                    retractpos[axis.index] += COORDINATED_HOME_RETRACT
            toolhead.move(retractpos, COORDINATED_HOME_SPEED)
            toolhead.wait_moves()
        except self.printer.command_error as e:
            # Not fatal - the individual homing that follows recovers
            gcode.respond_info("Coordinated bed homing failed: %s" % (str(e),))
    def home(self, homing_state):
        self.is_homing = True
        if self.derived is not None:
            self.derived.start_homing()
        try:
            self._do_home(homing_state)
        finally:
            self.is_homing = False
            if self.derived is not None:
                self.derived.finish_homing()
    def _do_home(self, homing_state):
        gcode = self.printer.lookup_object('gcode')
        requested = homing_state.get_axes()
        axes = [self.axes[index] for index in requested]
        # Home the vertical axes first, so the bed is clear of the nozzle
        vertical = [axis for axis in axes if axis.group == VERTICAL]
        all_vertical = [axis for axis in self.axes if axis.group == VERTICAL]
        if vertical:
            if len(vertical) == len(all_vertical) and len(vertical) > 1:
                # Every bed motor requested - coordinated fast home first
                gcode.respond_info("Starting coordinated bed homing")
                self.home_vertical_axes_together(homing_state, vertical)
            for axis in vertical:
                gcode.respond_info("Homing %s axis" % (axis.name.upper(),))
                self.home_axis(homing_state, axis)
        # Then the planar axes, then anything else that can be homed
        for axis in axes:
            if axis.group == VERTICAL:
                continue
            if not axis.can_home:
                # No endstop - already usable over its whole range
                continue
            if (self.dc_module is not None
                and axis.index == self.dual_carriage_axis):
                self.dc_module.home(homing_state, self.dual_carriage_axis)
            else:
                self.home_axis(homing_state, axis)
    # Move checking
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for axis in self.axes:
            i = axis.index
            low, high = axis.limits
            if move.axes_d[i] and (end_pos[i] < low or end_pos[i] > high):
                if low > high:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def _check_safety(self, move):
        if self.safety_module is None or self.is_homing or self.is_probing:
            # The end position of a homing/probing move is a travel
            # limit, not the point the bed will actually stop at
            return
        try:
            is_safe = self.safety_module.check_move(move)
        except self.printer.command_error:
            raise
        except Exception as e:
            logging.exception("%s safety check" % (self.kin_name,))
            raise move.move_error("Safety system error: %s" % (str(e),))
        if not is_safe:
            raise move.move_error("Safety check blocked move")
    def _group_limits(self, group):
        # Velocity and acceleration limits of a distance group
        axis = self.axes[group[0]]
        if axis.group == PLANAR:
            return self.toolhead.get_max_velocity()
        if axis.group == VERTICAL:
            return self.toolhead.get_max_z_velocity()
        return axis.max_velocity, axis.max_accel
    def check_move(self, move):
        axes_d = move.axes_d
        planar_only = True
        for axis in self.axes:
            if axis.group != PLANAR and axes_d[axis.index]:
                planar_only = False
                break
        # Range checking
        out_of_range = False
        for axis in self.axes:
            if axis.group != PLANAR:
                continue
            pos = move.end_pos[axis.index]
            if pos < axis.limits[0] or pos > axis.limits[1]:
                out_of_range = True
                break
        if out_of_range or not planar_only:
            self._check_endstops(move)
        self._check_safety(move)
        if planar_only:
            # Ordinary gantry move - the printer defaults already apply
            return
        # Each distance group runs at its own limit, and the group that
        # needs the most time sets the speed of the whole move
        move_d = move.move_d
        groups = self.distance_groups
        if self.slaved_axes:
            # A slaved axis does not bound a move it shares with other
            # axes, but when it moves on its own its limits are all
            # there is to go by
            grouped_moved = any([axes_d[i] for g in groups for i in g])
            if not grouped_moved:
                groups = [(axis.index,) for axis in self.slaved_axes
                          if axes_d[axis.index]]
        effective_v = effective_a = None
        if self.derived is not None:
            # Derived steppers are not axes, but their travel still
            # bounds the move
            limits = self.derived.check_move(move)
            if limits is not None:
                effective_v, effective_a = limits
        for group in groups:
            if len(group) == 1:
                group_d = abs(axes_d[group[0]])
            else:
                group_d = math.sqrt(sum([axes_d[i]**2 for i in group]))
            if not group_d:
                continue
            max_velocity, max_accel = self._group_limits(group)
            ratio = move_d / group_d
            v, a = max_velocity * ratio, max_accel * ratio
            effective_v = v if effective_v is None else min(effective_v, v)
            effective_a = a if effective_a is None else min(effective_a, a)
        if effective_v is not None:
            move.limit_speed(effective_v, effective_a)
    def get_status(self, eventtime):
        homed = "".join([axis.name for axis in self.axes if axis.is_homed()])
        res = {
            'homed_axes': homed,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }
        if self.derived is not None:
            res.update(self.derived.get_status(eventtime))
        return res
