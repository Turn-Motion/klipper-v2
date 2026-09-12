# Cartesian XY kinematics with three independent vertical axes (Z, A, B)
#
# The bed is carried by three motors: Z at the front and the A and B
# "bed joint" motors at the rear.  All three are first-class kinematic
# axes, so a move may command any combination of x, y, z, a, and b.
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2024-2026  Turn-Motion
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper
from extras.homing import HomingMove
from . import idex_modes, cartesianxy3z_safety

AXES = "xyzab"
VERTICAL_AXES = (2, 3, 4)
COORDINATED_HOME_SPEED = 15.0
COORDINATED_HOME_RETRACT = 2.0

class CartesianXY3Z:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.axis_names = AXES
        # Setup axis rails
        self.dual_carriage_axis = None
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in AXES]
        for rail, axis in zip(self.rails, AXES):
            rail.setup_itersolve('cartesianxy3z_stepper_alloc', axis.encode())
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord([r[0] for r in ranges])
        self.axes_max = toolhead.Coord([r[1] for r in ranges])
        self.dc_module = None
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            dc_axis = dc_config.getchoice('axis', ['x', 'y'])
            self.dual_carriage_axis = {'x': 0, 'y': 1}[dc_axis]
            # setup second dual carriage rail
            self.rails.append(stepper.LookupMultiRail(dc_config))
            self.rails[5].setup_itersolve('cartesianxy3z_stepper_alloc',
                                          dc_axis.encode())
            self.dc_module = idex_modes.DualCarriages(
                    self.printer, [self.rails[self.dual_carriage_axis]],
                    [self.rails[5]], axes=[self.dual_carriage_axis],
                    safe_dist=[dc_config.getfloat(
                        'safe_distance', None, minval=0.)])
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
        # Boundary checks - the Z/A/B limits are owned by the toolhead
        self.toolhead = toolhead
        self.limits = [(1.0, -1.0)] * 5
        max_z_velocity, max_z_accel = toolhead.get_max_z_velocity()
        logging.info("CartesianXY3Z limits: max_z_velocity=%.1f"
                     " max_z_accel=%.1f", max_z_velocity, max_z_accel)
        # Homing/probing state (safety checks are suspended during both)
        self.is_homing = False
        self.is_probing = False
        # Bed geometry safety checks
        self.safety_module = cartesianxy3z_safety.CartesianXY3ZSafety(config)
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        rails = self.rails[:5]
        if self.dc_module:
            dc_rail = self.dc_module.get_primary_rail(self.dual_carriage_axis) \
                    or self.rails[self.dual_carriage_axis]
            rails = (rails[:self.dual_carriage_axis] +
                     [dc_rail] + rails[self.dual_carriage_axis+1:])
        return [stepper_positions[rail.get_name()] for rail in rails]
    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range
    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        for axis_name in homing_axes:
            axis = AXES.index(axis_name)
            rail = None
            if self.dc_module and axis == self.dual_carriage_axis:
                rail = self.dc_module.get_primary_rail(self.dual_carriage_axis)
            if rail is None:
                rail = self.rails[axis]
            self.limits[axis] = rail.get_range()
    def clear_homing_state(self, clear_axes):
        for axis, axis_name in enumerate(AXES):
            if axis_name in clear_axes:
                self.limits[axis] = (1.0, -1.0)
    def set_probing(self, state):
        self.is_probing = state
    # Homing
    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home_vertical_axes_together(self, homing_state):
        # Fast phase: drive Z, A, and B towards their endstops in one
        # coordinated move.  The move stops as soon as any endstop
        # triggers; the axes are then backed off slightly.  Each axis is
        # afterwards homed individually for precision.
        gcode = self.printer.lookup_object('gcode')
        toolhead = self.toolhead
        try:
            # Force the kinematics to a known start position
            startpos = list(toolhead.get_position())
            for axis in VERTICAL_AXES:
                startpos[axis] = self.rails[axis].get_range()[0]
            toolhead.set_position(startpos, homing_axes="zab")
            # Move every vertical axis towards its endstop
            movepos = list(startpos)
            all_endstops = []
            for axis in VERTICAL_AXES:
                rail = self.rails[axis]
                position_min, position_max = rail.get_range()
                if rail.get_homing_info().positive_dir:
                    movepos[axis] = position_max
                else:
                    movepos[axis] = position_min
                all_endstops.extend(rail.get_endstops())
            hmove = HomingMove(self.printer, all_endstops, toolhead=toolhead)
            hmove.homing_move(movepos, COORDINATED_HOME_SPEED,
                              check_triggered=True)
            # Back off from the endstops
            curpos = toolhead.get_position()
            retractpos = list(curpos)
            for axis in VERTICAL_AXES:
                if self.rails[axis].get_homing_info().positive_dir:
                    retractpos[axis] -= COORDINATED_HOME_RETRACT
                else:
                    retractpos[axis] += COORDINATED_HOME_RETRACT
            toolhead.move(retractpos, COORDINATED_HOME_SPEED)
            toolhead.wait_moves()
        except self.printer.command_error as e:
            # Not fatal - the individual homing that follows recovers
            gcode.respond_info(
                "Coordinated Z/A/B fast homing failed: %s" % (str(e),))
    def home(self, homing_state):
        self.is_homing = True
        try:
            self._do_home(homing_state)
        finally:
            self.is_homing = False
    def _do_home(self, homing_state):
        gcode = self.printer.lookup_object('gcode')
        axes = homing_state.get_axes()
        # Home the vertical axes first (so the bed is clear of the nozzle)
        vertical = [axis for axis in VERTICAL_AXES if axis in axes]
        if vertical:
            if set(vertical) == set(VERTICAL_AXES):
                # All three requested - coordinated fast home first
                gcode.respond_info("Starting coordinated Z/A/B homing")
                self.home_vertical_axes_together(homing_state)
            for axis in vertical:
                gcode.respond_info("Homing %s axis" % (AXES[axis].upper(),))
                self.home_axis(homing_state, axis, self.rails[axis])
        # Then home X and Y
        for axis in (0, 1):
            if axis not in axes:
                continue
            if self.dc_module is not None and axis == self.dual_carriage_axis:
                self.dc_module.home(homing_state, self.dual_carriage_axis)
            else:
                self.home_axis(homing_state, axis, self.rails[axis])
    # Move checking
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in range(5):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def _check_safety(self, move):
        if self.safety_module is None or self.is_homing or self.is_probing:
            # The end position of a homing/probing move is a travel
            # limit, not the point the bed will actually stop at
            return
        try:
            is_safe = self.safety_module.check_move(move)
        except Exception as e:
            logging.exception("CartesianXY3Z safety check")
            raise move.move_error("Safety system error: %s" % (str(e),))
        if not is_safe:
            raise move.move_error("Safety check blocked move")
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        vertical_d = [abs(move.axes_d[i]) for i in VERTICAL_AXES]
        if not any(vertical_d):
            # Pure XY move - use defaults
            self._check_safety(move)
            return
        # Move with Z, A, or B - update velocity and accel for the slower
        # vertical axes.  Each axis group runs at its own limit and the
        # slowest group determines the duration of the move.
        self._check_endstops(move)
        self._check_safety(move)
        max_velocity, max_accel = self.toolhead.get_max_velocity()
        max_z_velocity, max_z_accel = self.toolhead.get_max_z_velocity()
        xy_move_d = math.sqrt(move.axes_d[0]**2 + move.axes_d[1]**2)
        vertical_max_d = max(vertical_d)
        t_xy = xy_move_d / max_velocity
        t_vertical = vertical_max_d / max_z_velocity
        effective_v = move.move_d / max(t_xy, t_vertical)
        effective_a = max_z_accel * move.move_d / vertical_max_d
        if xy_move_d:
            effective_a = min(effective_a, max_accel * move.move_d / xy_move_d)
        move.limit_speed(effective_v, effective_a)
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip(AXES, self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CartesianXY3Z(toolhead, config)
