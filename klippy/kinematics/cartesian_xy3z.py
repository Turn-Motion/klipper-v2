<<<<<<< HEAD
# Code for handling the kinematics of cartesian robots with additional a and b axes
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2024  Modified for additional axes
=======
# Code for handling the kinematics of cartesian robots with 3 separated z-axis support
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
from . import idex_modes

<<<<<<< HEAD
class CartXY3ZKinematics:
=======
class Cart3ZKinematics:
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis rails
        self.dual_carriage_axis = None
        self.dual_carriage_rails = []
<<<<<<< HEAD
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzab']
        for rail, axis in zip(self.rails, 'xyzab'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
=======
        # CHANGED: Create rails for X, Y, and 3 Z-axis steppers
        self.rails = [
            stepper.LookupMultiRail(config.getsection('stepper_x')),
            stepper.LookupMultiRail(config.getsection('stepper_y')),
            # Three independent Z-axis rails
            stepper.LookupMultiRail(config.getsection('stepper_z')),
            stepper.LookupMultiRail(config.getsection('stepper_a')),
            stepper.LookupMultiRail(config.getsection('stepper_b'))
        ]
        # CHANGED: Configure kinematic solvers for each axis
        axis_names = ['x', 'y', 'z', 'a', 'b']
        for rail, axis in zip(self.rails, axis_names):  # 'z' repeated for 3 Z axes
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
        # Calculate axis limits
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        # Dual carriage handling
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
        self.dc_module = None
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            dc_axis = dc_config.getchoice('axis', ['x', 'y'])
            self.dual_carriage_axis = {'x': 0, 'y': 1}[dc_axis]
            # setup second dual carriage rail
            self.rails.append(stepper.LookupMultiRail(dc_config))
<<<<<<< HEAD
            self.rails[3].setup_itersolve('cartesian_stepper_alloc',
                                          dc_axis.encode())
=======
            # CHANGED: Add dual-carriage rail dynamically
            dual_carriage_rail_index = len(self.rails) - 1
            self.rails[dual_carriage_rail_index].setup_itersolve(
                'cartesian_stepper_alloc', dc_axis.encode())
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
            dc_rail_0 = idex_modes.DualCarriagesRail(
                    self.rails[self.dual_carriage_axis],
                    axis=self.dual_carriage_axis, active=True)
            dc_rail_1 = idex_modes.DualCarriagesRail(
                    self.rails[3], axis=self.dual_carriage_axis, active=False)
            self.dc_module = idex_modes.DualCarriages(
                    dc_config, dc_rail_0, dc_rail_1,
                    axis=self.dual_carriage_axis)
<<<<<<< HEAD
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # Setup boundary checks
=======
        # Register steppers with toolhead            
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # CHANGED: Configure Z-axis motion limits, all 3 z-axes share the same limits
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
<<<<<<< HEAD
        self.limits = [(1.0, -1.0)] * 5
=======
        self.limits = [(1.0, -1.0)] * 5 # CHANGED: 5 axes (X, Y, Z, A, B)
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        rails = self.rails
        if self.dc_module:
            primary_rail = self.dc_module.get_primary_rail().get_rail()
            rails = (rails[:self.dc_module.axis] +
                     [primary_rail] + rails[self.dc_module.axis+1:])
        return [stepper_positions[rail.get_name()] for rail in rails]
    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range
<<<<<<< HEAD
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
        for axis_name in homing_axes:
            axis = "xyzab".index(axis_name)
            if self.dc_module and axis == self.dc_module.axis:
                rail = self.dc_module.get_primary_rail().get_rail()
            else:
                rail = self.rails[axis]
            self.limits[axis] = rail.get_range()
    def clear_homing_state(self, clear_axes):
        for axis, axis_name in enumerate("xyzab"):
            if axis_name in clear_axes:
=======
    #CHANGED: Add according to toucan3D
    def override_rail(self, i, rail):
        self.rails[i] = rail
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            #CHANGED: Modified according to toucan3D
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    #Def by Toucan3D, not used for unconfirmed reason, potentially to replace clear_homing_state
    #def note_z_not_homed(self):
    # Helper for Safe Z Home
    #    self.limits[2] = (1.0, -1.0)
    def clear_homing_state(self, clear_axes):
        #CHANGED: to adapt to new axis number
        axis_map = {
        'x': 0,
        'y': 1,
        'z': 2,
        'a': 3,  # Your additional A axis (was Z1)
        'b': 4   # Your additional B axis (was Z2)
        }
        for axis_name in clear_axes:
            axis = axis_map.get(axis_name)
            if axis is not None:
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
                self.limits[axis] = (1.0, -1.0)
    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
<<<<<<< HEAD
        homepos = [None, None, None, None, None]
=======
        homepos = [None, None, None, None]
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            if self.dc_module is not None and axis == self.dual_carriage_axis:
                self.dc_module.home(homing_state)
            else:
                self.home_axis(homing_state, axis, self.rails[axis])
    def _check_endstops(self, move):
        end_pos = move.end_pos
<<<<<<< HEAD
        for i in (0, 1, 2):
=======
        for i in (0, 1, 2, 3, 4): #CHANGED: Add axes
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
<<<<<<< HEAD
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        
        # CHANGED Check if any of the slower axes (z, a, b) are moving 
        if any(move.axes_d[i] for i in (2, 3, 4)):  # Check z, a, b axes
            self._check_endstops(move)
            # Apply speed limits for each moving axis
            for axis in (2, 3, 4):  # z, a, b axes
                if move.axes_d[axis]:
                    ratio = move.move_d / abs(move.axes_d[axis])
                    move.limit_speed(
                        self.max_z_velocity * ratio, 
                        self.max_z_accel * ratio)
            return
        
        # Normal XY move - use defaults
        return
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyzab", self.limits) if l <= h]
=======
        #Check XY boundaries
        z_movement = any(move.axes_d[i] for i in [2,3,4])
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        #CHANGED: Check if any Z-axis is moving
        if not z_movement:
            return  # Normal XY move
        # Move with Z - update velocity and accel for slower Z axis (CH: need recheck)
        max_z_move = max(abs(move.axes_d[i]) for i in [2,3,4])
        z_ratio = move.move_d / max_z_move
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def get_status(self, eventtime):
        axis_names = ['x', 'y', 'z', 'a', 'b']  # Explicit 5-axis list
        axes = [a for a, (l, h) in zip(axis_names, self.limits) if l <= h]
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
<<<<<<< HEAD
    return CartXY3ZKinematics(toolhead, config)
=======
    return Cart3ZKinematics(toolhead, config)
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
