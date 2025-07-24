# Code for handling the kinematics of cartesian robots with X, Y, Z, A, B axes
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
# Import HomingMove directly
from extras.homing import HomingMove
from . import idex_modes

class CartesianXY3Z:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis rails
        self.dual_carriage_axis = None
        self.dual_carriage_rails = []
        # Initialize rails for x, y, z, a, b axes
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzab']
        for rail, axis in zip(self.rails, 'xyzab'):
            rail.setup_itersolve('cartesianxy3z_stepper_alloc', axis.encode())
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        # Option to bypass Z home together for debugging
        self.bypass_z_home_together = config.getboolean('bypass_z_home_together', False)
        self.dc_module = None
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            dc_axis = dc_config.getchoice('axis', ['x', 'y'])
            self.dual_carriage_axis = {'x': 0, 'y': 1}[dc_axis]
            # setup second dual carriage rail
            self.rails.append(stepper.LookupMultiRail(dc_config))
            self.rails[5].setup_itersolve('cartesianxy3z_stepper_alloc',
                                          dc_axis.encode())
            dc_rail_0 = idex_modes.DualCarriagesRail(
                    self.rails[self.dual_carriage_axis],
                    axis=self.dual_carriage_axis, active=True)
            dc_rail_1 = idex_modes.DualCarriagesRail(
                    self.rails[5], axis=self.dual_carriage_axis, active=False)
            self.dc_module = idex_modes.DualCarriages(
                    dc_config, dc_rail_0, dc_rail_1,
                    axis=self.dual_carriage_axis)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        # Configure max velocity and acceleration for Z, A, and B axes
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.max_a_velocity = config.getfloat('max_a_velocity', self.max_z_velocity,
                                              above=0., maxval=max_velocity)
        self.max_a_accel = config.getfloat('max_a_accel', self.max_z_accel,
                                           above=0., maxval=max_accel)
        self.max_b_velocity = config.getfloat('max_b_velocity', self.max_z_velocity,
                                              above=0., maxval=max_velocity)
        self.max_b_accel = config.getfloat('max_b_accel', self.max_z_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 5  # For x, y, z, a, b axes

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
                self.limits[axis] = (1.0, -1.0)

    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None, None]  # x, y, z, a, b
        
        # Get reference to gcode for logging
        gcode = self.printer.lookup_object('gcode')
        axis_name = "xyzab"[axis]
        
        # Debug endstop configuration
        endstops = rail.get_endstops()
        gcode.respond_info(f"DEBUG: {axis_name} Endstops: {endstops}")
        for es, name in endstops:
            gcode.respond_info(f"DEBUG: {axis_name} Endstop '{name}' steppers: {es.get_steppers()}")
        
        # Safely get position_endstop
        try:
            homepos[axis] = hi.position_endstop
            gcode.respond_info(f"DEBUG: {axis_name} using position_endstop={hi.position_endstop}")
        except AttributeError:
            # If position_endstop is not available, use position_max for Z axes and position_min for X/Y
            if axis >= 2:  # Z, A, B axes
                homepos[axis] = position_max
            else:  # X, Y axes
                homepos[axis] = position_min
        
        forcepos = list(homepos)
        
        # Calculate forcepos based on homing direction
        if hasattr(hi, 'positive_dir') and hi.positive_dir is not None:
            gcode.respond_info(f"DEBUG: {axis_name} using positive_dir={hi.positive_dir}")
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
                gcode.respond_info(f"DEBUG: {axis_name} homing in positive direction")
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
                gcode.respond_info(f"DEBUG: {axis_name} homing in negative direction")
        else:
            # Fallback logic for determining homing direction
            gcode.respond_info(f"DEBUG: {axis_name} no positive_dir attribute")
            if axis < 2:  # X/Y axes
                # Typically, X/Y home toward their min positions
                forcepos[axis] = position_min - 5.0  # Move slightly beyond expected trigger point
                gcode.respond_info(f"DEBUG: {axis_name} using fallback negative direction")
            else:  # Z, A, B axes
                forcepos[axis] = position_max  # Default to moving to max position for Z axes
                gcode.respond_info(f"DEBUG: {axis_name} using fallback forcepos={forcepos[axis]}")

        
        # Perform homing
        gcode.respond_info(f"DEBUG: {axis_name} Starting home_rails with forcepos={forcepos}, homepos={homepos}")
        
        # Check what speed will be used
        homing_speed = getattr(hi, 'speed', 5.0)  # Default to 5mm/s if not specified
        second_homing_speed = getattr(hi, 'second_homing_speed', homing_speed/2.0)
        retract_dist = getattr(hi, 'retract_dist', 5.0)
        retract_speed = getattr(hi, 'retract_speed', homing_speed/2.0)
        
        gcode.respond_info(f"DEBUG: {axis_name} Homing speeds: initial={homing_speed}, second={second_homing_speed}")
        gcode.respond_info(f"DEBUG: {axis_name} Retract: dist={retract_dist}, speed={retract_speed}")
        
        try:
            homing_state.home_rails([rail], forcepos, homepos)
            gcode.respond_info(f"DEBUG: {axis_name} home_rails completed successfully")
            
            # Check if endstop was triggered during homing
            for stepper_name, trigger_pos in homing_state.trigger_mcu_pos.items():
                if any(stepper_name == s.get_name() for es, _ in endstops for s in es.get_steppers()):
                    gcode.respond_info(f"DEBUG: {axis_name} Endstop triggered for {stepper_name} at pos={trigger_pos}")
        
        except Exception as e:
            gcode.respond_info(f"DEBUG: {axis_name} home_rails FAILED with error: {str(e)}")

    def home_z_axes_together(self, homing_state):
        # Get the Z, A, B rails if they need to be homed
        z_rails = []
        z_axes = []
        for axis_name in ['z', 'a', 'b']:
            axis = "xyzab".index(axis_name)
            if axis in homing_state.get_axes():
                z_rails.append(self.rails[axis])
                z_axes.append(axis)
        
        if not z_rails:
            return  # No Z axes to home
        
        # Determine movement for all Z axes together
        forcepos = [None, None, None, None, None]  # x, y, z, a, b
        homepos = [None, None, None, None, None]  # x, y, z, a, b
        
        # Set the target positions for all Z axes
        for i, axis in enumerate(z_axes):
            rail = z_rails[i]
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            
            # All Z axes move in the same direction during initial movement
            if hi.positive_dir:
                forcepos[axis] = position_min + 1.0  # Move to near minimum
            else:
                forcepos[axis] = position_max - 1.0  # Move to near maximum
        
        # Get gcode object for logging
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_info(f"DEBUG: Starting Z/A/B axes initial positioning with rails: {z_rails}")
        
        # Use proper homing motion instead of regular move
        # Create a temporary homepos with all Z axes at the same target position
        homepos_z = [None] * 5  # x, y, z, a, b
        
        # Choose a direction that moves all Z axes safely (use first rail's direction as reference)
        reference_rail = z_rails[0]
        reference_hi = reference_rail.get_homing_info()
        homing_speed = getattr(reference_hi, 'speed', 5.0)  # Use first rail's speed
        
        # Create a dummy forcepoos for coordinated Z movement
        dummy_forcepos = [None] * 5  # x, y, z, a, b
        
        for axis in z_axes:
            min_pos, max_pos = self.rails[axis].get_range()
            # Choose target position based on homing direction of reference rail
            if getattr(reference_hi, 'positive_dir', False):
                dummy_forcepos[axis] = min_pos - 10.0  # Force movement in positive direction
                homepos_z[axis] = min_pos + 5.0  # Target position slightly above min
                gcode.respond_info(f"DEBUG: Axis {axis} moving in positive direction")
            else:
                dummy_forcepos[axis] = max_pos + 10.0  # Force movement in negative direction
                homepos_z[axis] = max_pos - 5.0  # Target position slightly below max
                gcode.respond_info(f"DEBUG: Axis {axis} moving in negative direction")
        
        # Use a proper homing move that will stop when any endstop triggers
        try:
            # Create a combined list of endstops from all z_rails
            all_endstops = []
            for rail in z_rails:
                rail_endstops = rail.get_endstops()
                all_endstops.extend(rail_endstops)
            
            gcode.respond_info(f"DEBUG: Monitoring endstops for Z move: {all_endstops}")
            
            # Create a HomingMove object with all Z endstops
            # Using the HomingMove class we imported at the top of the file
            hmove = HomingMove(self.printer, all_endstops, toolhead=homing_state.toolhead)
            
            # Reset position before move
            toolhead = homing_state.toolhead
            toolhead.set_position([0., 0., 0., 0., 0.] + toolhead.get_position()[5:], homing_axes=[])
            
            # Calculate target position for synchronous movement
            current_pos = toolhead.get_position()
            move_pos = list(current_pos)
            
            # Update target positions for all Z axes to move together
            for axis in z_axes:
                # Use the homepos_z values we calculated above based on homing direction
                move_pos[axis] = homepos_z[axis]
            
            gcode.respond_info(f"DEBUG: Starting coordinated homing move for Z/A/B axes to: {move_pos}")
            
            # Use a proper homing move that will monitor endstops
            # The homing_move method will stop when any endstop triggers
            hmove.homing_move(move_pos, 5.0)  # Slow speed for safety
            
            gcode.respond_info(f"DEBUG: Z/A/B axes coordinated homing move completed")
            
            # Check which endstops were triggered (if any)
            for i, rail in enumerate(z_rails):
                endstops = rail.get_endstops()
                for es, name in endstops:
                    triggered = False
                    for s in es.get_steppers():
                        if s.get_name() in hmove.stepper_positions:
                            triggered = True
                            gcode.respond_info(f"DEBUG: Endstop '{name}' on axis {z_axes[i]} was triggered during coordinated move")
            
            gcode.respond_info(f"DEBUG: Coordinated Z/A/B move completed - ready for individual homing")
        
            
        except Exception as e:
            gcode.respond_info(f"DEBUG: Z/A/B axes initial positioning failed: {str(e)}")
            # Don't throw an error here, we'll try to home axes individually
    
    def home(self, homing_state):
        # Custom homing sequence: Z/A/B together first, then individually, then X, then Y
        axes = homing_state.get_axes()
        
        # Get gcode object for logging
        gcode = self.printer.lookup_object('gcode')
        
        # Debug logging
        gcode.respond_info(f"DEBUG: home method called with axes={axes}")
        
        # First move Z, A, B axes together if any of them need to be homed
        z_axes_to_home = []
        for axis_name in ['z', 'a', 'b']:
            axis = "xyzab".index(axis_name)
            if axis in axes:
                z_axes_to_home.append(axis)
        
        gcode.respond_info(f"DEBUG: z_axes_to_home={z_axes_to_home}")
        
        if z_axes_to_home:
            # Check if we should bypass coordinated Z homing when only one Z-axis is requested
            # and no X/Y axes are involved (allows debugging individual Z/A/B axis endstops)
            single_z_axis_homing = (len(z_axes_to_home) == 1 and len(axes) == 1)
            
            if not single_z_axis_homing:
                # First move Z, A, B axes together to get them roughly aligned
                gcode.respond_info("DEBUG: Starting home_z_axes_together")
                self.home_z_axes_together(homing_state)
                gcode.respond_info("DEBUG: Completed home_z_axes_together")
            else:
                # Show warning when bypassing coordinated homing
                axis_name = "zab"[z_axes_to_home[0] - 2]  # Convert axis index to name (z=2, a=3, b=4)
                gcode.respond_info(
                    f"WARNING: Bypassing coordinated Z/A/B homing for {axis_name.upper()}-axis. "
                    f"This is for debugging purposes only. Make sure your endstops are correctly configured.")
            
            # Then home each Z axis individually in the specified order
            for axis_name in ['z', 'a', 'b']:
                axis = "xyzab".index(axis_name)
                if axis in axes:
                    gcode.respond_info(f"DEBUG: Starting individual homing for {axis_name}-axis")
                    self.home_axis(homing_state, axis, self.rails[axis])
        
        # Then home X if needed
        if 0 in axes:  # X axis
            gcode.respond_info("DEBUG: Starting X-axis homing")
            if self.dc_module is not None and 0 == self.dual_carriage_axis:
                gcode.respond_info("DEBUG: Using dual_carriage module for X-axis homing")
                self.dc_module.home(homing_state)
            else:
                gcode.respond_info("DEBUG: Using standard homing for X-axis")
                self.home_axis(homing_state, 0, self.rails[0])
        
        # Finally home Y if needed
        if 1 in axes:  # Y axis
            gcode.respond_info("DEBUG: Starting Y-axis homing")
            if self.dc_module is not None and 1 == self.dual_carriage_axis:
                gcode.respond_info("DEBUG: Using dual_carriage module for Y-axis homing")
                self.dc_module.home(homing_state)
            else:
                gcode.respond_info("DEBUG: Using standard homing for Y-axis")
                self.home_axis(homing_state, 1, self.rails[1])
                
        gcode.respond_info("DEBUG: Home method completed")

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in range(5):  # Check all axes (x, y, z, a, b)
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        
        # Check if any of Z, A, or B axes are moving
        if not any(move.axes_d[i] for i in [2, 3, 4]):  # z, a, b axes
            # Normal XY move - use defaults
            return

        # Move with Z, A, or B - update velocity and accel for slower axes
        self._check_endstops(move)
        
        # Call external safety module if it exists
        try:
            safety_module = self.printer.lookup_object('cartesianxy3z_safety')
            # The safety module will raise an exception if the move is unsafe
            safety_module.check_move(move)
        except self.printer.config_error:
            # Safety module not loaded, continue without safety checks
            gcode = self.printer.lookup_object('gcode')
            gcode.respond_info("CartesianXY3Z Safety: Module not found - continuing without safety checks")
            logging.info("CartesianXY3Z: Safety module not found, continuing without safety checks") #store info into the klipper log
            pass
        
        # Calculate the slowest axis ratio for Z, A, B
        z_ratio = 1.0
        if move.axes_d[2]:  # Z axis
            z_ratio = min(z_ratio, move.move_d / abs(move.axes_d[2]))
        if move.axes_d[3]:  # A axis
            z_ratio = min(z_ratio, move.move_d / abs(move.axes_d[3]))
        if move.axes_d[4]:  # B axis
            z_ratio = min(z_ratio, move.move_d / abs(move.axes_d[4]))
            
        # Use the slowest axis settings
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyzab", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CartesianXY3Z(toolhead, config) 