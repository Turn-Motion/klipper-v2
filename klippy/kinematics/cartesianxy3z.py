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
# Direct import of safety module
from . import cartesianxy3z_safety

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
        self.bypass_z_home_together = config.getboolean('bypass_z_home_together', True)
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
        # Setup boundary checks — Z limits are owned by the toolhead
        _, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = toolhead.max_z_velocity
        self.max_z_accel = toolhead.max_z_accel
        # Log loaded velocity/accel limits to klippy.log
        logging.info(
            "CartesianXY3Z limits — max_accel: %.1f | "
            "max_z_velocity: %.1f, max_z_accel: %.1f",
            max_accel, self.max_z_velocity, self.max_z_accel)
        # Also send to Mainsail console once printer is ready
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready_log_limits)
        # Get the post-homing Z movement distance (default 62.8mm)
        self.post_homing_z_lift = config.getfloat('post_homing_z_lift', -60)
        self.limits = [(1.0, -1.0)] * 5  # For x, y, z, a, b axes
        
        # Initialize safety module directly
        self.is_homing = False
        self.is_probing = False
        self.safety_module = None
        if cartesianxy3z_safety is not None:
            try:
                # Log safety module configuration status
                logging.info("CartesianXY3Z: Initializing safety module")
                
                # Option: Use the exact parameter names that safety.py expects
                safety_params = [
                    'x_distance_between_joints_ab',
                    'y_distance_between_ab_center_and_z',
                    'flex_plate_thickness',
                    'printing_area_x_length',
                    'printing_area_y_length',
                    'total_measured_height',
                    'offset_measure_point_1_to_rotation_center',
                    'offset_measure_point_2_to_plate_surface',
                    'x_railspace_ab_outwards_neutral',      
                    'y_railspace_z_inwards_neutral'
                ]
                
                # Check each parameter and log whether it's using default or config value
                found_params = []
                default_params = []
                for param in safety_params:
                    # Use sentinel to check if parameter exists in config
                    sentinel_obj = object()
                    val = config.get(param, sentinel_obj, note_valid=False)
                    if val is not sentinel_obj:
                        found_params.append(param)
                    else:
                        default_params.append(param)
                
                if found_params:
                    logging.info("CartesianXY3Z Safety: Found %d parameters in config: %s", 
                                len(found_params), ", ".join(found_params))
                
                if default_params:
                    logging.info("CartesianXY3Z Safety: Using defaults for %d parameters: %s", 
                                len(default_params), ", ".join(default_params))
                
                # Create safety module with config
                self.safety_module = cartesianxy3z_safety.CartesianXY3ZSafety(config)
                logging.info("CartesianXY3Z: Safety module loaded successfully")
                
            except Exception as e:
                logging.warning("CartesianXY3Z: Failed to initialize safety module: %s", str(e))
                self.safety_module = None

    def _handle_ready_log_limits(self):
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_info(
            "CartesianXY3Z axis limits loaded from config:\n"
            "  max_z_velocity: %.1f  max_z_accel: %.1f"
            % (self.max_z_velocity, self.max_z_accel))

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
        #gcode.respond_info(f"DEBUG: {axis_name} Endstops: {endstops}")
        for es, name in endstops:
            #gcode.respond_info(f"DEBUG: {axis_name} Endstop '{name}' steppers: {es.get_steppers()}")
            pass
        
        # Safely get position_endstop
        try:
            homepos[axis] = hi.position_endstop
            #gcode.respond_info(f"DEBUG: {axis_name} using position_endstop={hi.position_endstop}")
        except AttributeError:
            # If position_endstop is not available, use position_max for Z axes and position_min for X/Y
            if axis >= 2:  # Z, A, B axes
                homepos[axis] = position_max
            else:  # X, Y axes
                homepos[axis] = position_min
        
        forcepos = list(homepos)
        
        # Calculate forcepos based on homing direction
        if hasattr(hi, 'positive_dir') and hi.positive_dir is not None:
            #gcode.respond_info(f"DEBUG: {axis_name} using positive_dir={hi.positive_dir}")
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
                #gcode.respond_info(f"DEBUG: {axis_name} homing in positive direction")
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
                #gcode.respond_info(f"DEBUG: {axis_name} homing in negative direction")
        else:
            # Fallback logic for determining homing direction
            #gcode.respond_info(f"DEBUG: {axis_name} no positive_dir attribute")
            if axis < 2:  # X/Y axes
                # Typically, X/Y home toward their min positions
                forcepos[axis] = position_min - 5.0  # Move slightly beyond expected trigger point
                #gcode.respond_info(f"DEBUG: {axis_name} using fallback negative direction")
            else:  # Z, A, B axes
                forcepos[axis] = position_max  # Default to moving to max position for Z axes
                #gcode.respond_info(f"DEBUG: {axis_name} using fallback forcepos={forcepos[axis]}")

        # Check what speed will be used
        homing_speed = getattr(hi, 'speed', 5.0)  # Default to 5mm/s if not specified
        second_homing_speed = getattr(hi, 'second_homing_speed', homing_speed/2.0)
        retract_dist = getattr(hi, 'retract_dist', 5.0)
        retract_speed = getattr(hi, 'retract_speed', homing_speed/2.0)
        
       
        try:
            homing_state.home_rails([rail], forcepos, homepos)
            #gcode.respond_info(f"DEBUG: {axis_name} home_rails completed successfully")
            
            # Check if endstop was triggered during homing
            for stepper_name, trigger_pos in homing_state.trigger_mcu_pos.items():
                if any(stepper_name == s.get_name() for es, _ in endstops for s in es.get_steppers()):
                    #gcode.respond_info(f"DEBUG: {axis_name} Endstop triggered for {stepper_name} at pos={trigger_pos}")
                    pass
        
        except Exception as e:
            #gcode.respond_info(f"DEBUG: {axis_name} home_rails FAILED with error: {str(e)}")
            pass

    def home_z_axes_together(self, homing_state):
        # This method performs coordinated force move for all Z/A/B axes together
        # When any endstop triggers, that axis stops and retracts slightly
        # Then waits for other axes to complete their fast homing
        
        gcode = self.printer.lookup_object('gcode')
        toolhead = homing_state.toolhead
        z_axes = [2, 3, 4]  # z, a, b
        
        #gcode.respond_info("DEBUG: Starting home_z_axes_together")
        
        try:
            # Phase 1: Force move all Z/A/B axes together until endstops trigger
            # This is the fast homing phase where all axes move simultaneously
            
            # Reset positions for homing using position_min from each rail
            homing_axes_str = 'zab'
            z_min, _ = self.rails[2].get_range()  # Z axis
            a_min, _ = self.rails[3].get_range()  # A axis
            b_min, _ = self.rails[4].get_range()  # B axis
            
            # Set position to position_min for Z, A, B axes
            toolhead.set_position([0., 0., z_min, a_min, b_min] + toolhead.get_position()[5:], homing_axes=homing_axes_str)
            
            # Calculate force move positions for all Z axes
            current_pos = toolhead.get_position()
            
            # Create individual homing moves for each axis to allow independent stopping
            axis_states = {}  # Track which axes are still moving
            
            for axis in z_axes:
                rail = self.rails[axis]
                hi = rail.get_homing_info()
                axis_name = 'xyzab'[axis]
                
                # Get position_min and position_max for this axis
                position_min, position_max = rail.get_range()
                
                # Determine movement direction
                if getattr(hi, 'positive_dir', False):
                    # Use position_max for force move distance
                    target_pos = position_max
                    direction = "positive"
                else:
                    # Use position_min for force move distance in negative direction
                    target_pos = position_min
                    direction = "negative"
                
                axis_states[axis] = {
                    'rail': rail,
                    'target_pos': target_pos,
                    'direction': direction,
                    'completed': False,
                    'endstops': rail.get_endstops()
                }
                
                #gcode.respond_info(f"DEBUG: Axis {axis_name} will force move in {direction} direction")
            
            # Start coordinated force move using HomingMove with all endstops
            all_endstops = []
            for axis_info in axis_states.values():
                all_endstops.extend(axis_info['endstops'])
            
            # Create target position for coordinated move
            move_pos = list(current_pos)
            for axis, info in axis_states.items():
                move_pos[axis] = info['target_pos']
            
            #gcode.respond_info("DEBUG: Starting coordinated force move for all Z/A/B axes")
            fast_speed = 15.0  # Fast force move speed
            
            # Use HomingMove to coordinate all axes - when ANY endstop triggers, ALL axes stop
            hmove = HomingMove(self.printer, all_endstops, toolhead=toolhead)
            hmove.homing_move(move_pos, fast_speed, check_triggered=True)
            
            #gcode.respond_info("DEBUG: Coordinated force move completed - at least one endstop triggered")
            
            # Phase 2: Retract all axes slightly from endstops
            retract_distance = 2.0  # Small retraction to back off endstops
            current_pos = toolhead.get_position()
            retract_pos = list(current_pos)
            
            for axis in z_axes:
                rail = self.rails[axis]
                hi = rail.get_homing_info()
                axis_name = 'xyzab'[axis]
                
                # Retract in opposite direction of homing
                if getattr(hi, 'positive_dir', False):
                    retract_pos[axis] = current_pos[axis] - retract_distance
                else:
                    retract_pos[axis] = current_pos[axis] + retract_distance
                
                #gcode.respond_info(f"DEBUG: Retracting {axis_name} axis by {retract_distance}mm")
            
            # Perform retraction move
            toolhead.move(retract_pos, fast_speed)
            toolhead.wait_moves()
            #gcode.respond_info("DEBUG: All Z/A/B axes retracted from endstops")
            #gcode.respond_info("DEBUG: Coordinated z homing fast homing completed")
            
        except Exception as e:
            gcode.respond_info(f"ERROR: Coordinated z homing fast homing failed: {str(e)}")
            #gcode.respond_info("DEBUG: Will proceed with individual axis homing")
            # Don't raise exception - let individual homing handle recovery
    
    def set_probing(self, state):
        self.is_probing = state

    def home(self, homing_state):
        self.is_homing = True
        try:
            self._do_home(homing_state)
        finally:
            self.is_homing = False

    def _do_home(self, homing_state):
        gcode = self.printer.lookup_object('gcode')
        axes = homing_state.get_axes()
        z_axes_homed = False
        
        # Check if any Z axes need to be homed
        z_axes_to_home = []
        for axis_name in ['z', 'a', 'b']:
            axis = "xyzab".index(axis_name)
            if axis in axes:
                z_axes_to_home.append(axis)
                z_axes_homed = True
        
        # Handle Z/A/B axes homing
        if z_axes_to_home:
            # Only use coordinated homing if ALL Z, A, B axes are being homed together
            # This happens with G28 (all axes) or G28 Z A B (explicit all Z axes)
            all_z_axes = [2, 3, 4]  # z, a, b
            
            if set(z_axes_to_home) != set(all_z_axes):
                #gcode.respond_info("DEBUG: Using individual homing due to not all Z axes being homed")
                
                # Home each Z axis individually
                for axis in z_axes_to_home:
                    axis_name = 'xyzab'[axis]
                    gcode.respond_info(f"Starting individual homing for {axis_name}-axis")
                    self.home_axis(homing_state, axis, self.rails[axis])
                    gcode.respond_info(f"Completed individual homing for {axis_name}-axis")
            else:
                # Use coordinated homing followed by individual precision homing
                gcode.respond_info("Starting coordinated z-together homing fast homing sequence")
                self.home_z_axes_together(homing_state)
                gcode.respond_info("Completed coordinated z-together homing fast homing")
                
                # Now do individual homing for precision (Z, then A, then B)
                for axis in [2, 3, 4]:  # z, a, b in order
                    if axis in z_axes_to_home:
                        axis_name = 'xyzab'[axis]
                        gcode.respond_info(f"Starting individual precision homing for {axis_name}-axis")
                        self.home_axis(homing_state, axis, self.rails[axis])
                        gcode.respond_info(f"Completed individual precision homing for {axis_name}-axis")
        
        # Perform post-homing Z lift if Z was homed (only lift Z axis by 62.8mm)
        #if 2 in z_axes_to_home:  # Only if Z axis was homed
         #   toolhead = homing_state.toolhead
          #  current_pos = toolhead.get_position()
           # new_pos = list(current_pos)
            
            # Apply the lift only to Z axis (index 2) - 62.8mm away from endstop
            #new_pos[2] += self.post_homing_z_lift
            
            #gcode.respond_info(f"DEBUG: Moving Z axis away from endstop by {self.post_homing_z_lift}mm")
            #try:
            #    toolhead.move(new_pos, self.max_z_velocity)
                #toolhead.wait_moves()
                #gcode.respond_info("DEBUG: Post-homing Z lift completed")
                #new_pos[2] = 445.0  # Set Z to 445mm
                #toolhead.set_position(new_pos)
                #gcode.respond_info("DEBUG: Setting kinematic position Z=445")
            #except Exception as e:
            #    gcode.respond_info(f"ERROR: Post-homing Z movement failed: {str(e)}")
        
        # Home X axis
        if 0 in axes:
            if self.dc_module is not None and 0 == self.dual_carriage_axis:
                self.dc_module.home(homing_state)
            else:
                self.home_axis(homing_state, 0, self.rails[0])
        
        # Home Y axis
        if 1 in axes:
            #gcode.respond_info("DEBUG: Starting Y-axis homing")
            if self.dc_module is not None and 1 == self.dual_carriage_axis:
                #gcode.respond_info("DEBUG: Using dual_carriage module for Y-axis homing")
                self.dc_module.home(homing_state)
            else:
                #gcode.respond_info("DEBUG: Using standard homing for Y-axis")
                self.home_axis(homing_state, 1, self.rails[1])
                
        #gcode.respond_info("DEBUG: Home method completed")

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
            # Pure XY move - still check safety for Y restriction
            if self.safety_module is not None and not self.is_homing \
                    and not self.is_probing:
                is_safe = self.safety_module.check_move(move)
                if not is_safe:
                    raise move.move_error("Safety check blocked move")
            return

        # Move with Z, A, or B - update velocity and accel for slower axes
        self._check_endstops(move)

        # Skip safety check during homing or probing - end position is the
        # probe travel limit, not the actual stop point (load cell triggers early)
        if self.safety_module is not None and not self.is_homing \
                and not self.is_probing:
            try:
                is_safe = self.safety_module.check_move(move)
            except Exception as e:
                gcode = self.printer.lookup_object('gcode')
                gcode.respond_info(
                    "CartesianXY3Z: Safety system error: %s" % (str(e),))
                logging.error(
                    "CartesianXY3Z: Safety check exception: %s", str(e))
                raise move.move_error(
                    "Safety system error: %s" % (str(e),))
            if not is_safe:
                raise move.move_error("Safety check blocked move")
        
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