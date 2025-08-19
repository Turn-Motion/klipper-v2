# Mechanical bed tilt calibration with multiple Z steppers
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import mathutil
from . import probe

class ZAdjustHelper:
    def __init__(self, config, z_count):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.z_count = z_count
        self.z_steppers = []
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
    def handle_connect(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        gcode = self.printer.lookup_object('gcode')
        logging.info("Z_TILT DEBUG: Kinematics type: %s", type(kin).__name__)
        gcode.respond_info("Z-Tilt: Kinematics type: %s" % type(kin).__name__)
        
        # Get all steppers and log their details
        all_steppers = kin.get_steppers()
        logging.info("Z_TILT DEBUG: Total steppers found: %d", len(all_steppers))
        gcode.respond_info("Z-Tilt: Total steppers found: %d" % len(all_steppers))
        for i, s in enumerate(all_steppers):
            logging.info("Z_TILT DEBUG: Stepper %d: name=%s, z=%s, a=%s, b=%s", 
                        i, s.get_name(), s.is_active_axis('z'), 
                        s.is_active_axis('a'), s.is_active_axis('b'))
        
        # Original implementation (commented out)
        # Find all Z-axis steppers (including those named stepper_a and stepper_b)
        # z_steppers = [s for s in kin.get_steppers() if s.is_active_axis('z')]
        
        # TEMPORARY FIX: Find Z-axis steppers by name and axis flags
        z_steppers = []
        for s in kin.get_steppers():
            # Check if stepper name indicates it's a Z axis stepper
            name = s.get_name()
            if name == 'stepper_z' or s.is_active_axis('z'):
                logging.info("Z_TILT DEBUG: Adding Z stepper: %s", name)
                z_steppers.append(s)
        
        logging.info("Z_TILT DEBUG: Found %d Z-axis steppers", len(z_steppers))
        
        # ---- Turn V2 ----
        # For CartesianXY3Z kinematics, also include A and B axis steppers, this is a particular fix for TurnV2
        # since they physically move in the Z direction
        if hasattr(kin, 'rails') and len(kin.rails) >= 5:  # CartesianXY3Z has 5 rails: x,y,z,a,b
            logging.info("Z_TILT DEBUG: CartesianXY3Z detected with %d rails", len(kin.rails))
            gcode.respond_info("Z-Tilt: CartesianXY3Z kinematics detected with %d rails" % len(kin.rails))
            
            # Original implementation (commented out)
            # a_steppers = [s for s in kin.get_steppers() if s.is_active_axis('a')]
            # for i, s in enumerate(a_steppers):
            #    logging.info("Z_TILT DEBUG: A-stepper %d: %s", i, s.get_name())
            
            # TEMPORARY FIX: Find A-axis steppers by name
            a_steppers = []
            for s in kin.get_steppers():
                name = s.get_name()
                if name == 'stepper_a' or s.is_active_axis('a'):
                    logging.info("Z_TILT DEBUG: Adding A stepper: %s", name)
                    a_steppers.append(s)
            
            logging.info("Z_TILT DEBUG: Found %d A-axis steppers", len(a_steppers))
            
            # Original implementation (commented out)
            # b_steppers = [s for s in kin.get_steppers() if s.is_active_axis('b')]
            # for i, s in enumerate(b_steppers):
            #    logging.info("Z_TILT DEBUG: B-stepper %d: %s", i, s.get_name())
            
            # TEMPORARY FIX: Find B-axis steppers by name
            b_steppers = []
            for s in kin.get_steppers():
                name = s.get_name()
                if name == 'stepper_b' or s.is_active_axis('b'):
                    logging.info("Z_TILT DEBUG: Adding B stepper: %s", name)
                    b_steppers.append(s)
            
            logging.info("Z_TILT DEBUG: Found %d B-axis steppers", len(b_steppers))
            
            # Add A and B steppers to Z steppers list
            z_steppers.extend(a_steppers)
            z_steppers.extend(b_steppers)
            logging.info("Z_TILT DEBUG: After adding A+B, total Z steppers: %d", len(z_steppers))
            gcode.respond_info("Z-Tilt: Using %d Z steppers (including A and B axes)" % len(z_steppers))
        else:
            logging.info("Z_TILT DEBUG: Not CartesianXY3Z or missing rails attribute")
        # ---- Turn V2 ----
        
        if len(z_steppers) != self.z_count:
            raise self.printer.config_error(
                "%s z_positions needs exactly %d items" % (
                    self.name, len(z_steppers)))
        if len(z_steppers) < 2:
            raise self.printer.config_error(
                "%s requires multiple z steppers" % (self.name,))
        
        # Log the names of the Z steppers found
        stepper_names = [s.get_name() for s in z_steppers]
        logging.info("Z-Tilt using Z steppers: %s", stepper_names)
        gcode.respond_info("Z-Tilt using Z steppers: %s" % stepper_names)
        
        self.z_steppers = z_steppers
    def adjust_steppers(self, adjustments, speed):
        toolhead = self.printer.lookup_object('toolhead')
        gcode = self.printer.lookup_object('gcode')
        curpos = toolhead.get_position()
        
        logging.info("Z_TILT DEBUG: adjust_steppers called with adjustments=%s, speed=%.3f", adjustments, speed)
        logging.info("Z_TILT DEBUG: Current position before adjustment: X=%.3f Y=%.3f Z=%.3f", 
                   curpos[0], curpos[1], curpos[2])
        # Report on movements
        stepstrs = ["%s = %.6f" % (s.get_name(), a)
                    for s, a in zip(self.z_steppers, adjustments)]
        msg = "Making the following Z adjustments:\n%s" % ("\n".join(stepstrs),)
        gcode.respond_info(msg)
        
        # Log detailed stepper information
        logging.info("Z_TILT DEBUG: Detailed stepper adjustments:")
        for i, (stepper, adj) in enumerate(zip(self.z_steppers, adjustments)):
            logging.info("Z_TILT DEBUG:   Stepper %d: %s, adjustment=%.6f", 
                       i, stepper.get_name(), adj)
        # Disable Z stepper movements
        toolhead.flush_step_generation()
        for s in self.z_steppers:
            s.set_trapq(None)
        # Move each z stepper (sorted from lowest to highest) until they match
        positions = [(-a, s) for a, s in zip(adjustments, self.z_steppers)]
        positions.sort(key=(lambda k: k[0]))
        
        logging.info("Z_TILT DEBUG: Sorted stepper positions for movement:")
        for i, (offset, stepper) in enumerate(positions):
            logging.info("Z_TILT DEBUG:   %d: %s, offset=%.6f", 
                       i, stepper.get_name(), offset)
        first_stepper_offset, first_stepper = positions[0]
        z_low = curpos[2] - first_stepper_offset
        
        logging.info("Z_TILT DEBUG: Base Z calculation: first_stepper=%s, offset=%.6f, z_low=%.6f", 
                   first_stepper.get_name(), first_stepper_offset, z_low)
        for i in range(len(positions)-1):
            stepper_offset, stepper = positions[i]
            next_stepper_offset, next_stepper = positions[i+1]
            
            logging.info("Z_TILT DEBUG: Processing stepper pair %d: %s (offset=%.6f) -> %s (offset=%.6f)", 
                       i, stepper.get_name(), stepper_offset, next_stepper.get_name(), next_stepper_offset)
            toolhead.flush_step_generation()
            stepper.set_trapq(toolhead.get_trapq())
            curpos[2] = z_low + next_stepper_offset
            
            logging.info("Z_TILT DEBUG: Setting stepper %s trapq and moving to Z=%.6f", 
                       stepper.get_name(), curpos[2])
            try:
                logging.info("Z_TILT DEBUG: Executing move to Z=%.6f at speed=%.3f", curpos[2], speed)
                toolhead.move(curpos, speed)
                toolhead.set_position(curpos)
                logging.info("Z_TILT DEBUG: Move completed successfully")
            except Exception as e:
                logging.exception("ZAdjustHelper adjust_steppers")
                logging.info("Z_TILT DEBUG: Move failed with error: %s", str(e))
                toolhead.flush_step_generation()
                for s in self.z_steppers:
                    s.set_trapq(toolhead.get_trapq())
                raise
        # Z should now be level - do final cleanup
        last_stepper_offset, last_stepper = positions[-1]
        logging.info("Z_TILT DEBUG: Final cleanup - last stepper: %s, offset=%.6f", 
                   last_stepper.get_name(), last_stepper_offset)
        toolhead.flush_step_generation()
        last_stepper.set_trapq(toolhead.get_trapq())
        curpos[2] += first_stepper_offset
        logging.info("Z_TILT DEBUG: Final position set to Z=%.6f", curpos[2])
        toolhead.set_position(curpos)

class ZAdjustStatus:
    def __init__(self, printer):
        self.applied = False
        printer.register_event_handler("stepper_enable:motor_off",
                                        self._motor_off)
    def check_retry_result(self, retry_result):
        if retry_result == "done":
            self.applied = True
        return retry_result
    def reset(self):
        self.applied = False
    def get_status(self, eventtime):
        return {'applied': self.applied}
    def _motor_off(self, print_time):
        self.reset()

class RetryHelper:
    def __init__(self, config, error_msg_extra = ""):
        self.gcode = config.get_printer().lookup_object('gcode')
        self.default_max_retries = config.getint("retries", 0, minval=0)
        self.default_retry_tolerance = \
            config.getfloat("retry_tolerance", 0., above=0.)
        self.value_label = "Probed points range"
        self.error_msg_extra = error_msg_extra
        self.debug = False
        
        # Log configuration
        if self.default_max_retries > 0:
            self.gcode.respond_info(
                "Z-Tilt: Configured with %d retries and %.6f tolerance" % 
                (self.default_max_retries, self.default_retry_tolerance))
    def start(self, gcmd):
        self.max_retries = gcmd.get_int('RETRIES', self.default_max_retries,
                                        minval=0, maxval=30)
        self.retry_tolerance = gcmd.get_float('RETRY_TOLERANCE',
                                              self.default_retry_tolerance,
                                              minval=0.0, maxval=1.0)
        self.current_retry = 0
        self.previous = None
        self.increasing = 0
    def check_increase(self, error):
        if self.previous and error > self.previous + 0.0000001:
            self.increasing += 1
        elif self.increasing > 0:
            self.increasing -= 1
        self.previous = error
        return self.increasing > 1
    def check_retry(self, z_positions):
        if self.max_retries == 0:
            return "done"
        error = round(max(z_positions) - min(z_positions),6)
        self.gcode.respond_info(
            "Retries: %d/%d %s: %0.6f tolerance: %0.6f" % (
                self.current_retry, self.max_retries, self.value_label,
                error, self.retry_tolerance))
        if self.check_increase(error):
            raise self.gcode.error("Retries aborting: %s is increasing. %s"
                                   % (self.value_label, self.error_msg_extra))
        if error <= self.retry_tolerance:
            return "done"
        self.current_retry += 1
        if self.current_retry > self.max_retries:
            raise self.gcode.error("Too many retries")
        return "retry"

class ZTilt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.z_positions = config.getlists('z_positions', seps=(',', '\n'),
                                           parser=float, count=2)
        self.retry_helper = RetryHelper(config)
        self.probe_helper = probe.ProbePointsHelper(config, self.probe_finalize)
        self.probe_helper.minimum_points(2)
        self.z_status = ZAdjustStatus(self.printer)
        self.z_helper = ZAdjustHelper(config, len(self.z_positions))
        # Register Z_TILT_ADJUST command
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('Z_TILT_ADJUST', self.cmd_Z_TILT_ADJUST,
                               desc=self.cmd_Z_TILT_ADJUST_help)
    cmd_Z_TILT_ADJUST_help = "Adjust the Z tilt"
    def cmd_Z_TILT_ADJUST(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        self.z_status.reset()
        self.retry_helper.start(gcmd)
        gcode.respond_info("Starting Z-Tilt adjustment...")
        self.probe_helper.start_probe(gcmd)
    def probe_finalize(self, offsets, positions):
        # Setup for coordinate descent analysis
        z_offset = offsets[2]
        gcode = self.printer.lookup_object('gcode')
        logging.info("Z_TILT DEBUG: Probe finalize called with offsets=%s, positions=%s", offsets, positions)
        logging.info("Calculating bed tilt with: %s", positions)
        gcode.respond_info("Z-Tilt: Calculating bed tilt with probed points")
        params = { 'x_adjust': 0., 'y_adjust': 0., 'z_adjust': z_offset }
        logging.info("Z_TILT DEBUG: Initial parameters: x_adjust=%.6f, y_adjust=%.6f, z_adjust=%.6f", 
                   params['x_adjust'], params['y_adjust'], params['z_adjust'])
        
        # Perform coordinate descent
        def adjusted_height(pos, params):
            x, y, z = pos
            result = (z - x*params['x_adjust'] - y*params['y_adjust'] - params['z_adjust'])
            return result
        
        def errorfunc(params):
            total_error = 0.
            errors = []
            for pos in positions:
                adj_height = adjusted_height(pos, params)
                error = adj_height**2
                total_error += error
                errors.append((pos, adj_height, error))
            
            # Log detailed error calculations periodically (not every iteration to avoid log flooding)
            if self.retry_helper.debug and hasattr(errorfunc, 'call_count'):
                errorfunc.call_count += 1
                if errorfunc.call_count % 10 == 0:  # Log every 10th call
                    logging.info("Z_TILT DEBUG: Error calculation iteration %d", errorfunc.call_count)
                    for i, (pos, adj_height, err) in enumerate(errors):
                        logging.info("Z_TILT DEBUG:   Point %d (%.3f,%.3f,%.3f): adjusted_height=%.6f, error=%.6f", 
                                    i, pos[0], pos[1], pos[2], adj_height, err)
                    logging.info("Z_TILT DEBUG:   Total error: %.6f", total_error)
            return total_error
        
        # Initialize call counter for error function
        errorfunc.call_count = 0
        
        logging.info("Z_TILT DEBUG: Starting coordinate descent calculation")
        gcode.respond_info("Z-Tilt: Calculating optimal adjustments...")
        new_params = mathutil.coordinate_descent(
            params.keys(), params, errorfunc)
        logging.info("Z_TILT DEBUG: Coordinate descent completed after %d iterations", errorfunc.call_count)
        gcode.respond_info("Z-Tilt: Calculation completed after %d iterations" % errorfunc.call_count)
        
        # Apply results
        speed = self.probe_helper.get_lift_speed()
        logging.info("Calculated bed tilt parameters: %s", new_params)
        logging.info("Z_TILT DEBUG: Final parameters: x_adjust=%.6f, y_adjust=%.6f, z_adjust=%.6f", 
                   new_params['x_adjust'], new_params['y_adjust'], new_params['z_adjust'])
        gcode.respond_info("Z-Tilt: Calculated parameters: x_adjust=%.6f, y_adjust=%.6f, z_adjust=%.6f" % 
                        (new_params['x_adjust'], new_params['y_adjust'], new_params['z_adjust']))
        
        x_adjust = new_params['x_adjust']
        y_adjust = new_params['y_adjust']
        z_adjust = (new_params['z_adjust'] - z_offset
                    - x_adjust * offsets[0] - y_adjust * offsets[1])
        
        logging.info("Z_TILT DEBUG: Calculated final adjustments: x_adjust=%.6f, y_adjust=%.6f, z_adjust=%.6f", 
                   x_adjust, y_adjust, z_adjust)
        logging.info("Z_TILT DEBUG: Using offsets[0]=%.6f, offsets[1]=%.6f, z_offset=%.6f", 
                   offsets[0], offsets[1], z_offset)
        
        # Calculate per-position adjustments
        adjustments = [x*x_adjust + y*y_adjust + z_adjust for x, y in self.z_positions]
        
        # Display per-position adjustments to user
        adj_msg = ["Z-Tilt: Per-position adjustments:"]
        for i, (pos, adj) in enumerate(zip(self.z_positions, adjustments)):
            adj_msg.append("  Position (%.1f,%.1f): adjustment=%.6f" % (pos[0], pos[1], adj))
        gcode.respond_info('\n'.join(adj_msg))
        
        logging.info("Z_TILT DEBUG: Per-position adjustments:")
        for i, (pos, adj) in enumerate(zip(self.z_positions, adjustments)):
            logging.info("Z_TILT DEBUG:   Position %d (%.3f,%.3f): adjustment=%.6f", 
                       i, pos[0], pos[1], adj)
        
        # Log before applying adjustments
        logging.info("Z_TILT DEBUG: About to apply adjustments to steppers at speed=%.3f", speed)
        
        # Apply the adjustments to steppers
        gcode.respond_info("Z-Tilt: Applying calculated adjustments to steppers...")
        self.z_helper.adjust_steppers(adjustments, speed)
        
        # Log after applying adjustments
        logging.info("Z_TILT DEBUG: Stepper adjustments completed")
        gcode.respond_info("Z-Tilt: Stepper adjustments completed")
        
        # Check retry status
        z_positions = [p[2] for p in positions]
        retry_result = self.retry_helper.check_retry(z_positions)
        logging.info("Z_TILT DEBUG: Retry check result: %s with z_positions=%s", 
                   retry_result, z_positions)
        
        # Report final status to user
        if retry_result == "done":
            gcode.respond_info("Z-Tilt: Adjustment completed successfully")
        else:
            gcode.respond_info("Z-Tilt: Retrying adjustment")
        
        return self.z_status.check_retry_result(retry_result)
    def get_status(self, eventtime):
            return self.z_status.get_status(eventtime)

def load_config(config):
    return ZTilt(config)