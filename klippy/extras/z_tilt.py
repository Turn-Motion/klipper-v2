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
        logging.info("Z_TILT DEBUG: Kinematics type: %s", type(kin).__name__)
        
        # Get all steppers and log their details
        all_steppers = kin.get_steppers()
        logging.info("Z_TILT DEBUG: Total steppers found: %d", len(all_steppers))
        for i, s in enumerate(all_steppers):
            logging.info("Z_TILT DEBUG: Stepper %d: name=%s, z=%s, a=%s, b=%s", 
                        i, s.get_name(), s.is_active_axis('z'), 
                        s.is_active_axis('a'), s.is_active_axis('b'))
        
        # Find all Z-axis steppers (including those named stepper_a and stepper_b)
        z_steppers = [s for s in kin.get_steppers() if s.is_active_axis('z')]
        logging.info("Z_TILT DEBUG: Found %d Z-axis steppers", len(z_steppers))
        
        # ---- Turn V2 ----
        # For CartesianXY3Z kinematics, also include A and B axis steppers, this is a particular fix for TurnV2
        # since they physically move in the Z direction
        if hasattr(kin, 'rails') and len(kin.rails) >= 5:  # CartesianXY3Z has 5 rails: x,y,z,a,b
            logging.info("Z_TILT DEBUG: CartesianXY3Z detected with %d rails", len(kin.rails))
            a_steppers = [s for s in kin.get_steppers() if s.is_active_axis('a')]
            logging.info("Z_TILT DEBUG: Found %d A-axis steppers", len(a_steppers))
            for i, s in enumerate(a_steppers):
                logging.info("Z_TILT DEBUG: A-stepper %d: %s", i, s.get_name())
                
            b_steppers = [s for s in kin.get_steppers() if s.is_active_axis('b')]
            logging.info("Z_TILT DEBUG: Found %d B-axis steppers", len(b_steppers))
            for i, s in enumerate(b_steppers):
                logging.info("Z_TILT DEBUG: B-stepper %d: %s", i, s.get_name())
                
            z_steppers.extend(a_steppers)
            z_steppers.extend(b_steppers)
            logging.info("Z_TILT DEBUG: After adding A+B, total Z steppers: %d", len(z_steppers))
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
        
        self.z_steppers = z_steppers
    def adjust_steppers(self, adjustments, speed):
        toolhead = self.printer.lookup_object('toolhead')
        gcode = self.printer.lookup_object('gcode')
        curpos = toolhead.get_position()
        # Report on movements
        stepstrs = ["%s = %.6f" % (s.get_name(), a)
                    for s, a in zip(self.z_steppers, adjustments)]
        msg = "Making the following Z adjustments:\n%s" % ("\n".join(stepstrs),)
        gcode.respond_info(msg)
        # Disable Z stepper movements
        toolhead.flush_step_generation()
        for s in self.z_steppers:
            s.set_trapq(None)
        # Move each z stepper (sorted from lowest to highest) until they match
        positions = [(-a, s) for a, s in zip(adjustments, self.z_steppers)]
        positions.sort(key=(lambda k: k[0]))
        first_stepper_offset, first_stepper = positions[0]
        z_low = curpos[2] - first_stepper_offset
        for i in range(len(positions)-1):
            stepper_offset, stepper = positions[i]
            next_stepper_offset, next_stepper = positions[i+1]
            toolhead.flush_step_generation()
            stepper.set_trapq(toolhead.get_trapq())
            curpos[2] = z_low + next_stepper_offset
            try:
                toolhead.move(curpos, speed)
                toolhead.set_position(curpos)
            except:
                logging.exception("ZAdjustHelper adjust_steppers")
                toolhead.flush_step_generation()
                for s in self.z_steppers:
                    s.set_trapq(toolhead.get_trapq())
                raise
        # Z should now be level - do final cleanup
        last_stepper_offset, last_stepper = positions[-1]
        toolhead.flush_step_generation()
        last_stepper.set_trapq(toolhead.get_trapq())
        curpos[2] += first_stepper_offset
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
        
        # Configuration for the homing sequence
        self.home_xy_before_z = config.getboolean('home_xy_before_z', True)
        self.xy_position = config.getfloatlist('xy_position', [250., 250.], count=2)
        self.z_endstop_name = config.get('z_endstop_sensor', 'probe')
        self.z_hop = config.getfloat('z_hop', 5.0)
        
        # Register Z_TILT_ADJUST command
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('Z_TILT_ADJUST', self.cmd_Z_TILT_ADJUST,
                               desc=self.cmd_Z_TILT_ADJUST_help)
    
    def up_home_z(self):
        """Move Z upward until the hot end sensor is triggered"""
        gcode = self.printer.lookup_object('gcode')
        toolhead = self.printer.lookup_object('toolhead')
        
        # Get current position
        curpos = toolhead.get_position()
        
        # Execute up-homing by running a gcode command
        gcode.respond_info("Performing up-home Z operation")
        try:
            # Try to use the specified endstop for up-homing
            gcode.run_script_from_command("PROBE_UP")
        except Exception as e:
            gcode.respond_info("Warning: PROBE_UP failed, using fallback method: %s" % str(e))
            # Fallback method: Move Z up by z_hop distance
            curpos[2] += self.z_hop
            toolhead.move(curpos, 5.0)
            gcode.respond_info("Moved Z up by %.1fmm as fallback" % self.z_hop)
    
    cmd_Z_TILT_ADJUST_help = "Adjust the Z tilt with complete homing sequence"
    def cmd_Z_TILT_ADJUST(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        toolhead = self.printer.lookup_object('toolhead')
        
        # Reset Z tilt status
        self.z_status.reset()
        
        # Step 1: Home X and Y if configured
        if self.home_xy_before_z:
            gcode.respond_info("Homing X and Y axes")
            try:
                gcode.run_script_from_command("G28 X Y")
            except Exception as e:
                raise gcmd.error("Error homing X Y: %s" % str(e))
        
        # Step 2: Move to XY position
        gcode.respond_info("Moving to position X=%.1f Y=%.1f" % 
                          (self.xy_position[0], self.xy_position[1]))
        try:
            curpos = toolhead.get_position()
            curpos[0] = self.xy_position[0]
            curpos[1] = self.xy_position[1]
            toolhead.move(curpos, 50.0)
        except Exception as e:
            raise gcmd.error("Error moving to XY position: %s" % str(e))
        
        # Step 3: Perform up-home Z
        self.up_home_z()
        
        # Step 4: Start the Z-tilt adjustment process
        gcode.respond_info("Starting Z-tilt adjustment")
        self.retry_helper.start(gcmd)
        self.probe_helper.start_probe(gcmd)
        
        # Note: The probe_finalize method will be called after probing completes
        # and will handle the actual Z adjustment
    
    def probe_finalize(self, offsets, positions):
        # Setup for coordinate descent analysis
        z_offset = offsets[2]
        logging.info("Calculating bed tilt with: %s", positions)
        params = { 'x_adjust': 0., 'y_adjust': 0., 'z_adjust': z_offset }
        # Perform coordinate descent
        def adjusted_height(pos, params):
            x, y, z = pos
            return (z - x*params['x_adjust'] - y*params['y_adjust']
                    - params['z_adjust'])
        def errorfunc(params):
            total_error = 0.
            for pos in positions:
                total_error += adjusted_height(pos, params)**2
            return total_error
        new_params = mathutil.coordinate_descent(
            params.keys(), params, errorfunc)
        # Apply results
        speed = self.probe_helper.get_lift_speed()
        logging.info("Calculated bed tilt parameters: %s", new_params)
        x_adjust = new_params['x_adjust']
        y_adjust = new_params['y_adjust']
        z_adjust = (new_params['z_adjust'] - z_offset
                    - x_adjust * offsets[0] - y_adjust * offsets[1])
        adjustments = [x*x_adjust + y*y_adjust + z_adjust
                       for x, y in self.z_positions]
        self.z_helper.adjust_steppers(adjustments, speed)
        
        # After adjustments are complete, perform another up-home Z
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_info("Z-tilt adjustment complete, performing final up-home Z")
        self.up_home_z()
        
        return self.z_status.check_retry_result(
            self.retry_helper.check_retry([p[2] for p in positions]))
    def get_status(self, eventtime):
            return self.z_status.get_status(eventtime)

def load_config(config):
    return ZTilt(config)
