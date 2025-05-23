# Bed tilt compensation
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import mathutil
from . import probe

class BedTilt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.x_adjust = config.getfloat('x_adjust', 0.)
        self.y_adjust = config.getfloat('y_adjust', 0.)
        self.z_adjust = config.getfloat('z_adjust', 0.)
        # Add A and B adjustments since they move with Z
        self.a_adjust = config.getfloat('a_adjust', 0.)
        self.b_adjust = config.getfloat('b_adjust', 0.)
        if config.get('points', None) is not None:
            BedTiltCalibrate(config, self)
        self.toolhead = None
        # Register move transform with g-code class
        gcode_move = self.printer.load_object(config, 'gcode_move')
        gcode_move.set_move_transform(self)
    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
    def get_position(self):
        x, y, z, a, b, e = self.toolhead.get_position()
        # Apply adjustments to Z, A, and B since they move together
        return [x, y, z - x*self.x_adjust - y*self.y_adjust - self.z_adjust,
                a - x*self.x_adjust - y*self.y_adjust - self.a_adjust,
                b - x*self.x_adjust - y*self.y_adjust - self.b_adjust, e]
    def move(self, newpos, speed):
        x, y, z, a, b, e = newpos
        # Apply inverse adjustments to Z, A, and B since they move together
        self.toolhead.move([x, y,
                           z + x*self.x_adjust + y*self.y_adjust + self.z_adjust,
                           a + x*self.x_adjust + y*self.y_adjust + self.a_adjust,
                           b + x*self.x_adjust + y*self.y_adjust + self.b_adjust, e],
                          speed)
    def update_adjust(self, x_adjust, y_adjust, z_adjust, a_adjust=None, b_adjust=None):
        self.x_adjust = x_adjust
        self.y_adjust = y_adjust
        self.z_adjust = z_adjust
        # Update A and B adjustments if provided, otherwise use Z adjustment
        self.a_adjust = a_adjust if a_adjust is not None else z_adjust
        self.b_adjust = b_adjust if b_adjust is not None else z_adjust
        gcode_move = self.printer.lookup_object('gcode_move')
        gcode_move.reset_last_position()
        configfile = self.printer.lookup_object('configfile')
        configfile.set('bed_tilt', 'x_adjust', "%.6f" % (x_adjust,))
        configfile.set('bed_tilt', 'y_adjust', "%.6f" % (y_adjust,))
        configfile.set('bed_tilt', 'z_adjust', "%.6f" % (z_adjust,))
        configfile.set('bed_tilt', 'a_adjust', "%.6f" % (a_adjust,))
        configfile.set('bed_tilt', 'b_adjust', "%.6f" % (b_adjust,))

# Helper script to calibrate the bed tilt
class BedTiltCalibrate:
    def __init__(self, config, bedtilt):
        self.printer = config.get_printer()
        self.bedtilt = bedtilt
        self.probe_helper = probe.ProbePointsHelper(config, self.probe_finalize)
        self.probe_helper.minimum_points(3)
        # Register BED_TILT_CALIBRATE command
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'BED_TILT_CALIBRATE', self.cmd_BED_TILT_CALIBRATE,
            desc=self.cmd_BED_TILT_CALIBRATE_help)
    cmd_BED_TILT_CALIBRATE_help = "Bed tilt calibration script"
    def cmd_BED_TILT_CALIBRATE(self, gcmd):
        self.probe_helper.start_probe(gcmd)
    def probe_finalize(self, offsets, positions):
        # Setup for coordinate descent analysis
        z_offset = offsets[2]
        a_offset = offsets[3] if len(offsets) > 3 else z_offset
        b_offset = offsets[4] if len(offsets) > 4 else z_offset
        logging.info("Calculating bed_tilt with: %s", positions)
        params = { 'x_adjust': self.bedtilt.x_adjust,
                  'y_adjust': self.bedtilt.y_adjust,
                  'z_adjust': z_offset,
                  'a_adjust': a_offset,
                  'b_adjust': b_offset }
        logging.info("Initial bed_tilt parameters: %s", params)
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
        # Update current bed_tilt calculations
        x_adjust = new_params['x_adjust']
        y_adjust = new_params['y_adjust']
        z_adjust = (new_params['z_adjust'] - z_offset
                    - x_adjust * offsets[0] - y_adjust * offsets[1])
        a_adjust = (new_params['a_adjust'] - a_offset
                    - x_adjust * offsets[0] - y_adjust * offsets[1])
        b_adjust = (new_params['b_adjust'] - b_offset
                    - x_adjust * offsets[0] - y_adjust * offsets[1])
        self.bedtilt.update_adjust(x_adjust, y_adjust, z_adjust, a_adjust, b_adjust)
        # Log and report results
        logging.info("Calculated bed_tilt parameters: %s", new_params)
        for pos in positions:
            logging.info("orig: %s new: %s", adjusted_height(pos, params),
                         adjusted_height(pos, new_params))
        msg = ("x_adjust: %.6f y_adjust: %.6f z_adjust: %.6f\n"
               "a_adjust: %.6f b_adjust: %.6f" % (
                   x_adjust, y_adjust, z_adjust, a_adjust, b_adjust))
        self.printer.set_rollover_info("bed_tilt", "bed_tilt: %s" % (msg,))
        self.gcode.respond_info(
            "%s\nThe above parameters have been applied to the current\n"
            "session. The SAVE_CONFIG command will update the printer\n"
            "config file and restart the printer." % (msg,))

def load_config(config):
    return BedTilt(config)
