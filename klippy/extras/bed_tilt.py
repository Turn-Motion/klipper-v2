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
        # The A and B bed motors move with Z and carry the same tilt
        # correction plus their own offsets
        self.a_adjust = config.getfloat('a_adjust', self.z_adjust)
        self.b_adjust = config.getfloat('b_adjust', self.z_adjust)
        if config.get('points', None) is not None:
            BedTiltCalibrate(config, self)
        self.toolhead = None
        # Register move transform with g-code class
        gcode_move = self.printer.load_object(config, 'gcode_move')
        gcode_move.set_move_transform(self)
    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
    def get_position(self):
        pos = self.toolhead.get_position()
        x, y, z, a, b = pos[:5]
        tilt = x*self.x_adjust + y*self.y_adjust
        return [x, y, z - tilt - self.z_adjust, a - tilt - self.a_adjust,
                b - tilt - self.b_adjust] + pos[5:]
    def move(self, newpos, speed):
        x, y, z, a, b = newpos[:5]
        tilt = x*self.x_adjust + y*self.y_adjust
        self.toolhead.move([x, y, z + tilt + self.z_adjust,
                            a + tilt + self.a_adjust,
                            b + tilt + self.b_adjust] + newpos[5:], speed)
    def update_adjust(self, x_adjust, y_adjust, z_adjust,
                      a_adjust=None, b_adjust=None):
        if a_adjust is None:
            a_adjust = z_adjust
        if b_adjust is None:
            b_adjust = z_adjust
        self.x_adjust = x_adjust
        self.y_adjust = y_adjust
        self.z_adjust = z_adjust
        self.a_adjust = a_adjust
        self.b_adjust = b_adjust
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
    def probe_finalize(self, positions):
        logging.info("Calculating bed_tilt with: %s", positions)
        # Find best fit for: a + b*bed_x + c*bed_y = bed_z
        eqs = []
        ans = []
        for pos in positions:
            eqs.append([1., pos.bed_x, pos.bed_y])
            ans.append([pos.bed_z])
        res = mathutil.solve_linear_equations(eqs, ans)
        if res is None:
            raise self.gcode.error("Unable to calculate tilt")
        z_adjust = res[0][0]
        x_adjust = res[1][0]
        y_adjust = res[2][0]
        # Update current bed_tilt calculations
        self.bedtilt.update_adjust(x_adjust, y_adjust, z_adjust)
        # Log and report results
        msg = ("x_adjust: %.6f y_adjust: %.6f z_adjust: %.6f"
               " (a_adjust and b_adjust set to z_adjust)" % (
                   x_adjust, y_adjust, z_adjust))
        self.printer.set_rollover_info("bed_tilt", "bed_tilt: %s" % (msg,))
        self.gcode.respond_info(
            "%s\nThe above parameters have been applied to the current\n"
            "session. The SAVE_CONFIG command will update the printer\n"
            "config file and restart the printer." % (msg,))

def load_config(config):
    return BedTilt(config)
