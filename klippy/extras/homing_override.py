# Run user defined actions in place of a normal G28 homing command
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class HomingOverride:
    def __init__(self, config):
        self.printer = config.get_printer()
        # The axis roster is not known until the kinematics loads, so
        # take whatever "set_position_<axis>" options are present
        self.start_pos_config = {}
        for option in config.get_prefix_options('set_position_'):
            name = option[len('set_position_'):].lower()
            if len(name) == 1 and name.isalpha():
                self.start_pos_config[name] = config.getfloat(option, None)
        self.axes = config.get('axes', '').upper()
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.template = gcode_macro.load_template(config, 'gcode')
        self.in_script = False
        self.printer.load_object(config, 'homing')
        self.gcode = self.printer.lookup_object('gcode')
        self.prev_G28 = self.gcode.register_command("G28", None)
        self.gcode.register_command("G28", self.cmd_G28)
    def cmd_G28(self, gcmd):
        if self.in_script:
            # Was called recursively - invoke the real G28 command
            self.prev_G28(gcmd)
            return

        toolhead = self.printer.lookup_object('toolhead')
        axis_names = toolhead.get_axis_names()
        axes = self.axes or axis_names.upper()

        # if no axis is given as parameter we assume the override
        no_axis = True
        for axis in axis_names:
            if gcmd.get(axis.upper(), None) is not None:
                no_axis = False
                break

        if no_axis:
            override = True
        else:
            # check if we home an axis which needs the override
            override = False
            for axis in axes:
                if gcmd.get(axis, None) is not None:
                    override = True

        if not override:
            self.prev_G28(gcmd)
            return

        # Calculate forced position (if configured)
        pos = toolhead.get_position()
        homing_axes = ""
        for name, loc in self.start_pos_config.items():
            if loc is None:
                continue
            axis = axis_names.find(name)
            if axis < 0:
                raise gcmd.error("homing_override: this printer has no '%s'"
                                 " axis" % (name.upper(),))
            pos[axis] = loc
            homing_axes += name
        toolhead.set_position(pos, homing_axes=homing_axes)
        # Perform homing
        context = self.template.create_template_context()
        context['params'] = gcmd.get_command_parameters()
        context['rawparams'] = gcmd.get_raw_command_parameters()
        try:
            self.in_script = True
            self.template.run_gcode_from_command(context)
        finally:
            self.in_script = False

def load_config(config):
    return HomingOverride(config)
