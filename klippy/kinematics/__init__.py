# Package definition for the kinematics directory
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2024  Modified for additional axes
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from . import cartesian, cartesian_xy3z, corexy, corexz, delta, deltesian
from . import polar, rotary_delta, winch

def load_kinematics(toolhead, config):
    kinematics = config.getchoice('kinematics', {
        'cartesian': cartesian,
        'cartesian_xy3z': cartesian_xy3z,  # Added new kinematics type
        'corexy': corexy,
        'corexz': corexz,
        'delta': delta,
        'deltesian': deltesian,
        'polar': polar,
        'rotary_delta': rotary_delta,
        'winch': winch,
    })
    return kinematics.load_kinematics(toolhead, config)
