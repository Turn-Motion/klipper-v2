# Package definition for the kinematics directory
#
# Copyright (C) 2018-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from . import cartesian, corexy, corexz, delta, polar, rotary_delta
from . import hybrid_corexy, hybrid_corexz, deltesian, winch, none
from . import cartesianxy3z

def get_kinematics(config):
    """Get the kinematics class for the specified config."""
    kinematics = config.getchoice('kinematics', {
        'cartesian': cartesian.load_kinematics,
        'corexy': corexy.load_kinematics,
        'corexz': corexz.load_kinematics,
        'delta': delta.load_kinematics,
        'polar': polar.load_kinematics,
        'rotary_delta': rotary_delta.load_kinematics,
        'hybrid_corexy': hybrid_corexy.load_kinematics,
        'hybrid_corexz': hybrid_corexz.load_kinematics,
        'deltesian': deltesian.load_kinematics,
        'winch': winch.load_kinematics,
        'none': none.load_kinematics,
        'cartesianxy3z': cartesianxy3z.load_kinematics,
    })
    return kinematics
