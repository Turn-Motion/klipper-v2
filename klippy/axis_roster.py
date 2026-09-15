# Helpers for querying the kinematics' axis roster
#
# The kinematics class owns the list of kinematic axes: how many there
# are, what letter each one has, and which of them carry the bed.  This
# module is the single place the rest of the host code asks, so that no
# module has to hard-code an axis index or assume a particular machine.
#
# A kinematics class may implement:
#   axis_names        - the axis letters, in order (default "xyz")
#   get_bed_axes()    - axis indexes that raise and lower the bed, the
#                       primary ("Z") axis first (default: the z axis)
#   get_bed_steppers()- the steppers those axes drive (default: derived
#                       from get_bed_axes())
#   get_derived_steppers() - steppers whose position follows the other
#                       axes but which are not axes themselves; these
#                       are never treated as bed motors
#
# Copyright (C) 2024-2026  Turn-Motion
#
# This file may be distributed under the terms of the GNU GPLv3 license.

DEFAULT_AXIS_NAMES = "xyz"

def get_axis_names(printer):
    toolhead = printer.lookup_object('toolhead', None)
    if toolhead is None:
        return DEFAULT_AXIS_NAMES
    return toolhead.get_axis_names()

def get_axis_count(printer):
    return len(get_axis_names(printer))

def get_extruder_index(printer):
    return len(get_axis_names(printer))

def get_axis_index(printer, axis_name):
    # Index of an axis letter, or None if this machine has no such axis
    names = get_axis_names(printer)
    index = names.find(axis_name)
    if index < 0:
        return None
    return index

def get_kinematics(printer):
    return printer.lookup_object('toolhead').get_kinematics()

def get_bed_axes(printer):
    # The kinematic axes that move the bed vertically.  The first entry
    # is the axis whose coordinate is the bed height ("Z" on every
    # machine); any further entries are additional bed motors that must
    # move with it to keep the bed's tilt.
    kin = get_kinematics(printer)
    getter = getattr(kin, 'get_bed_axes', None)
    if getter is not None:
        return list(getter())
    index = get_axis_index(printer, 'z')
    if index is None:
        return []
    return [index]

def get_derived_steppers(printer):
    # Steppers whose position is derived from the kinematic axes rather
    # than being an axis of their own
    kin = get_kinematics(printer)
    getter = getattr(kin, 'get_derived_steppers', None)
    if getter is None:
        return []
    return list(getter())

def get_bed_steppers(printer):
    # The steppers that carry the bed.  Probing and bed levelling drive
    # these together; derived steppers are deliberately excluded.
    kin = get_kinematics(printer)
    getter = getattr(kin, 'get_bed_steppers', None)
    if getter is not None:
        return list(getter())
    bed_axes = get_bed_axes(printer)
    derived = set([id(s) for s in get_derived_steppers(printer)])
    return [s for s in kin.get_steppers()
            if id(s) not in derived
            and any([s.is_active_axis(axis) for axis in bed_axes])]

def bed_delta_position(printer, pos, delta):
    # Apply a vertical delta to every bed axis of a position
    newpos = list(pos)
    for axis in get_bed_axes(printer):
        newpos[axis] += delta
    return newpos

def bed_target_position(printer, pos, height):
    # Move the bed to an absolute height, carrying the other bed axes by
    # the same delta so that any tilt correction between them survives
    bed_axes = get_bed_axes(printer)
    if not bed_axes:
        return list(pos)
    return bed_delta_position(printer, pos, height - pos[bed_axes[0]])
