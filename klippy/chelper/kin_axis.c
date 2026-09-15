// Single-coordinate ("pass through") stepper kinematics for any axis
//
// A stepper allocated here follows exactly one kinematic axis of the
// move.  The axis is given by its index in the kinematics' axis roster,
// so the same solver serves x, y, z and every additional axis a
// kinematics declares (the bed joint motors and a rotary axis, for
// this fork's machines).
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2024-2026  Turn-Motion
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible, container_of
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

struct axis_stepper {
    struct stepper_kinematics sk;
    int axis;
};

static double
axis_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                           , double move_time)
{
    struct axis_stepper *as = container_of(sk, struct axis_stepper, sk);
    return move_get_coord(m, move_time).axis[as->axis];
}

struct stepper_kinematics * __visible
axis_stepper_alloc(int axis)
{
    if (axis < 0 || axis >= KIN_AXES) {
        errorf("axis_stepper_alloc: axis %d out of range", axis);
        return NULL;
    }
    struct axis_stepper *as = malloc(sizeof(*as));
    memset(as, 0, sizeof(*as));
    as->axis = axis;
    as->sk.calc_position_cb = axis_stepper_calc_position;
    as->sk.active_flags = AF_AXIS(axis);
    return &as->sk;
}
