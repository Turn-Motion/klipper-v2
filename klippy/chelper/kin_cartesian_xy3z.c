<<<<<<< HEAD
// Cartesian kinematics stepper pulse time generation (with additional a and b axes)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2024  Modified for additional axes
=======
// Cartesian kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

static double
cart_stepper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).x;
}

static double
cart_stepper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).y;
}

static double
cart_stepper_z_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).z;
}

static double
cart_stepper_a_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).a;
}

static double
cart_stepper_b_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).b;
}

struct stepper_kinematics * __visible
cartesian_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (axis == 'x') {
        sk->calc_position_cb = cart_stepper_x_calc_position;
        sk->active_flags = AF_X;
    } else if (axis == 'y') {
        sk->calc_position_cb = cart_stepper_y_calc_position;
        sk->active_flags = AF_Y;
    } else if (axis == 'z') {
        sk->calc_position_cb = cart_stepper_z_calc_position;
        sk->active_flags = AF_Z;
<<<<<<< HEAD
    } else if (axis == 'a') {
        sk->calc_position_cb = cart_stepper_a_calc_position;
        sk->active_flags = AF_A;  // Note: AF_A needs to be defined in itersolve.h
    } else if (axis == 'b') {
        sk->calc_position_cb = cart_stepper_b_calc_position;
        sk->active_flags = AF_B;  // Note: AF_B needs to be defined in itersolve.h
    }
    return sk;
} 
=======
    }else if (axis == 'a') {
        sk->calc_position_cb = cart_stepper_a_calc_position;
        sk->active_flags = AF_A;
    } else if (axis == 'b') {
        sk->calc_position_cb = cart_stepper_b_calc_position;
        sk->active_flags = AF_B;
    }
    return sk;
}
>>>>>>> parent of 42d3b60e5 (Revert to 0.12.0)
