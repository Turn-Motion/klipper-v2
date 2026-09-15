// Derived horizontal bed-joint carriages
//
// The bed is a rigid plate carried on three ball joints:
//   A (front-left) and B (front-right) slide horizontally in X
//   Z (back) slides horizontally in Y
// Each joint sits on a vertical actuator commanded by the Z/A/B axes.
//
// Plate rigidity fixes the A and B slides completely - they are NOT free
// degrees of freedom.  Writing the A-B rigid-link constraint and eliminating
// the Z joint's free Y slide gives, for the joint world X positions:
//
//     z_mz = Z - (A + B) / 2                (pitch variable)
//     z_ab = B - A                          (roll variable)
//     csx  = sqrt(d_x^2 - z_ab^2)           (projected A-B separation)
//     U    = mid_x + z_mz*z_ab/csx - csx/2  (A joint world X)
//     V    = mid_x + z_mz*z_ab/csx + csx/2  (B joint world X)
//
// with d_x the A-B joint spacing and mid_x their midpoint.  Verified against
// the reference printerkin.solver implementation to ~1e-13 mm across the whole
// reachable envelope (see test/test_uv_derived.py).
//
// Steppers are commanded in *slide* coordinates - the offset from the home
// joint position - so both read 0 with the bed level, matching printerkin's
// A_slide / B_slide.
//
// The Z, A and B axis indexes are passed in, because the kinematics class
// owns the axis roster and a machine may order its axes differently.
//
// Copyright (C) 2024-2026  Turn-Motion
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible, container_of
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

// Floor on csx^2 as a fraction of d_x^2, so a pose that cannot be assembled
// yields a finite (if meaningless) position instead of NaN.  The reachable
// axis range keeps csx above ~57% of d_x, so this never engages in practice -
// the kinematics' check_move() is the real guard.
#define MIN_CSX2_FRAC 0.01

struct bedjoint_stepper {
    struct stepper_kinematics sk;
    double d_x2;    // (b_home_x - a_home_x)^2
    double mid_x;   // (a_home_x + b_home_x) / 2
    double home_x;  // this joint's world X at home (slide == 0 there)
    double side;    // -1.0 for the A-side carriage, +1.0 for the B-side
    int z_axis, a_axis, b_axis;
};

static double
bedjoint_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                               , double move_time)
{
    struct bedjoint_stepper *bs = container_of(
        sk, struct bedjoint_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double cz = c.axis[bs->z_axis];
    double ca = c.axis[bs->a_axis];
    double cb = c.axis[bs->b_axis];
    double z_mz = cz - 0.5 * (ca + cb);
    double z_ab = cb - ca;
    double rad = bs->d_x2 - z_ab * z_ab;
    double floor_rad = MIN_CSX2_FRAC * bs->d_x2;
    if (rad < floor_rad)
        rad = floor_rad;
    double csx = sqrt(rad);
    double world_x = bs->mid_x + z_mz * z_ab / csx + bs->side * 0.5 * csx;
    return world_x - bs->home_x;
}

// side < 0 selects the A-side carriage, side >= 0 the B-side one.
struct stepper_kinematics * __visible
bedjoint_stepper_alloc(double a_home_x, double b_home_x, double side
                       , int z_axis, int a_axis, int b_axis)
{
    if (z_axis < 0 || z_axis >= KIN_AXES || a_axis < 0 || a_axis >= KIN_AXES
        || b_axis < 0 || b_axis >= KIN_AXES) {
        errorf("bedjoint_stepper_alloc: axis index out of range");
        return NULL;
    }
    struct bedjoint_stepper *bs = malloc(sizeof(*bs));
    memset(bs, 0, sizeof(*bs));
    double d_x = b_home_x - a_home_x;
    bs->d_x2 = d_x * d_x;
    bs->mid_x = 0.5 * (a_home_x + b_home_x);
    if (side < 0.) {
        bs->side = -1.0;
        bs->home_x = a_home_x;
    } else {
        bs->side = 1.0;
        bs->home_x = b_home_x;
    }
    bs->z_axis = z_axis;
    bs->a_axis = a_axis;
    bs->b_axis = b_axis;
    bs->sk.calc_position_cb = bedjoint_stepper_calc_position;
    // The carriage position is a function of Z, A and B, so step generation
    // must run whenever any of them moves.
    bs->sk.active_flags = (AF_AXIS(z_axis) | AF_AXIS(a_axis)
                           | AF_AXIS(b_axis));
    return &bs->sk;
}
