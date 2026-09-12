// Derived horizontal bed-joint carriages for cartesianxy3z2h kinematics
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
// Copyright (C) 2024-2026  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible, container_of
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

// Floor on csx^2 as a fraction of d_x^2, so a pose that cannot be assembled
// yields a finite (if meaningless) position instead of NaN.  The reachable
// axis range keeps csx above ~57% of d_x, so this never engages in practice -
// CartesianXY3Z2H.check_move() is the real guard.
#define MIN_CSX2_FRAC 0.01

struct h_stepper {
    struct stepper_kinematics sk;
    double d_x2;    // (b_home_x - a_home_x)^2
    double mid_x;   // (a_home_x + b_home_x) / 2
    double home_x;  // this joint's world X at home (slide == 0 there)
    double side;    // -1.0 for the A-side carriage, +1.0 for the B-side
};

static double
h_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                        , double move_time)
{
    struct h_stepper *hs = container_of(sk, struct h_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double z_mz = c.z - 0.5 * (c.a + c.b);
    double z_ab = c.b - c.a;
    double rad = hs->d_x2 - z_ab * z_ab;
    double floor_rad = MIN_CSX2_FRAC * hs->d_x2;
    if (rad < floor_rad)
        rad = floor_rad;
    double csx = sqrt(rad);
    double world_x = hs->mid_x + z_mz * z_ab / csx + hs->side * 0.5 * csx;
    return world_x - hs->home_x;
}

// side < 0 selects the A-side carriage, side >= 0 the B-side one.
struct stepper_kinematics * __visible
cartesianxy3z2h_h_stepper_alloc(double a_home_x, double b_home_x, double side)
{
    struct h_stepper *hs = malloc(sizeof(*hs));
    memset(hs, 0, sizeof(*hs));
    double d_x = b_home_x - a_home_x;
    hs->d_x2 = d_x * d_x;
    hs->mid_x = 0.5 * (a_home_x + b_home_x);
    if (side < 0.) {
        hs->side = -1.0;
        hs->home_x = a_home_x;
    } else {
        hs->side = 1.0;
        hs->home_x = b_home_x;
    }
    hs->sk.calc_position_cb = h_stepper_calc_position;
    // The carriage position is a function of Z, A and B, so step generation
    // must run whenever any of them moves.
    hs->sk.active_flags = AF_Z | AF_A | AF_B;
    return &hs->sk;
}
