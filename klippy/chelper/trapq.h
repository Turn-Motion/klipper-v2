#ifndef TRAPQ_H
#define TRAPQ_H

#include "list.h" // list_node

// Maximum number of kinematic axes a move can carry.  The kinematics
// class decides how many of them a printer actually uses (three for
// the stock cartesian, corexy, delta, ... kinematics); unused entries
// are always zero.
#define KIN_AXES 8

struct coord {
    union {
        struct {
            double x, y, z;
        };
        double axis[KIN_AXES];
    };
};

struct move {
    double print_time, move_t;
    double start_v, half_accel;
    struct coord start_pos, axes_r;

    struct list_node node;
};

struct trapq {
    struct list_head moves, history;
};

struct pull_move {
    double print_time, move_t;
    double start_v, accel;
    double start_pos[KIN_AXES];
    double axes_r[KIN_AXES];
};

int trapq_get_kin_axes(void);
void trapq_fill_coord(struct coord *c, int naxes, const double *vals);
struct move *move_alloc(void);
double move_get_distance(struct move *m, double move_time);
struct coord move_get_coord(struct move *m, double move_time);
struct trapq *trapq_alloc(void);
void trapq_free(struct trapq *tq);
void trapq_check_sentinels(struct trapq *tq);
void trapq_add_move(struct trapq *tq, struct move *m);
void trapq_append(struct trapq *tq, double print_time
                  , double accel_t, double cruise_t, double decel_t
                  , int naxes, const double *start_pos, const double *axes_r
                  , double start_v, double cruise_v, double accel);
void trapq_finalize_moves(struct trapq *tq, double print_time
                          , double clear_history_time);
void trapq_set_position(struct trapq *tq, double print_time
                        , int naxes, const double *pos);
int trapq_extract_old(struct trapq *tq, struct pull_move *p, int max
                      , double start_time, double end_time);

#endif // trapq.h
