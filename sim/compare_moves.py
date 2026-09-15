#!/usr/bin/env python
# Compare two [move_capture] JSONL files move by move
#
# Used to prove that a change to the motion core produces the same
# motion: same positions, same extrusion, same velocities.  See
# FORK_CHANGES.md section 7.
#
#   compare_moves.py <baseline.jsonl> <candidate.jsonl> [position tolerance]
#
# Copyright (C) 2024-2026  Turn-Motion
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json, sys

POS_TOL = 1e-6
VEL_TOL = 1e-6


def load(path):
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def nonempty(moves):
    return [m for m in moves if m['md'] > 1e-9]


def main():
    if len(sys.argv) < 3:
        sys.stderr.write("usage: compare_moves.py BASE CAND [pos_tol]\n")
        return 2
    base_path, cand_path = sys.argv[1], sys.argv[2]
    pos_tol = float(sys.argv[3]) if len(sys.argv) > 3 else POS_TOL
    base, cand = load(base_path), load(cand_path)
    b, c = nonempty(base), nonempty(cand)
    print("baseline : %s  %d moves (%d non-empty)"
          % (base_path, len(base), len(b)))
    print("candidate: %s  %d moves (%d non-empty)"
          % (cand_path, len(cand), len(c)))
    if not b or not c:
        print("FAIL: one of the captures is empty")
        return 1
    if len(b) != len(c):
        print("FAIL: different number of non-empty moves (%d vs %d)"
              % (len(b), len(c)))
        return 1
    pos_bad = ext_bad = vel_bad = 0
    worst_pos = worst_vel = 0.
    worst_pos_i = worst_vel_i = -1
    for i, (mb, mc) in enumerate(zip(b, c)):
        naxes = min(len(mb['ep']), len(mc['ep']))
        for key in ('sp', 'ep'):
            for j in range(naxes):
                d = abs(mb[key][j] - mc[key][j])
                if d > worst_pos:
                    worst_pos, worst_pos_i = d, i
                if d > pos_tol:
                    pos_bad += 1
        if abs(mb['ed'] - mc['ed']) > pos_tol:
            ext_bad += 1
        for key in ('sv', 'cv', 'ev'):
            d = abs(mb[key] - mc[key])
            if d > worst_vel:
                worst_vel, worst_vel_i = d, i
            if d > VEL_TOL:
                vel_bad += 1
    print("worst position difference : %.9f (move %d)"
          % (worst_pos, worst_pos_i))
    print("worst velocity difference : %.9f (move %d)"
          % (worst_vel, worst_vel_i))
    print("moves with position differences  : %d" % (pos_bad,))
    print("moves with extrusion differences : %d" % (ext_bad,))
    print("moves with velocity differences  : %d" % (vel_bad,))
    if pos_bad or ext_bad or vel_bad:
        print("FAIL: captures differ")
        for i, (mb, mc) in enumerate(zip(b, c)):
            if mb['ep'] != mc['ep'] or abs(mb['cv'] - mc['cv']) > VEL_TOL:
                print("  first differing move %d:" % (i,))
                print("    base: %s" % (json.dumps(mb),))
                print("    cand: %s" % (json.dumps(mc),))
                break
        return 1
    print("PASS: %d moves identical" % (len(b),))
    return 0


if __name__ == '__main__':
    sys.exit(main())
