# Derived horizontal bed-joint carriages (the U and V slides)
#
# The bed is a rigid plate on three ball joints.  The A and B joints
# slide horizontally in X; that slide is fully determined by the
# commanded bed pose through plate rigidity, so U and V are not degrees
# of freedom.  They are modelled as steppers with a non-trivial position
# function - the same way delta towers are - rather than as G-code axes:
#
#   * G-code keeps sending the kinematic axes; there is no U or V word.
#   * The position tuple is unchanged, so nothing else in the host code
#     has to know about them.
#   * U/V follow every bed transform (z_tilt, SET_GCODE_OFFSET, bed_mesh,
#     probe lifts) automatically, because they are recomputed from
#     whatever the bed axes actually are.
#   * Homing is free: toolhead.set_position() -> stepper.set_position()
#     -> itersolve_set_position() -> the derived callback.
#
# The equations live in chelper/kin_bedjoint.c and are verified against
# the reference printerkin.solver implementation in test/test_uv_derived.py.
#
# Two behaviours are selectable, so both can be measured on the machine:
#
#   travel_check = sampled | endpoints
#       The derived position is nonlinear in the bed pose, so a straight
#       chord between a move's endpoints understates both the peak
#       carriage rate (the end of a long sweep runs ~2x faster than the
#       chord average) and the extreme position (U(s) can have an
#       interior extremum, so a move whose endpoints are both in range
#       can still take a carriage past its rail limit partway through).
#       "sampled" evaluates along the move and catches both; "endpoints"
#       evaluates only the two endpoints, which is cheaper.
#
#   homing_mode = detached | attached
#       Klipper homes by first forcing the start position well beyond
#       the axis range so the move is guaranteed to reach the endstop.
#       For a single bed axis that is a pose the rigid plate cannot
#       physically occupy, and the derived carriage position computed
#       from it is meaningless.  "detached" takes the carriages off the
#       motion queue for the duration - the plate back-drives them while
#       the bed homes, and they are re-derived from the true pose
#       afterwards, emitting no steps.  "attached" leaves them on the
#       queue, so they follow the commanded (fictional) pose.
#
# Copyright (C) 2024-2026  Turn-Motion
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper

# Bed-pose travel (mm) per interior sample, and the cap on the count
UV_SAMPLE_MM = 10.0
UV_MAX_SAMPLES = 16

class Carriage:
    def __init__(self, name, mcu_stepper, position_min, position_max):
        self.name = name
        self.stepper = mcu_stepper
        self.position_min = position_min
        self.position_max = position_max

class BedJointCarriages:
    def __init__(self, kin, config):
        self.printer = config.get_printer()
        self.kin = kin
        toolhead = kin.toolhead
        hcfg = config.getsection('bed_joint_carriages')
        # The bed axes this derives from, in the order Z, A, B
        bed_axes = kin.get_bed_axes()
        if len(bed_axes) != 3:
            raise config.error(
                "[bed_joint_carriages] needs exactly three bed axes, but the"
                " %s kinematics has %d" % (kin.kin_name, len(bed_axes)))
        self.bed_axes = bed_axes
        # Geometry: the world X of each ball joint with the bed level
        self.a_joint_home_x = hcfg.getfloat('a_joint_home_x', -23.0)
        self.b_joint_home_x = hcfg.getfloat('b_joint_home_x', 623.0)
        if self.b_joint_home_x <= self.a_joint_home_x:
            raise config.error(
                "b_joint_home_x must be greater than a_joint_home_x")
        self.d_x = self.b_joint_home_x - self.a_joint_home_x
        # Behaviour selection
        self.travel_check = hcfg.getchoice('travel_check',
                                           ['sampled', 'endpoints'], 'sampled')
        self.homing_mode = hcfg.getchoice('homing_mode',
                                          ['detached', 'attached'], 'detached')
        # Velocity/accel limits.  Default to the bed limits so an
        # existing config needs no new keys.
        max_z_velocity, max_z_accel = toolhead.get_max_z_velocity()
        self.max_velocity = hcfg.getfloat('max_velocity', max_z_velocity,
                                          above=0.)
        self.max_accel = hcfg.getfloat('max_accel', max_z_accel, above=0.)
        # The carriages themselves
        names = [n.strip() for n in hcfg.getlist(
            "carriages", ["stepper_u", "stepper_v"])]
        if len(names) != 2:
            raise config.error(
                "[bed_joint_carriages] 'carriages' needs exactly two"
                " stepper sections")
        self.carriages = []
        for section_name, side in zip(names, (-1.0, 1.0)):
            if not config.has_section(section_name):
                raise config.error(
                    "Section '%s' is required by [bed_joint_carriages]"
                    % (section_name,))
            scfg = config.getsection(section_name)
            s = stepper.PrinterStepper(scfg)
            s.setup_itersolve('bedjoint_stepper_alloc',
                              self.a_joint_home_x, self.b_joint_home_x, side,
                              bed_axes[0], bed_axes[1], bed_axes[2])
            # Slide travel limits (0 == the level-bed home position).
            # REQUIRED, deliberately: the carriages have no endstop and
            # no encoder, so this range check is the only thing standing
            # between a commanded bed pose and a joint being driven off
            # the end of its rail by the (full power) bed actuators.
            position_min = scfg.getfloat('position_min')
            position_max = scfg.getfloat('position_max', above=position_min)
            self.carriages.append(Carriage(section_name, s,
                                           position_min, position_max))
        self.homing_trapq = None
        logging.info("bed_joint_carriages: %s, a_joint_home_x=%.3f"
                     " b_joint_home_x=%.3f (d_x=%.3f) max_velocity=%.1f"
                     " max_accel=%.1f travel_check=%s homing_mode=%s",
                     ", ".join([c.name for c in self.carriages]),
                     self.a_joint_home_x, self.b_joint_home_x, self.d_x,
                     self.max_velocity, self.max_accel,
                     self.travel_check, self.homing_mode)
    def get_steppers(self):
        return [c.stepper for c in self.carriages]
    def set_position(self, newpos):
        # The carriages recompute their own position from the bed pose -
        # this is what makes homing them unnecessary
        for c in self.carriages:
            c.stepper.set_position(newpos)
    def calc_carriage_positions(self, pos):
        # Derived slide positions, using the same C solver the step
        # generator uses
        return [c.stepper.calc_position_from_coord(pos)
                for c in self.carriages]
    # Homing
    def start_homing(self):
        if self.homing_mode != 'detached':
            return
        toolhead = self.kin.toolhead
        self.homing_trapq = toolhead.get_trapq()
        toolhead.flush_step_generation()
        for c in self.carriages:
            c.stepper.set_trapq(None)
    def finish_homing(self):
        if self.homing_mode != 'detached' or self.homing_trapq is None:
            return
        toolhead = self.kin.toolhead
        toolhead.flush_step_generation()
        curpos = toolhead.get_position()
        for c in self.carriages:
            c.stepper.set_trapq(self.homing_trapq)
            # Re-derive from the real bed pose.  set_position keeps the
            # MCU step count and moves the offset, so no steps are emitted.
            c.stepper.set_position(curpos)
        self.homing_trapq = None
    # Move checking
    def _sample_count(self, move):
        if self.travel_check == 'endpoints':
            return 1
        # The derived position depends on the bed pose only through
        # z_mz = Z - (A+B)/2 and z_ab = B - A, so scale the sampling to
        # how far those two actually travel
        axes_d = move.axes_d
        z_axis, a_axis, b_axis = self.bed_axes
        d_z_ab = abs(axes_d[b_axis] - axes_d[a_axis])
        d_z_mz = abs(axes_d[z_axis]
                     - 0.5 * (axes_d[a_axis] + axes_d[b_axis]))
        n = int(math.ceil(max(d_z_ab, d_z_mz) / UV_SAMPLE_MM))
        return max(1, min(n, UV_MAX_SAMPLES))
    def _check_range(self, move, positions):
        for c, p in zip(self.carriages, positions):
            if p < c.position_min or p > c.position_max:
                raise move.move_error(
                    "%s would reach %.3f (limit %.3f..%.3f)"
                    % (c.name, p, c.position_min, c.position_max))
    def check_move(self, move):
        # Range-check the derived carriages along the move and return the
        # speed and acceleration limits their travel implies, or None if
        # the bed pose does not change.
        bed_moved = any([move.axes_d[axis] for axis in self.bed_axes])
        if not bed_moved:
            return None
        if self.kin.is_homing:
            # The start position of a homing move is a forced fiction, so
            # a derived position computed from it is meaningless
            return None
        n = self._sample_count(move)
        start_pos, end_pos = move.start_pos, move.end_pos
        prev = self.calc_carriage_positions(start_pos)
        self._check_range(move, prev)
        # peak is the largest |dU/ds|, where s is the normalised move
        # parameter.  That is the quantity that bounds the move time,
        # because s advances linearly in time while cruising.
        peak = 0.
        for k in range(1, n + 1):
            if k == n:
                pos = end_pos          # exact endpoint, no float drift
            else:
                f = float(k) / n
                pos = [s + (e - s) * f for s, e in zip(start_pos, end_pos)]
            cur = self.calc_carriage_positions(pos)
            self._check_range(move, cur)
            for i in range(len(cur)):
                rate = abs(cur[i] - prev[i]) * n
                if rate > peak:
                    peak = rate
            prev = cur
        if peak <= 0.:
            return None
        # The peak carriage speed is peak/t for a move that takes time t,
        # so holding it at max_velocity gives the minimum time the
        # carriages need; the move distance over that time is the speed
        # cap.  The accel cap scales the same way an axis ratio does.
        ratio = move.move_d / peak
        return self.max_velocity * ratio, self.max_accel * ratio
    def get_status(self, eventtime):
        return {'bed_joint_carriages': {
            c.name: c.stepper.get_commanded_position()
            for c in self.carriages}}
