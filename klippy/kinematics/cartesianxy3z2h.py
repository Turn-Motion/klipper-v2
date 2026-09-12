# Cartesian XY3Z kinematics with derived horizontal bed-joint carriages
#
#   X, Y     nozzle gantry
#   Z, A, B  the three vertical bed actuators (define the bed plane)
#   U, V     the two horizontal bed-joint carriages - DERIVED, not axes
#
# The bed is a rigid plate on three ball joints.  The A and B joints slide
# horizontally in X; that slide is fully determined by the commanded (Z, A, B)
# through plate rigidity, so U and V are not degrees of freedom.  They are
# modelled as steppers with a non-trivial position function - the same way
# delta towers are - rather than as G-code axes:
#
#   * G-code keeps sending X Y Z A B; there is no U or V word.
#   * The position tuple stays [X, Y, Z, A, B, E]; nothing else in the host
#     code needs to change.
#   * U/V follow every Z/A/B transform (z_tilt, SET_GCODE_OFFSET, bed_mesh,
#     probe lifts) automatically, because they are recomputed from whatever
#     Z/A/B actually are.
#   * Homing is free: toolhead.set_position() -> stepper.set_position() ->
#     itersolve_set_position() -> the derived callback.
#
# The equations live in chelper/kin_cartesianxy3z2h.c and are verified against
# the reference printerkin.solver implementation in test/test_uv_derived.py.
#
# Copyright (C) 2024-2026  Turn-Motion
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper
from . import cartesianxy3z

# Derived-carriage sampling.  The carriages move nonlinearly with the bed
# pose: for pure roll the joint traces a circle of radius d_x/2, so the gain
# d(U)/d(B-A) grows from 0 at level to ~0.2 at the edge of the envelope.  A
# straight chord between the move's endpoints therefore understates both the
# peak rate (the end of a long sweep runs ~2x faster than the chord average)
# and the extreme position (U(s) can have an interior extremum, so a move
# whose endpoints are both in range can still take a carriage past its rail
# limit partway through).  So the derived position is sampled along the
# move; the sample count scales with how far the bed pose actually changes,
# which keeps an ordinary small bed adjustment at the two endpoint
# evaluations.  This is purely a host-side calculation for the speed cap and
# the range check - step placement is still solved exactly by itersolve.
UV_SAMPLE_MM = 10.0      # bed-pose travel (mm) per interior sample
UV_MAX_SAMPLES = 16

class CartesianXY3Z2H(cartesianxy3z.CartesianXY3Z):
    def __init__(self, toolhead, config):
        # The derived carriages: [(name, mcu_stepper, min, max)].  Set up
        # first because the base constructor already calls get_steppers().
        self.h_steppers = []
        cartesianxy3z.CartesianXY3Z.__init__(self, toolhead, config)
        # Geometry: the world X of each ball joint with the bed level.  d_x
        # and the midpoint follow from these.
        hcfg = config
        if config.has_section('cartesianxy3z2h'):
            hcfg = config.getsection('cartesianxy3z2h')
        self.a_joint_home_x = hcfg.getfloat('a_joint_home_x', -23.0)
        self.b_joint_home_x = hcfg.getfloat('b_joint_home_x', 623.0)
        if self.b_joint_home_x <= self.a_joint_home_x:
            raise config.error(
                "b_joint_home_x must be greater than a_joint_home_x")
        self.d_x = self.b_joint_home_x - self.a_joint_home_x
        self.enable_uv = hcfg.getboolean('enable_horizontal_carriages', True)
        # Velocity/accel limits for the horizontal carriages.  Default to the
        # vertical bed limits so an existing config needs no new keys.
        max_z_velocity, max_z_accel = toolhead.get_max_z_velocity()
        self._max_h_velocity = hcfg.getfloat('max_h_velocity', max_z_velocity,
                                             above=0.)
        self._max_h_accel = hcfg.getfloat('max_h_accel', max_z_accel,
                                          above=0.)
        if self.enable_uv:
            for name, side in (('stepper_u', -1.0), ('stepper_v', 1.0)):
                if not config.has_section(name):
                    raise config.error(
                        "Section '%s' is required by the cartesianxy3z2h"
                        " kinematics (set enable_horizontal_carriages: False"
                        " to run without the horizontal carriages)" % (name,))
                scfg = config.getsection(name)
                s = stepper.PrinterStepper(scfg)
                s.setup_itersolve('cartesianxy3z2h_h_stepper_alloc',
                                  self.a_joint_home_x, self.b_joint_home_x,
                                  side)
                # Slide travel limits (0 == the level-bed home position).
                # REQUIRED, deliberately: the carriages have no endstop and
                # no encoder, so this range check is the only thing standing
                # between a commanded bed pose and the joint being driven off
                # the end of its rail by the (full-power) Z/A/B actuators.
                smin = scfg.getfloat('position_min')
                smax = scfg.getfloat('position_max', above=smin)
                s.set_trapq(toolhead.get_trapq())
                self.h_steppers.append((name, s, smin, smax))
        logging.info("CartesianXY3Z2H: horizontal carriages %s,"
                     " a_joint_home_x=%.3f b_joint_home_x=%.3f (d_x=%.3f),"
                     " max_h_velocity=%.1f max_h_accel=%.1f",
                     "enabled" if self.h_steppers else "disabled",
                     self.a_joint_home_x, self.b_joint_home_x, self.d_x,
                     self._max_h_velocity, self._max_h_accel)
    # Stepper / position plumbing
    def get_steppers(self):
        res = cartesianxy3z.CartesianXY3Z.get_steppers(self)
        res.extend([s for _, s, _, _ in self.h_steppers])
        return res
    def get_derived_steppers(self):
        # Steppers whose position follows Z/A/B but which are not axes
        # (bed levelling and probing must not treat them as Z steppers)
        return [s for _, s, _, _ in self.h_steppers]
    def calc_position(self, stepper_positions):
        # Only the primary axis rails contribute - U/V are outputs, never
        # inputs, and including them would over-determine the back-solve.
        return cartesianxy3z.CartesianXY3Z.calc_position(
            self, stepper_positions)
    def set_position(self, newpos, homing_axes):
        cartesianxy3z.CartesianXY3Z.set_position(self, newpos, homing_axes)
        # The derived carriages recompute their own position from Z/A/B -
        # this is what makes homing them unnecessary.
        for _, s, _, _ in self.h_steppers:
            s.set_position(newpos)
    def calc_uv(self, pos):
        # Derived (U, V) slide positions for a toolhead position, using the
        # same C solver the step generator uses.
        return [s.calc_position_from_coord(pos) for _, s, _, _ in
                self.h_steppers]
    # Homing
    def home(self, homing_state):
        # Home the real axes with the derived carriages detached.
        #
        # Klipper homes by first forcing the start position 1.5x beyond the
        # axis range so the move is guaranteed to reach the endstop.  For a
        # single bed axis that is a pose the rigid plate cannot physically
        # occupy, and the derived carriage position computed from it is
        # meaningless.  Driving U/V from that fiction would slam them across
        # their whole travel, so detach them from the motion queue for the
        # duration: a stepper without a trapq simply does not move.
        #
        # This is safe precisely because the position is derived.  The plate
        # back-drives the carriages while the bed homes, and set_position()
        # afterwards re-syncs the model to the true post-homing pose without
        # emitting steps.
        toolhead = self.toolhead
        trapq = toolhead.get_trapq()
        if self.h_steppers:
            toolhead.flush_step_generation()
            for _, s, _, _ in self.h_steppers:
                s.set_trapq(None)
        try:
            cartesianxy3z.CartesianXY3Z.home(self, homing_state)
        finally:
            if self.h_steppers:
                toolhead.flush_step_generation()
                curpos = toolhead.get_position()
                for _, s, _, _ in self.h_steppers:
                    s.set_trapq(trapq)
                    s.set_position(curpos)
    # Move checking
    def _uv_sample_count(self, move):
        # The derived position depends on the bed pose only through
        # z_mz = Z - (A+B)/2 and z_ab = B - A, so scale the sampling to how
        # far those two actually travel.
        axes_d = move.axes_d
        d_z_ab = abs(axes_d[4] - axes_d[3])
        d_z_mz = abs(axes_d[2] - 0.5 * (axes_d[3] + axes_d[4]))
        n = int(math.ceil(max(d_z_ab, d_z_mz) / UV_SAMPLE_MM))
        return max(1, min(n, UV_MAX_SAMPLES))
    def _check_uv_range(self, move, uv):
        for (name, _, smin, smax), p in zip(self.h_steppers, uv):
            if p < smin or p > smax:
                raise move.move_error(
                    "%s would reach %.3f (limit %.3f..%.3f)"
                    % (name, p, smin, smax))
    def _check_horizontal_carriages(self, move):
        # Range-check the derived carriages along the move and return the
        # peak |dU/ds| per carriage, where s is the normalised move
        # parameter (0 at start_pos, 1 at end_pos).  For a linear axis that
        # is just the total travel; for the derived carriages it is the
        # quantity that bounds the move time, because s advances linearly
        # in time while cruising.
        if not self.h_steppers:
            return []
        n = self._uv_sample_count(move)
        start_pos, end_pos = move.start_pos, move.end_pos
        prev = self.calc_uv(start_pos)
        self._check_uv_range(move, prev)
        peak = [0.] * len(self.h_steppers)
        for k in range(1, n + 1):
            if k == n:
                pos = end_pos          # exact endpoint, no float drift
            else:
                f = float(k) / n
                pos = [s + (e - s) * f for s, e in zip(start_pos, end_pos)]
            cur = self.calc_uv(pos)
            self._check_uv_range(move, cur)
            for i in range(len(peak)):
                rate = abs(cur[i] - prev[i]) * n
                if rate > peak[i]:
                    peak[i] = rate
            prev = cur
        return peak
    def check_move(self, move):
        # The base class handles the axis limits, the safety module and the
        # XY / Z-A-B speed coordination
        cartesianxy3z.CartesianXY3Z.check_move(self, move)
        if not any(move.axes_d[i] for i in cartesianxy3z.VERTICAL_AXES):
            # Bed pose unchanged - U/V cannot move
            return
        if self.is_homing:
            # The start position of a homing move is a forced fiction (see
            # home()) and the carriages are detached from the trapq anyway
            return
        uv_rate = self._check_horizontal_carriages(move)
        if not uv_rate or max(uv_rate) <= 0.:
            return
        # uv_rate is the peak |dU/ds|; s covers the move in time t, so the
        # peak carriage speed is uv_rate / t.  Holding that at max_h_velocity
        # gives the minimum time the carriages need; the move distance over
        # that time is the corresponding speed cap.  The accel cap scales the
        # same way an ordinary axis ratio does (the path curvature adds a
        # term of ~0.5 mm/s^2 at this geometry, which is ignored).
        uv_peak = max(uv_rate)
        move.limit_speed(self._max_h_velocity * move.move_d / uv_peak,
                         self._max_h_accel * move.move_d / uv_peak)
    def get_status(self, eventtime):
        res = cartesianxy3z.CartesianXY3Z.get_status(self, eventtime)
        # U/V are not axes, so expose them separately for diagnostics
        if self.h_steppers:
            res['horizontal_carriages'] = {
                name: s.get_commanded_position()
                for name, s, _, _ in self.h_steppers}
        return res

def load_kinematics(toolhead, config):
    return CartesianXY3Z2H(toolhead, config)
