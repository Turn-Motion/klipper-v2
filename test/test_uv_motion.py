"""Exercise CartesianXY3Z2H.check_move() with the C bindings stubbed out.

Covers the parts of the derived-carriage kinematics that the batch-mode smoke
run cannot assert:

  * the three-group time-based limiter (XY / ZAB / UV)
  * range rejection of the DERIVED carriage positions
  * that a pure XY move is not slowed by the bed limits
  * that pitch-only moves produce no UV travel and so no UV limiting

Runs standalone with plain Python - no compiler, no printer:

    python test/test_uv_motion.py
"""
import sys, os, types, math

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'klippy'))
sys.path.insert(0, os.path.join(HERE, '..', 'klippy', 'extras'))

# Stub the C-dependent modules so toolhead/kinematics import without cffi.
for _name in ('mcu', 'chelper'):
    sys.modules[_name] = types.ModuleType(_name)
sys.modules['chelper'].get_ffi = lambda: (None, None)

import toolhead                                        # noqa: E402
from kinematics import cartesianxy3z as k5             # noqa: E402
from kinematics import cartesianxy3z2h as k2h          # noqa: E402

FAIL = []


def ok(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAIL.append(msg)


# ── Geometry (matches the reference machine) ─────────────────────────────────
A_HOME_X, B_HOME_X = -23.0, 623.0
D_X = B_HOME_X - A_HOME_X
MID_X = 0.5 * (A_HOME_X + B_HOME_X)


def derived_uv(pos):
    """Same equations as kin_cartesianxy3z2h.c, in slide coordinates."""
    z, a, b = pos[2], pos[3], pos[4]
    z_mz = z - 0.5 * (a + b)
    z_ab = b - a
    rad = max(D_X * D_X - z_ab * z_ab, 0.01 * D_X * D_X)
    csx = math.sqrt(rad)
    common = MID_X + z_mz * z_ab / csx
    return (common - 0.5 * csx - A_HOME_X, common + 0.5 * csx - B_HOME_X)


class FakeHStepper(object):
    """Stands in for the MCU_stepper wired to the derived C solver."""
    def __init__(self, index):
        self.index = index

    def calc_position_from_coord(self, coord):
        return derived_uv(coord)[self.index]


class FakeError(Exception):
    pass


class FakePrinter(object):
    def command_error(self, msg):
        return FakeError(msg)

    def lookup_object(self, name, default=None):
        return default


class FakeToolhead(object):
    printer = FakePrinter()
    max_velocity = 100.
    max_accel = 3000.
    max_z_velocity = 8.4
    max_z_accel = 300.
    junction_deviation = 0.05
    mcr_pseudo_accel = 1500.

    def get_max_velocity(self):
        return self.max_velocity, self.max_accel

    def get_max_z_velocity(self):
        return self.max_z_velocity, self.max_z_accel


TH = FakeToolhead()
ZERO = [0.] * 6          # X Y Z A B E - unchanged from master


def mk(end, start=None, speed=1e6):
    s = list(start if start is not None else ZERO)
    e = list(s)
    for i, v in end.items():
        e[i] = v
    mv = toolhead.Move(TH, s, e, speed)
    mv.toolhead = TH
    return mv


def make_kin(u_min=-120., u_max=120., h_vel=8.4, h_accel=300.,
             v_min=None, v_max=None):
    k = object.__new__(k2h.CartesianXY3Z2H)
    k.limits = [(-1000., 1000.)] * 5
    k.safety_module = None
    k.is_homing = False
    k.is_probing = False
    k.dc_module = None
    k.printer = TH.printer
    k.toolhead = TH
    k._max_h_velocity = h_vel
    k._max_h_accel = h_accel
    k.h_steppers = [
        ('stepper_u', FakeHStepper(0), u_min, u_max),
        ('stepper_v', FakeHStepper(1),
         u_min if v_min is None else v_min,
         u_max if v_max is None else v_max)]
    return k


def uv_chord(move):
    """The OLD endpoint-only travel, for comparison against the sampled peak."""
    s = derived_uv(move.start_pos)
    e = derived_uv(move.end_pos)
    return [abs(b - a) for a, b in zip(s, e)]


def uv_true_peak(move, samples=4000):
    """Peak |dU/ds| found by brute force, independent of the implementation."""
    sp, ep = move.start_pos, move.end_pos
    prev = derived_uv(sp)
    peak = [0., 0.]
    for k in range(1, samples + 1):
        f = float(k) / samples
        cur = derived_uv([a + (b - a) * f for a, b in zip(sp, ep)])
        for i in (0, 1):
            peak[i] = max(peak[i], abs(cur[i] - prev[i]) * samples)
        prev = cur
    return peak


def axis_speeds(mv):
    v = math.sqrt(mv.max_cruise_v2)
    return [abs(r) * v for r in mv.axes_r[:5]]


def axis_accels(mv):
    return [abs(r) * mv.accel for r in mv.axes_r[:5]]


print("Axis layout is unchanged from master (this is the point)")
print("-" * 70)
import gcode as gcode_mod                              # noqa: E402
_c = gcode_mod.Coord((1., 2., 3., 4., 5., 6.))
ok((_c.x, _c.y, _c.z, _c.a, _c.b, _c.e) == (1., 2., 3., 4., 5., 6.)
   and len(_c) == 6,
   "gcode.Coord is still (x, y, z, a, b, e) - no u/v fields")
ok(len(k5.AXES) == 5, "kinematics exposes 5 G-code axes")
ok(k5.AXES == 'xyzab', "axis letters are still 'xyzab'")
ok(issubclass(k2h.CartesianXY3Z2H, k5.CartesianXY3Z),
   "derived-carriage kinematics extends the 5-axis kinematics")

print()
print("Derived UV rate per move")
print("-" * 70)
kin = make_kin()
# Pure pitch: A == B, only Z differs -> zero carriage travel
m = mk({2: 20.}, start=[0., 0., 0., 0., 0., 0.])
uv = kin._check_horizontal_carriages(m)
ok(max(uv) < 1e-12, "pitch-only move -> zero UV rate (%.3e)" % max(uv))

# Pure roll: A and B differ -> real carriage travel
m = mk({3: -50., 4: 50.}, start=[0., 0., 0., 0., 0., 0.])
uv = kin._check_horizontal_carriages(m)
ok(uv[0] > 0., "roll move produces non-zero carriage rate")

print()
print("Sub-sampling: peak |dU/ds|, not the endpoint chord")
print("-" * 70)

# A move small enough to need no interior samples must reduce EXACTLY to the
# old endpoint-only behaviour.
m = mk({3: -1., 4: 1.}, start=[0., 0., 0., 0., 0., 0.])
ok(kin._uv_sample_count(m) == 1, "tiny bed move -> 1 sample (no extra cost)")
uv = kin._check_horizontal_carriages(m)
ok(abs(uv[0] - uv_chord(m)[0]) < 1e-12,
   "1 sample -> peak equals the chord exactly (unchanged behaviour)")

# A large sweep: the gain grows with roll, so the peak must exceed the chord.
m = mk({3: -100., 4: 100.}, start=[0., 0., 0., 0., 0., 0.])
n = kin._uv_sample_count(m)
uv = kin._check_horizontal_carriages(m)
chord = uv_chord(m)
true_peak = uv_true_peak(m)
ok(n == k2h.UV_MAX_SAMPLES,
   "200mm roll sweep -> sampling capped at %d" % k2h.UV_MAX_SAMPLES)
ok(uv[0] > 1.8 * chord[0],
   "peak rate is ~2x the chord (peak %.3f vs chord %.3f)" % (uv[0], chord[0]))
ok(abs(uv[0] - true_peak[0]) < 0.10 * true_peak[0],
   "%d samples resolve the true peak within 10%% (%.3f vs %.3f)"
   % (n, uv[0], true_peak[0]))
ok(uv[0] <= true_peak[0] + 1e-9,
   "sampled peak never overstates the true peak")

# The tighter rate must actually slow the move down.
kin = make_kin(h_vel=1.0)
m_peak = mk({3: -100., 4: 100.}, start=[0., 0., 0., 0., 0., 0.])
kin.check_move(m_peak)
t_peak = m_peak.move_d / math.sqrt(m_peak.max_cruise_v2)
ok(abs(t_peak - uv[0] / 1.0) < 1e-6,
   "move time is set by the PEAK rate (%.4f s)" % t_peak)
ok(t_peak > 1.8 * (chord[0] / 1.0),
   "chord-based limiting would have been ~2x too fast")

print()
print("Three-group time-based limiter (XY / ZAB / UV)")
print("-" * 70)

# Pure XY - must keep the full XY limits, no bed slowdown
kin = make_kin()
m = mk({0: 50., 1: 50.})
kin.check_move(m)
sp = axis_speeds(m)
ok(abs(math.hypot(sp[0], sp[1]) - 100.) < 1e-6,
   "pure XY keeps max_velocity 100 (got %.3f)" % math.hypot(sp[0], sp[1]))

# Coordinated Z/A/B - each bed axis at max_z_velocity
m = mk({2: 10., 3: 10., 4: 10.})
kin.check_move(m)
sp, ac = axis_speeds(m), axis_accels(m)
ok(abs(sp[2] - 8.4) < 1e-6 and abs(sp[3] - 8.4) < 1e-6,
   "coordinated Z/A/B: each bed axis at max_z_velocity 8.4")
ok(abs(ac[2] - 300.) < 1e-6, "coordinated Z/A/B: accel at max_z_accel 300")

# XY + Z: both groups must finish together
m = mk({0: 100., 2: 10.})
kin.check_move(m)
sp = axis_speeds(m)
t_xy = 100. / sp[0]
t_z = 10. / sp[2]
ok(abs(sp[2] - 8.4) < 1e-6, "XY100+Z10: Z at 8.4 mm/s")
ok(sp[0] <= 100. + 1e-6, "XY100+Z10: XY within max_velocity")
ok(abs(t_xy - t_z) < 1e-6,
   "XY100+Z10: groups finish together (%.4f vs %.4f s)" % (t_xy, t_z))

# A large roll with a deliberately slow UV limit must slow the whole move.
kin = make_kin(h_vel=0.05, h_accel=5.)
m = mk({3: -100., 4: 100.}, start=[0., 0., 0., 0., 0., 0.])
uv = kin._check_horizontal_carriages(m)
kin.check_move(m)
sp = axis_speeds(m)
t_zab = 100. / sp[3]
t_uv = uv[0] / 0.05
ok(sp[3] < 8.4,
   "slow UV limit drags the bed axes below max_z_velocity (%.4f)" % sp[3])
ok(abs(t_zab - t_uv) < 1e-6,
   "ZAB and UV groups finish together (%.4f vs %.4f s)" % (t_zab, t_uv))
ok(max(axis_accels(m)[2:5]) <= 300. + 1e-6,
   "bed accel still within max_z_accel")

# ...and with a generous UV limit the bed runs at its own maximum again
kin = make_kin(h_vel=1000., h_accel=100000.)
m = mk({3: -100., 4: 100.}, start=[0., 0., 0., 0., 0., 0.])
kin.check_move(m)
sp = axis_speeds(m)
ok(abs(sp[3] - 8.4) < 1e-6,
   "generous UV limit -> bed back at max_z_velocity (%.4f)" % sp[3])

print()
print("Range rejection of the DERIVED carriage positions")
print("-" * 70)

# A roll big enough to push a carriage past a tight limit
kin = make_kin(u_min=-0.5, u_max=0.5)
m = mk({3: -150., 4: 150.}, start=[0., 0., 0., 0., 0., 0.])
u_end = derived_uv([0., 0., 0., -150., 150.])[0]
try:
    kin.check_move(m)
    ok(False, "carriage beyond position_max is rejected")
except FakeError as e:
    ok('stepper_u' in str(e) or 'stepper_v' in str(e),
       "carriage beyond its limit is rejected (%s)" % (str(e)[:60],))
ok(abs(u_end) > 0.5, "the test pose really does exceed the limit (U=%.3f)"
   % u_end)

# The same move is accepted with the real limits
kin = make_kin(u_min=-120., u_max=120.)
m = mk({3: -150., 4: 150.}, start=[0., 0., 0., 0., 0., 0.])
try:
    kin.check_move(m)
    ok(True, "same move accepted with realistic limits (U=%.3f)" % u_end)
except FakeError as e:
    ok(False, "same move accepted with realistic limits (got %s)" % (e,))

# Pure XY must not be range-checked against the carriages at all
kin = make_kin(u_min=-0.0001, u_max=0.0001)
m = mk({0: 10., 1: 10.})
try:
    kin.check_move(m)
    ok(True, "pure XY move skips the carriage range check")
except FakeError as e:
    ok(False, "pure XY move skips the carriage range check (got %s)" % (e,))

print()
print("Interior extremum: both endpoints legal, midpoint is not")
print("-" * 70)
# Roll from B-A = +200 through 0 to -200 at zero pitch.  U is 15.87 at BOTH
# endpoints and dips to 0 halfway, so an endpoint-only check passes a move
# that takes the carriage outside its rail.
ROLL_START = [0., 0., 0., -100., 100., 0.]
ROLL_END = {3: 100., 4: -100.}
u_s = derived_uv(ROLL_START)[0]
u_e = derived_uv([0., 0., 0., 100., -100.])[0]
u_mid = derived_uv([0., 0., 0., 0., 0.])[0]
ok(abs(u_s - u_e) < 1e-9 and u_mid < u_s - 1.,
   "U dips mid-move: %.3f -> %.3f -> %.3f" % (u_s, u_mid, u_e))

kin = make_kin(u_min=5., u_max=120., v_min=-120., v_max=120.)
ok(u_s > 5. and u_e > 5.,
   "both endpoints satisfy position_min=5 (an endpoint check would pass)")
m = mk(ROLL_END, start=ROLL_START)
try:
    kin.check_move(m)
    ok(False, "interior excursion past position_min is caught")
except FakeError as e:
    ok('stepper_u' in str(e),
       "interior excursion past position_min is caught (%s)" % (str(e)[:60],))

# ...and the same move is fine once the limit actually covers the dip.
kin = make_kin(u_min=-120., u_max=120.)
m = mk(ROLL_END, start=ROLL_START)
try:
    kin.check_move(m)
    ok(True, "same move accepted when the limit covers the whole path")
except FakeError as e:
    ok(False, "same move accepted with real limits (got %s)" % (e,))

print()
print("Carriages absent (enable_horizontal_carriages: False)")
print("-" * 70)
kin = make_kin()
kin.h_steppers = []
m = mk({2: 10., 3: 10., 4: 10.})
try:
    kin.check_move(m)
    sp = axis_speeds(m)
    ok(abs(sp[2] - 8.4) < 1e-6,
       "no carriages -> plain ZAB limiting still correct (%.4f)" % sp[2])
except Exception as e:
    ok(False, "no carriages -> check_move works (got %s)" % (e,))
ok(kin.calc_uv([0.] * 6) == [], "calc_uv() returns [] when disabled")

print()
print("Homing poses are not range-checked against the carriages")
print("-" * 70)
# Klipper forces the homing start position 1.5x beyond the axis range, which
# for a single bed axis is a pose the rigid plate cannot occupy.  Homing A
# alone gives |B-A| = 792 against d_x = 646, and the derived U for that is
# ~5161mm.  That must not block homing - the carriages are detached from the
# trapq for the duration instead (see CartesianXY3Z2H.home).
A_FORCED = 439.0 - 1.5 * (439.0 - (-90.0))          # -354.5
HOMING_START = [0., 0., 438.8, A_FORCED, 437.8, 0.]
u_forced = derived_uv(HOMING_START)[0]
ok(abs(437.8 - A_FORCED) > D_X,
   "the forced homing pose really is infeasible (|B-A|=%.1f vs d_x=%.1f)"
   % (437.8 - A_FORCED, D_X))
ok(u_forced > 1000.,
   "and the derived U for it is nonsense (%.3f mm)" % u_forced)

kin = make_kin(u_min=-120., u_max=120.)
m = mk({3: 439.0}, start=HOMING_START)
kin.is_homing = False
try:
    kin.check_move(m)
    ok(False, "with is_homing False this pose IS rejected")
except FakeError:
    ok(True, "with is_homing False this pose is rejected (test is valid)")

kin.is_homing = True
try:
    kin.check_move(m)
    ok(True, "with is_homing True the same move is allowed through")
except FakeError as e:
    ok(False, "homing move must not be blocked (got %s)" % (str(e)[:70],))

print()
print("Carriage travel limits are REQUIRED config (source check)")
print("-" * 70)
# __init__ cannot be exercised here (it needs a real config and MCU), so
# guard the change at the source level the way test_uv_derived.py guards the
# C equations.  These limits are the only rail-end protection the derived
# carriages have, so a default would silently disable it.
_ksrc = open(os.path.join(HERE, '..', 'klippy', 'kinematics',
                          'cartesianxy3z2h.py')).read()
ok("scfg.getfloat('position_min')" in _ksrc,
   "position_min is read with no default (required)")
ok("scfg.getfloat('position_max', above=smin)" in _ksrc,
   "position_max is read with no default (required)")
ok("getfloat('position_min', -1e9)" not in _ksrc
   and "getfloat('position_max', 1e9" not in _ksrc,
   "the old permissive +/-1e9 defaults are gone")

print()
if FAIL:
    print("%d FAILURE(S):" % len(FAIL))
    for f in FAIL:
        print("  ! " + f)
    sys.exit(1)
print("All UV motion checks passed.")
