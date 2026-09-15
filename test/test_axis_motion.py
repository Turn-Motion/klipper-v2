"""Exercise the axis roster, the distance groups and check_move().

Covers the parts of the generic kinematics engine that a batch-mode smoke
run cannot assert:

  * the move distance is the largest distance group, not the euclidean
    distance over every axis
  * a stock three axis kinematics still measures the upstream euclidean
    distance, so nothing about the old machines changes
  * each group is limited by its own velocity and acceleration, and the
    group that needs the most time sets the speed of the whole move
  * a pure gantry move is not slowed by the bed limits
  * a rotary axis in "coordinated" mode may extend a combined move; in
    "slaved" mode it may not, but still moves (and is still limited)
    when it is the only axis moving
  * the derived bed joint carriages reject a move that would take a
    carriage off its rail, including one whose endpoints are both in
    range but which passes outside partway through

Runs standalone with plain Python - no compiler, no printer:

    python test/test_axis_motion.py
"""
import sys, os, types, math

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'klippy'))
sys.path.insert(0, os.path.join(HERE, '..', 'klippy', 'extras'))

# Stub the C-dependent modules so the kinematics import without cffi
for _name in ('mcu', 'chelper'):
    sys.modules[_name] = types.ModuleType(_name)
sys.modules['chelper'].get_ffi = lambda: (None, None)

import toolhead                                        # noqa: E402
from kinematics import axis_kinematics as ak           # noqa: E402

FAIL = []


def ok(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAIL.append(msg)


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol * max(1.0, abs(a), abs(b))


# ── Geometry of the reference machine ────────────────────────────────────────
A_HOME_X, B_HOME_X = -23.0, 623.0
D_X = B_HOME_X - A_HOME_X
MID_X = 0.5 * (A_HOME_X + B_HOME_X)

MAX_VELOCITY, MAX_ACCEL = 100., 3000.
MAX_Z_VELOCITY, MAX_Z_ACCEL = 8.4, 300.
MAX_R_VELOCITY, MAX_R_ACCEL = 90., 800.


def derived_uv(pos):
    """Same equations as kin_bedjoint.c, in slide coordinates."""
    z, a, b = pos[2], pos[3], pos[4]
    z_mz = z - 0.5 * (a + b)
    z_ab = b - a
    rad = max(D_X * D_X - z_ab * z_ab, 0.01 * D_X * D_X)
    csx = math.sqrt(rad)
    common = MID_X + z_mz * z_ab / csx
    return (common - 0.5 * csx - A_HOME_X, common + 0.5 * csx - B_HOME_X)


class FakeError(Exception):
    pass


class FakePrinter(object):
    command_error = FakeError

    def lookup_object(self, name, default=None):
        return default


class FakeToolhead(object):
    """Just enough toolhead for Move() and the engine's check_move()."""
    printer = FakePrinter()
    max_velocity = MAX_VELOCITY
    max_accel = MAX_ACCEL
    junction_deviation = 0.05
    mcr_pseudo_accel = MAX_ACCEL * 0.5
    extra_axes = []

    def __init__(self, axis_names, distance_groups):
        self.axis_names = axis_names
        self.axis_count = len(axis_names)
        self.distance_groups = distance_groups

    def get_max_velocity(self):
        return MAX_VELOCITY, MAX_ACCEL

    def get_max_z_velocity(self):
        return MAX_Z_VELOCITY, MAX_Z_ACCEL


class FakeRail(object):
    def __init__(self, name, position_min, position_max):
        self.name = name
        self.range = (position_min, position_max)

    def get_range(self):
        return self.range


class FakeAxisDef(object):
    def __init__(self, name, group):
        self.name = name
        self.group = group
        self.section = 'stepper_' + name


def build_kin(axis_specs, distance_groups, derived=None):
    """Build an AxisKinematics without touching config or steppers."""
    kin = ak.AxisKinematics.__new__(ak.AxisKinematics)
    kin.printer = FakePrinter()
    kin.kin_name = 'test'
    kin.axis_names = "".join([n for n, g, lo, hi in axis_specs])
    kin.toolhead = FakeToolhead(kin.axis_names, distance_groups)
    kin.axes = []
    for index, (name, group, lo, hi) in enumerate(axis_specs):
        axis = ak.Axis(FakeAxisDef(name, group), index,
                       FakeRail('stepper_' + name, lo, hi), True)
        axis.limits = (lo, hi)          # pretend every axis is homed
        if group == ak.ROTARY:
            axis.max_velocity = MAX_R_VELOCITY
            axis.max_accel = MAX_R_ACCEL
        kin.axes.append(axis)
    kin.axis_by_name = {a.name: a for a in kin.axes}
    kin.rails = [a.rail for a in kin.axes]
    kin.distance_groups = distance_groups
    kin.slaved_axes = kin._slaved_axes()
    kin.dc_module = None
    kin.dual_carriage_axis = None
    kin.safety_module = None
    kin.derived = derived
    kin.is_homing = kin.is_probing = False
    return kin


XY3Z_AXES = [('x', ak.PLANAR, -32.5, 570.), ('y', ak.PLANAR, -97.4, 280.),
             ('z', ak.VERTICAL, -90., 438.8), ('a', ak.VERTICAL, -90., 439.),
             ('b', ak.VERTICAL, -90., 439.8)]
XY3Z_GROUPS = [(0, 1), (2,), (3,), (4,)]


def make_move(kin, start, end, speed=MAX_VELOCITY):
    th = kin.toolhead
    naxes = th.axis_count
    start = list(start) + [0.] * (naxes + 1 - len(start))
    end = list(end) + [0.] * (naxes + 1 - len(end))
    return toolhead.Move(th, start, end, speed)


# ── 1. Move distance follows the kinematics' distance groups ────────────────
print("Move distance")
print("-" * 70)
kin = build_kin(XY3Z_AXES, XY3Z_GROUPS)
m = make_move(kin, [0.] * 6, [30., 40., 0., 0., 0., 0.])
ok(close(m.move_d, 50.), "pure XY move: distance is the XY norm (%.4f)"
   % (m.move_d,))
m = make_move(kin, [0.] * 6, [30., 40., 10., 0., 0., 0.])
ok(close(m.move_d, 50.),
   "XY plus a smaller Z: the XY group still sets the distance (%.4f)"
   % (m.move_d,))
m = make_move(kin, [0.] * 6, [3., 4., 10., 0., 0., 0.])
ok(close(m.move_d, 10.),
   "XY plus a larger Z: the Z group sets the distance (%.4f)" % (m.move_d,))
m = make_move(kin, [0.] * 6, [0., 0., 5., 8., 3., 0.])
ok(close(m.move_d, 8.),
   "bed tilt: the largest single bed motor sets the distance (%.4f)"
   % (m.move_d,))
m = make_move(kin, [0.] * 6, [0., 0., 0., 0., 0., 7.])
ok(close(m.move_d, 7.) and not m.is_kinematic_move,
   "extrude only move is unchanged (%.4f)" % (m.move_d,))

# The stock three axis behaviour must be untouched
print()
print("Stock three axis kinematics keep the upstream euclidean distance")
print("-" * 70)
stock = build_kin([('x', ak.PLANAR, -10., 200.), ('y', ak.PLANAR, -10., 200.),
                   ('z', ak.VERTICAL, -10., 200.)], [(0, 1, 2)])
m = make_move(stock, [0.] * 4, [3., 4., 12., 0.])
ok(close(m.move_d, 13.),
   "one group over x, y and z gives sqrt(x^2+y^2+z^2) (%.4f)" % (m.move_d,))

# ── 2. Speed limiting per group ─────────────────────────────────────────────
print()
print("Speed limiting")
print("-" * 70)
kin = build_kin(XY3Z_AXES, XY3Z_GROUPS)
m = make_move(kin, [0.] * 6, [100., 0., 0., 0., 0., 0.])
kin.check_move(m)
ok(close(math.sqrt(m.max_cruise_v2), MAX_VELOCITY)
   and close(m.accel, MAX_ACCEL),
   "a pure gantry move keeps the printer defaults (%.3f mm/s, %.1f mm/s^2)"
   % (math.sqrt(m.max_cruise_v2), m.accel))

m = make_move(kin, [0.] * 6, [0., 0., 10., 0., 0., 0.])
kin.check_move(m)
ok(close(math.sqrt(m.max_cruise_v2), MAX_Z_VELOCITY)
   and close(m.accel, MAX_Z_ACCEL),
   "a pure Z move runs at the bed limits (%.3f mm/s, %.1f mm/s^2)"
   % (math.sqrt(m.max_cruise_v2), m.accel))

# XY 100mm with Z 10mm: the move takes as long as the slower group needs
m = make_move(kin, [0.] * 6, [100., 0., 10., 0., 0., 0.])
kin.check_move(m)
v = math.sqrt(m.max_cruise_v2)
t_xy = 100. / MAX_VELOCITY
t_z = 10. / MAX_Z_VELOCITY
expect_v = m.move_d / max(t_xy, t_z)
ok(close(v, expect_v),
   "a combined move is as slow as its slowest group (%.4f vs %.4f mm/s)"
   % (v, expect_v))
ok(v < MAX_VELOCITY, "and that is slower than the gantry alone")

# ── 3. Range checking ───────────────────────────────────────────────────────
print()
print("Range checking")
print("-" * 70)
m = make_move(kin, [0.] * 6, [600., 0., 0., 0., 0., 0.])
try:
    kin.check_move(m)
    ok(False, "a move past position_max is rejected")
except FakeError as e:
    ok("out of range" in str(e).lower(),
       "a move past position_max is rejected (%s)" % (str(e)[:40],))
m = make_move(kin, [0.] * 6, [0., 0., 500., 0., 0., 0.])
try:
    kin.check_move(m)
    ok(False, "a bed move past position_max is rejected")
except FakeError:
    ok(True, "a bed move past position_max is rejected")

# ── 4. A rotary axis ────────────────────────────────────────────────────────
print()
print("Rotary axis, coordinated")
print("-" * 70)
XY3ZR_AXES = XY3Z_AXES + [('r', ak.ROTARY, -100000., 100000.)]
kinr = build_kin(XY3ZR_AXES, XY3Z_GROUPS + [(5,)])
m = make_move(kinr, [0.] * 7, [0., 0., 0., 0., 0., 90., 0.])
ok(close(m.move_d, 90.), "R alone: the distance is the angle (%.3f)"
   % (m.move_d,))
kinr.check_move(m)
ok(close(math.sqrt(m.max_cruise_v2), MAX_R_VELOCITY)
   and close(m.accel, MAX_R_ACCEL),
   "R alone runs at the rotary limits (%.3f deg/s, %.1f deg/s^2)"
   % (math.sqrt(m.max_cruise_v2), m.accel))

# X 100mm would take 1s; R 180deg at 90deg/s takes 2s - R must win
m = make_move(kinr, [0.] * 7, [100., 0., 0., 0., 0., 180., 0.])
kinr.check_move(m)
v = math.sqrt(m.max_cruise_v2)
expect_v = m.move_d / max(100. / MAX_VELOCITY, 180. / MAX_R_VELOCITY)
ok(close(v, expect_v),
   "coordinated: R may extend a combined move (%.4f vs %.4f)"
   % (v, expect_v))

print()
print("Rotary axis, slaved")
print("-" * 70)
kins = build_kin(XY3ZR_AXES, XY3Z_GROUPS)      # r in no group
ok([a.name for a in kins.slaved_axes] == ['r'],
   "an axis in no distance group is reported as slaved")
m = make_move(kins, [0.] * 7, [100., 0., 0., 0., 0., 180., 0.])
ok(close(m.move_d, 100.),
   "slaved: a combined move is measured by the linear axes alone (%.4f)"
   % (m.move_d,))
kins.check_move(m)
ok(close(math.sqrt(m.max_cruise_v2), MAX_VELOCITY),
   "slaved: R does not slow a combined move (%.4f mm/s)"
   % (math.sqrt(m.max_cruise_v2),))
m = make_move(kins, [0.] * 7, [0., 0., 0., 0., 0., 90., 0.])
ok(close(m.move_d, 90.) and m.is_kinematic_move,
   "slaved: R still moves when it is the only axis (%.3f)" % (m.move_d,))
kins.check_move(m)
ok(close(math.sqrt(m.max_cruise_v2), MAX_R_VELOCITY),
   "slaved: R alone is still held to the rotary limits (%.3f deg/s)"
   % (math.sqrt(m.max_cruise_v2),))

# ── 5. Derived bed joint carriages ──────────────────────────────────────────
print()
print("Derived bed joint carriages")
print("-" * 70)
from kinematics import bed_joint_carriages as bjc     # noqa: E402


class FakeStepper(object):
    def __init__(self, index):
        self.index = index

    def calc_position_from_coord(self, coord):
        return derived_uv(coord)[self.index]


def build_derived(kin, travel_check, position_min, position_max,
                  v_limits=None):
    d = bjc.BedJointCarriages.__new__(bjc.BedJointCarriages)
    d.printer = FakePrinter()
    d.kin = kin
    d.bed_axes = [2, 3, 4]
    d.travel_check = travel_check
    d.homing_mode = 'detached'
    d.max_velocity = MAX_Z_VELOCITY
    d.max_accel = MAX_Z_ACCEL
    v_min, v_max = v_limits or (position_min, position_max)
    d.carriages = [bjc.Carriage('stepper_u', FakeStepper(0),
                                position_min, position_max),
                   bjc.Carriage('stepper_v', FakeStepper(1),
                                v_min, v_max)]
    d.homing_trapq = None
    return d


kin = build_kin(XY3Z_AXES, XY3Z_GROUPS)
kin.derived = build_derived(kin, 'sampled', -120., 120.)
m = make_move(kin, [0.] * 6, [100., 50., 0., 0., 0., 0.])
kin.check_move(m)
ok(close(math.sqrt(m.max_cruise_v2), MAX_VELOCITY),
   "a pure gantry move does not move the carriages, so it is not limited")

# A pitch-only move (A == B) leaves the carriages where they are
m = make_move(kin, [0.] * 6, [0., 0., 8., 8., 8., 0.])
kin.check_move(m)
u0, v0 = derived_uv([0., 0., 8., 8., 8.])
ok(abs(u0) < 1e-9 and abs(v0) < 1e-9,
   "a pitch only move produces no carriage travel")

# A roll that takes a carriage past its rail must be rejected
kin.derived = build_derived(kin, 'sampled', -1., 1.)
m = make_move(kin, [0.] * 6, [0., 0., 0., -60., 60., 0.])
try:
    kin.check_move(m)
    ok(False, "a roll that overruns a carriage rail is rejected")
except FakeError as e:
    ok("stepper_" in str(e), "a roll that overruns a carriage rail is"
       " rejected (%s)" % (str(e)[:60],))

# Interior extremum: both endpoints in range, the middle is not
# U dips to 0 in the middle of a roll that sweeps through level, so a
# rail that starts above 0 is overrun partway through a move whose two
# endpoints are both comfortably inside it.
kin.derived = build_derived(kin, 'sampled', 5., 40., v_limits=(-40., -5.))
start = [0., 0., 100., 20., 180., 0.]
end = [0., 0., 100., 180., 20., 0.]
u_start, v_start = derived_uv(start)
u_end, v_end = derived_uv(end)
u_mid, v_mid = derived_uv([0., 0., 100., 100., 100., 0.])
print("    carriage U: start %.3f  middle %.3f  end %.3f"
      % (u_start, u_mid, u_end))
m = make_move(kin, start, end)
sampled_rejected = False
try:
    kin.check_move(m)
except FakeError:
    sampled_rejected = True
kin.derived = build_derived(kin, 'endpoints', 5., 40.,
                            v_limits=(-40., -5.))
m = make_move(kin, start, end)
endpoints_rejected = False
try:
    kin.check_move(m)
except FakeError:
    endpoints_rejected = True
interior_only = (min(u_start, u_end) > 5. and max(u_start, u_end) < 40.
                 and (u_mid < 5. or u_mid > 40.))
if interior_only:
    ok(sampled_rejected, "sampled: an interior rail overrun is caught")
    ok(not endpoints_rejected,
       "endpoints: the same move passes, which is why sampled is the default")
else:
    ok(sampled_rejected == endpoints_rejected,
       "both travel checks agree when there is no interior extremum")

# The carriage rate caps the move speed
kin.derived = build_derived(kin, 'sampled', -400., 400.)
m = make_move(kin, [0.] * 6, [0., 0., 0., -80., 80., 0.])
kin.check_move(m)
v = math.sqrt(m.max_cruise_v2)
ok(v <= MAX_Z_VELOCITY + 1e-9,
   "a large roll is capped at or below the carriage limit (%.4f mm/s)" % (v,))

print()
if FAIL:
    print("%d FAILURE(S):" % (len(FAIL),))
    for f in FAIL:
        print("  ! " + f)
    sys.exit(1)
print("All axis motion checks passed.")
