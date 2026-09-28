"""Check the direction of the coordinated bed levelling correction.

A three-screw bed is levelled by moving the screws relative to each other.
Getting that direction backwards does not fail loudly: it doubles the tilt
it was asked to remove, so the machine keeps printing and the bed keeps
getting worse. This test pins the direction down three ways.

  1. Replay upstream's detach / reattach walk and read off the net
     displacement per stepper. That is the definition the fitted
     adjustments are written against.
  2. Drive the fork's coordinated move with a stubbed toolhead and check
     it commands the same direction.
  3. Simulate a rigid plate on the machine's real screw geometry, probe
     it, fit it, correct it and re-probe. The correct sign converges in
     one pass; the wrong one diverges.

Runs standalone with plain Python - no compiler, no printer:

    python test/test_z_tilt.py
"""
import sys, os, types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'klippy'))
sys.path.insert(0, os.path.join(HERE, '..', 'klippy', 'extras'))

for _name in ('mcu', 'chelper', 'pins'):
    sys.modules[_name] = types.ModuleType(_name)
sys.modules['chelper'].get_ffi = lambda: (None, None)

import mathutil                                        # noqa: E402
import axis_roster                                     # noqa: E402

FAIL = []


def ok(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAIL.append(msg)


# Screw and probe geometry of the production machine, from printer.cfg
Z_POSITIONS = [(300., 323.), (-23., 20.5), (623., 20.5)]
PROBE_POINTS = [(300., 280.), (100., 80.), (500., 80.)]
BED_AXES = [2, 3, 4]


# ── 1. what upstream's walk actually does ───────────────────────────────────
def upstream_walk(adjustments):
    """Net physical displacement per stepper under upstream's
    ZAdjustHelper.adjust_steppers: detach every Z stepper, then reattach
    them one at a time while walking the toolhead."""
    n = len(adjustments)
    phys = [0.] * n
    attached = [False] * n
    z = 0.
    positions = sorted([(-a, i) for i, a in enumerate(adjustments)])
    z_low = z - positions[0][0]
    for k in range(n - 1):
        attached[positions[k][1]] = True
        newz = z_low + positions[k + 1][0]
        for m in range(n):
            if attached[m]:
                phys[m] += newz - z
        z = newz
    return phys


print("Upstream's walk defines the sign")
print("-" * 70)
adj = [0.4, 0.0, -0.4]
phys = upstream_walk(adj)
rel = [round(p - min(phys), 9) for p in phys]
want = [round(a - min(adj), 9) for a in adj]
print("    adjustments       %s" % ([round(a, 4) for a in adj],))
print("    displacement      %s" % ([round(p, 4) for p in phys],))
ok(rel == want,
   "each stepper is displaced by +adjustment, not -adjustment")
ok(rel == [0.8, 0.4, 0.0],
   "[+0.4, 0, -0.4] gives [+0.8, +0.4, 0] as documented")


# ── 2. the fork's coordinated move commands the same direction ──────────────
print()
print("The coordinated move matches it")
print("-" * 70)
from extras import z_tilt                              # noqa: E402


class FakeKin(object):
    def get_bed_axes(self):
        return list(BED_AXES)


class FakeToolhead(object):
    def __init__(self, position):
        self.position = list(position)
        self.moved = None
        self.kin = FakeKin()

    def get_kinematics(self):
        return self.kin

    def get_axis_names(self):
        return 'xyzab'

    def get_axis_count(self):
        return 5

    def get_position(self):
        return list(self.position)

    def move(self, newpos, speed):
        self.moved = list(newpos)

    def set_position(self, newpos):
        self.position = list(newpos)


class FakeGcodeMove(object):
    def __init__(self):
        self.base_position = [0.] * 6
        self.homing_position = [0.] * 6
        self.reset_calls = 0

    def reset_last_position(self):
        self.reset_calls += 1


class FakePrinter(object):
    def __init__(self, toolhead):
        self.objects = {'toolhead': toolhead, 'gcode_move': FakeGcodeMove()}

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


def run_adjust(adjustments, start=(0., 0., 10., 10., 10., 0.),
               offsets=None):
    th = FakeToolhead(start)
    printer = FakePrinter(th)
    gm = printer.objects['gcode_move']
    if offsets is not None:
        for axis, value in offsets.items():
            gm.homing_position[axis] = value
            gm.base_position[axis] = value
    helper = z_tilt.ZAdjustHelper.__new__(z_tilt.ZAdjustHelper)
    helper.printer = printer
    helper._adjust_axes(list(BED_AXES), list(adjustments), 5.)
    return th, gm


th, gm = run_adjust([0.4, 0.0, -0.4])
start = [0., 0., 10., 10., 10., 0.]
delta = [round(th.moved[a] - start[a], 9) for a in BED_AXES]
print("    adjustments       [0.4, 0.0, -0.4]")
print("    commanded delta   %s" % (delta,))
ok(delta == [0.4, 0.0, -0.4],
   "the screw with the largest adjustment moves the furthest in +Z")
ok(abs(sum(delta)) < 1e-9,
   "the mean is removed, so the bed is not shifted bodily")

th2, _ = run_adjust([0.1, 0.1, 0.1])
delta2 = [round(th2.moved[a] - start[a], 9) for a in BED_AXES]
ok(delta2 == [0., 0., 0.],
   "a uniform adjustment is no tilt at all, so nothing moves")


# ── 2b. the offsets it writes are state it does not own ─────────────────────
print()
print("Levelling preserves offsets it did not set")
print("-" * 70)

# Every bed axis carries the same offset the tool change applies, so the
# commanded coordinate is the same on all three before levelling runs.
TOOL_OFFSET = -0.8
th3, gm3 = run_adjust([0.4, 0.0, -0.4],
                      offsets={a: TOOL_OFFSET for a in BED_AXES})
moved3 = [th3.moved[a] for a in BED_AXES]
deltas3 = [round(m - start[a], 9) for m, a in zip(moved3, BED_AXES)]
after_hp = [round(gm3.homing_position[a], 9) for a in BED_AXES]
after_bp = [round(gm3.base_position[a], 9) for a in BED_AXES]
print("    offsets before    %s" % ([TOOL_OFFSET] * 3,))
print("    screws moved by   %s" % (deltas3,))
print("    offsets after     %s" % (after_hp,))

ok(after_hp == [round(TOOL_OFFSET + d, 9) for d in deltas3],
   "each offset shifts by exactly the distance its screw moved")
ok(abs(sum(after_hp) / 3. - TOOL_OFFSET) < 1e-9,
   "the tool offset survives levelling instead of being discarded")
ok(after_bp == after_hp,
   "base_position tracks it, so the commanded coordinate does not jump")

# Reported coordinate per axis is last_position - base_position.  Levelling
# must not change what the operator sees.
reported_before = [round(start[a] - TOOL_OFFSET, 9) for a in BED_AXES]
reported_after = [round(m - b, 9) for m, b in zip(moved3, after_bp)]
ok(reported_after == reported_before,
   "all three axes report the same coordinate they did before (%s)"
   % (reported_after,))

# With no prior offset the original invariant still holds.
th4, gm4 = run_adjust([0.4, 0.0, -0.4])
rep4 = [round(th4.moved[a] - gm4.base_position[a], 9) for a in BED_AXES]
ok(len(set(rep4)) == 1,
   "with no prior offset the three axes still agree (%s)" % (rep4,))


# ── 3. a plate on the real geometry converges ───────────────────────────────
print()
print("A simulated plate converges rather than diverging")
print("-" * 70)


def plate_height(screws, pt):
    eqs = [[1., x, y] for x, y in Z_POSITIONS]
    c = mathutil.solve_linear_equations(eqs, [[s] for s in screws])
    return c[0][0] + c[1][0] * pt[0] + c[2][0] * pt[1]


def fit(screws):
    # What z_tilt.probe_finalize computes from the probed points.  The bed
    # rides on the screws, so the recorded trigger height falls as the
    # plate rises.
    eqs, ans = [], []
    for p in PROBE_POINTS:
        eqs.append([1., p[0], p[1]])
        ans.append([-plate_height(screws, p)])
    res = mathutil.solve_linear_equations(eqs, ans,
                                          allow_underdetermined=True)
    za, xa, ya = res[0][0], res[1][0], res[2][0]
    return [x * xa + y * ya + za for x, y in Z_POSITIONS]


def out_of_level(screws):
    h = [plate_height(screws, p) for p in PROBE_POINTS]
    return max(h) - min(h)


def iterate(sign, screws, passes=3):
    s, hist = list(screws), []
    for _ in range(passes):
        a = fit(s)
        mean = sum(a) / len(a)
        s = [si + sign * (ai - mean) for si, ai in zip(s, a)]
        hist.append(out_of_level(s))
    return hist


plate = [0., 0., 0.8]        # the B screw sits 0.8 mm high
before = out_of_level(plate)
good = iterate(+1, plate)
bad = iterate(-1, plate)
print("    out of level before   %.6f mm" % (before,))
print("    with +adjustment      %s"
      % ("  ".join("%.6f" % h for h in good),))
print("    with -adjustment      %s"
      % ("  ".join("%.6f" % h for h in bad),))
ok(before > 0.1, "the simulated plate starts meaningfully out of level")
ok(good[0] < 1e-9, "+adjustment levels the plate in a single pass")
ok(all(g < 1e-9 for g in good), "and repeating it changes nothing")
ok(bad[-1] > before * 3, "-adjustment diverges instead")

print()
if FAIL:
    print("%d FAILURE(S):" % (len(FAIL),))
    for f in FAIL:
        print("  ! " + f)
    sys.exit(1)
print("All z_tilt direction checks passed.")
