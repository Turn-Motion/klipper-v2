"""Exercise the g-code move transforms that adjust the bed.

bed_tilt, z_thermal_adjust and the bed_mesh move splitter each used to
carry their own copy of "the bed is axes 2, 3 and 4".  They now ask
axis_roster, and this test checks that they apply an adjustment to every
bed axis the kinematics reports - and to the Z axis alone on a stock
three axis machine, which is upstream's behaviour.

These modules have no upstream regression test, so this is their only
automated coverage.

Runs standalone with plain Python - no compiler, no printer:

    python test/test_bed_transforms.py
"""
import sys, os, types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'klippy'))
sys.path.insert(0, os.path.join(HERE, '..', 'klippy', 'extras'))

# Stub the C-dependent modules and the heavier extras imports so the
# transform modules import without cffi or a printer
for _name in ('mcu', 'chelper', 'pins'):
    sys.modules[_name] = types.ModuleType(_name)
sys.modules['chelper'].get_ffi = lambda: (None, None)

import axis_roster                                     # noqa: E402

FAIL = []


def ok(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAIL.append(msg)


def close_list(a, b, tol=1e-9):
    return len(a) == len(b) and all([abs(x - y) <= tol for x, y in zip(a, b)])


class FakeKin(object):
    def __init__(self, bed_axes):
        self._bed_axes = bed_axes

    def get_bed_axes(self):
        return list(self._bed_axes)


class FakeToolhead(object):
    def __init__(self, axis_names, position):
        self._axis_names = axis_names
        self.position = list(position)
        self.moved = None

    def get_axis_names(self):
        return self._axis_names

    def get_axis_count(self):
        return len(self._axis_names)

    def get_extruder_index(self):
        return len(self._axis_names)

    def get_kinematics(self):
        return self.kin

    def get_position(self):
        return list(self.position)

    def move(self, newpos, speed):
        self.moved = (list(newpos), speed)


class FakePrinter(object):
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.objects = {'toolhead': toolhead}

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


def make_printer(axis_names, bed_axes, position):
    th = FakeToolhead(axis_names, position)
    th.kin = FakeKin(bed_axes)
    return FakePrinter(th), th


# ── 1. axis_roster itself ───────────────────────────────────────────────────
print("axis_roster")
print("-" * 70)
printer, th = make_printer('xyzab', [2, 3, 4], [1., 2., 10., 11., 12., 5.])
ok(axis_roster.get_bed_axes(printer) == [2, 3, 4],
   "the kinematics' bed axes are used when it reports them")
ok(axis_roster.get_extruder_index(printer) == 5,
   "the extruder index is the axis count")
ok(close_list(axis_roster.bed_delta_position(printer, th.get_position(), 2.),
              [1., 2., 12., 13., 14., 5.]),
   "a bed delta moves every bed axis and nothing else")
ok(close_list(axis_roster.bed_target_position(printer, th.get_position(), 20.),
              [1., 2., 20., 21., 22., 5.]),
   "a bed target moves Z to the height and carries the rest by the delta")

stock_printer, stock_th = make_printer('xyz', [2], [1., 2., 10., 5.])
ok(axis_roster.get_bed_axes(stock_printer) == [2],
   "a stock kinematics reports the z axis alone")
ok(close_list(axis_roster.bed_delta_position(
    stock_printer, stock_th.get_position(), 2.), [1., 2., 12., 5.]),
   "and a bed delta then touches only Z, as upstream does")


class NoBedAxesKin(object):
    pass


plain, plain_th = make_printer('xyz', [2], [0., 0., 0., 0.])
plain_th.kin = NoBedAxesKin()
ok(axis_roster.get_bed_axes(plain) == [2],
   "a kinematics that says nothing still gets the z axis")

# ── 2. bed_tilt ─────────────────────────────────────────────────────────────
print()
print("bed_tilt")
print("-" * 70)
from extras import bed_tilt                            # noqa: E402


class FakeConfig(object):
    def __init__(self, values):
        self.values = values

    def getfloat(self, name, default=None, **kwargs):
        return self.values.get(name, default)


def make_bed_tilt(axis_names, bed_axes, position, extra=None):
    printer, th = make_printer(axis_names, bed_axes, position)
    bt = bed_tilt.BedTilt.__new__(bed_tilt.BedTilt)
    bt.printer = printer
    bt.toolhead = th
    bt.x_adjust = 0.01
    bt.y_adjust = 0.02
    bt.z_adjust = 0.5
    bt.extra_adjust = {}
    bt.bed_axes = [2]
    bt.config = FakeConfig(extra or {})
    bt.handle_connect()
    return bt, th


bt, th = make_bed_tilt('xyzab', [2, 3, 4], [100., 200., 10., 11., 12., 5.],
                       {'a_adjust': 0.7, 'b_adjust': 0.9})
tilt = 100. * 0.01 + 200. * 0.02
pos = bt.get_position()
ok(close_list(pos, [100., 200., 10. - tilt - 0.5, 11. - tilt - 0.7,
                    12. - tilt - 0.9, 5.]),
   "get_position removes the tilt and each axis' own adjust")
bt.move([100., 200., 10., 11., 12., 5.], 50.)
moved, speed = th.moved
ok(close_list(moved, [100., 200., 10. + tilt + 0.5, 11. + tilt + 0.7,
                      12. + tilt + 0.9, 5.]),
   "move adds them back on every bed axis")
ok(bt._adjust(3) == 0.7 and bt._adjust(4) == 0.9,
   "each extra bed axis reads its own <axis>_adjust option")

bt2, th2 = make_bed_tilt('xyzab', [2, 3, 4], [0., 0., 0., 0., 0., 0.])
ok(bt2._adjust(3) == bt2.z_adjust and bt2._adjust(4) == bt2.z_adjust,
   "an unset <axis>_adjust defaults to z_adjust")

bts, ths = make_bed_tilt('xyz', [2], [100., 200., 10., 5.])
pos = bts.get_position()
ok(close_list(pos, [100., 200., 10. - tilt - 0.5, 5.]),
   "a stock three axis machine is unaffected")

# ── 3. z_thermal_adjust ─────────────────────────────────────────────────────
print()
print("z_thermal_adjust")
print("-" * 70)
from extras import z_thermal_adjust                    # noqa: E402

printer, th = make_printer('xyzab', [2, 3, 4], [1., 2., 10., 11., 12., 5.])
zta = z_thermal_adjust.ZThermalAdjuster.__new__(
    z_thermal_adjust.ZThermalAdjuster)
zta.printer = printer
zta.z_adjust_mm = 0.25
zta.last_z_adjust_mm = 0.25
zta.off_above_z = -1.          # never recompute, just apply
adjusted = zta.calc_adjust([1., 2., 10., 11., 12., 5.])
ok(close_list(adjusted, [1., 2., 10.25, 11.25, 12.25, 5.]),
   "calc_adjust raises every bed axis")
ok(close_list(zta.calc_unadjust(adjusted), [1., 2., 10., 11., 12., 5.]),
   "calc_unadjust is its exact inverse")

stock_printer, stock_th = make_printer('xyz', [2], [1., 2., 10., 5.])
zta.printer = stock_printer
ok(close_list(zta.calc_adjust([1., 2., 10., 5.]), [1., 2., 10.25, 5.]),
   "a stock three axis machine adjusts Z alone")

# ── 4. bed_mesh move splitter ───────────────────────────────────────────────
print()
print("bed_mesh move splitter")
print("-" * 70)
from extras import bed_mesh                            # noqa: E402


class FlatMesh(object):
    def calc_z(self, x, y):
        return 0.4


class SplitterConfig(object):
    def getfloat(self, name, default=None, **kwargs):
        return default if default is not None else 0.025


sp = bed_mesh.MoveSplitter(SplitterConfig(), None)
sp.initialize(FlatMesh(), 0., bed_axes=[2, 3, 4])
sp.build_move([0., 0., 5., 6., 7., 0.], [10., 0., 5., 6., 7., 0.], 1.)
out = sp.split()
ok(close_list(out, [10., 0., 5.4, 6.4, 7.4, 0.]),
   "the mesh offset is applied to every bed axis (%s)"
   % (["%.2f" % (v,) for v in out],))

sp = bed_mesh.MoveSplitter(SplitterConfig(), None)
sp.initialize(FlatMesh(), 0., bed_axes=[2])
sp.build_move([0., 0., 5., 0.], [10., 0., 5., 0.], 1.)
out = sp.split()
ok(close_list(out, [10., 0., 5.4, 0.]),
   "and to Z alone on a stock three axis machine (%s)"
   % (["%.2f" % (v,) for v in out],))

print()
if FAIL:
    print("%d FAILURE(S):" % (len(FAIL),))
    for f in FAIL:
        print("  ! " + f)
    sys.exit(1)
print("All bed transform checks passed.")
