"""Run a real Z_TILT_ADJUST against a simulated tilted bed.

test_z_tilt.py checks the levelling arithmetic in isolation.  This test
instead drives the whole command the way the operator does, through the
real g-code dispatcher, the real gcode_move, the real probe point helper
and the real manual probe helper.  Only the motion hardware is
simulated: a fake toolhead, a fake kinematics with five axes (x, y, z, a,
b), and a rigid plate resting on the machine's three bed screws.

That matters because the two things being checked are properties of the
whole path rather than of one function:

  * the correction has to reduce the tilt rather than double it, and the
    only definition of "tilt" that counts is the one the probe measures;
  * a session g-code offset set before levelling - a tool change offset,
    say - has to survive it, which depends on how the real gcode_move
    turns base_position into the coordinate the operator is shown.

The bed model is derived independently of test_z_tilt.py's, from the
measurement rather than from upstream's stepper walk, so agreement
between the two is evidence and not a shared assumption.  Bed axis
coordinate v means "gap v between nozzle and plate at that screw", so a
plate mounted e too high at a screw reads as gap v - e there, and the
nozzle touches at the z where that plane reaches zero.

No compiler and no printer needed:

    python test/test_z_tilt_command.py
"""
import sys, os, types, logging

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'klippy'))
sys.path.insert(0, os.path.join(HERE, '..', 'klippy', 'extras'))

for _name in ('mcu', 'chelper', 'pins'):
    sys.modules[_name] = types.ModuleType(_name)
sys.modules['chelper'].get_ffi = lambda: (None, None)

import gcode as gcode_mod                               # noqa: E402
import mathutil                                         # noqa: E402
from extras import gcode_move as gcode_move_mod         # noqa: E402
from extras import z_tilt as z_tilt_mod                 # noqa: E402

logging.disable(logging.CRITICAL)

FAIL = []


def ok(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAIL.append(msg)


# Geometry of the production machine, from printer.cfg
AXIS_NAMES = 'xyzab'
BED_AXES = [2, 3, 4]
Z_POSITIONS = [(300., 323.), (-23., 20.5), (623., 20.5)]
PROBE_POINTS = [(300., 280.), (100., 80.), (500., 80.)]
HORIZONTAL_MOVE_Z = 5.
TOOL_OFFSET = -0.8


def plane_through_screws(values):
    # The affine function of (x, y) taking the given value at each screw
    eqs = [[1., x, y] for x, y in Z_POSITIONS]
    c = mathutil.solve_linear_equations(eqs, [[v] for v in values])
    a, bx, by = c[0][0], c[1][0], c[2][0]
    return lambda x, y: a + bx * x + by * y


# ---------------------------------------------------------------- harness ---
class ConfigError(Exception):
    pass


class FakeMutex:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


class FakeReactor:
    def mutex(self):
        return FakeMutex()


class FakePrinter:
    config_error = ConfigError
    command_error = gcode_mod.CommandError

    def __init__(self):
        self.objects = {}
        self.handlers = {}

    def add_object(self, name, obj):
        self.objects[name] = obj

    def lookup_object(self, name, default=Ellipsis):
        if name in self.objects:
            return self.objects[name]
        if default is Ellipsis:
            raise ConfigError("Unknown config object '%s'" % (name,))
        return default

    def load_object(self, config, name, default=Ellipsis):
        return self.lookup_object(name, default)

    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)

    def send_event(self, event, *args):
        return [cb(*args) for cb in self.handlers.get(event, [])]

    def get_start_args(self):
        return {}

    def get_reactor(self):
        return FakeReactor()

    # Failure paths must be loud rather than swallowed
    def invoke_shutdown(self, msg):
        raise AssertionError("printer shutdown: %s" % (msg,))

    def request_exit(self, result):
        raise AssertionError("printer exit requested: %s" % (result,))


class FakeConfig:
    def __init__(self, printer, name, values):
        self.printer = printer
        self.name = name
        self.values = values

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.name

    def get(self, option, default=Ellipsis, **kwargs):
        if option in self.values:
            return self.values[option]
        if default is Ellipsis:
            raise ConfigError("Option '%s' is not valid" % (option,))
        return default

    def getfloat(self, option, default=Ellipsis, **kwargs):
        return self.get(option, default)

    def getint(self, option, default=Ellipsis, **kwargs):
        return self.get(option, default)

    def getlists(self, option, default=Ellipsis, **kwargs):
        return self.get(option, default)


class FakeStepper:
    def __init__(self, name, axis, toolhead):
        self.name = name
        self.axis = axis
        self.toolhead = toolhead

    def get_name(self):
        return self.name

    def is_active_axis(self, axis):
        return axis == self.axis

    def get_commanded_position(self):
        return self.toolhead.position[self.axis]

    def set_trapq(self, trapq):
        pass


class FakeKinematics:
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.steppers = [FakeStepper('stepper_' + n, i, toolhead)
                         for i, n in enumerate(AXIS_NAMES)]

    def get_steppers(self):
        return list(self.steppers)

    def get_bed_axes(self):
        return list(BED_AXES)

    def calc_position(self, stepper_positions):
        return [stepper_positions['stepper_' + n] for n in AXIS_NAMES]

    def set_position(self, newpos, homing_axes):
        pass


class SimToolhead:
    # A five axis toolhead carrying a rigid plate on three bed screws.
    # plate_error[i] is how much higher than nominal the plate sits at
    # bed screw i, so the gap the nozzle sees at that screw is the axis
    # coordinate minus that error.
    def __init__(self, printer, plate_error, start):
        self.printer = printer
        self.plate_error = list(plate_error)
        self.position = list(start)
        self.kin = FakeKinematics(self)
        self.move_log = []

    # --- axis roster ---
    def get_kinematics(self):
        return self.kin

    def get_axis_names(self):
        return AXIS_NAMES

    def get_axis_count(self):
        return len(AXIS_NAMES)

    def get_extruder_index(self):
        return len(AXIS_NAMES)

    def get_extra_axes(self):
        return [None] * (len(AXIS_NAMES) + 1)

    def get_trapq(self):
        return None

    # --- motion ---
    def get_position(self):
        return list(self.position)

    def move(self, newpos, speed):
        self.move_log.append(list(newpos))
        self.position = list(newpos)

    def manual_move(self, coord, speed):
        curpos = list(self.position)
        for i in range(len(coord)):
            if coord[i] is not None:
                curpos[i] = coord[i]
        self.move(curpos, speed)
        self.printer.send_event("toolhead:manual_move")

    def set_position(self, newpos, homing_axes=""):
        naxes = self.get_axis_count()
        if len(newpos) < naxes:
            newpos = list(newpos) + self.position[len(newpos):naxes]
        self.position[:naxes] = list(newpos[:naxes])
        self.kin.set_position(newpos, homing_axes)
        self.printer.send_event("toolhead:set_position")

    def flush_step_generation(self):
        pass

    def get_last_move_time(self):
        return 0.

    def dwell(self, delay):
        pass

    def wait_moves(self):
        pass

    # --- the simulated bed ---
    def gap_plane(self):
        values = [self.position[a] - e
                  for a, e in zip(BED_AXES, self.plate_error)]
        return plane_through_screws(values)

    def touch_z(self, x, y):
        # Descending every bed axis together by d reduces the gap
        # everywhere by d, so contact is this far below the current z
        return self.position[BED_AXES[0]] - self.gap_plane()(x, y)

    def out_of_level(self):
        gap = self.gap_plane()
        heights = [gap(x, y) for x, y in PROBE_POINTS]
        return max(heights) - min(heights)


class FakeManualProbeStatus:
    # Stands in for the [manual_probe] object.  ManualProbeHelper only
    # touches .status and .reset_status(), and the real class needs a
    # delta tower config to construct.
    def __init__(self):
        self.reset_status()

    def reset_status(self):
        self.status = {'is_active': False, 'z_position': None,
                       'z_position_lower': None, 'z_position_upper': None}

    def get_status(self, eventtime):
        return self.status


def build_printer(plate_error, start):
    printer = FakePrinter()
    gcode = gcode_mod.GCodeDispatch(printer)
    gcode.set_axis_names(AXIS_NAMES)
    printer.add_object('gcode', gcode)
    responses = []
    gcode.register_output_handler(responses.append)

    toolhead = SimToolhead(printer, plate_error, start)
    printer.add_object('toolhead', toolhead)
    printer.add_object('manual_probe', FakeManualProbeStatus())

    gm = gcode_move_mod.GCodeMove(FakeConfig(printer, 'gcode_move', {}))
    printer.add_object('gcode_move', gm)

    z_tilt_cfg = FakeConfig(printer, 'z_tilt', {
        'z_positions': list(Z_POSITIONS),
        'points': list(PROBE_POINTS),
        'horizontal_move_z': HORIZONTAL_MOVE_Z,
        'speed': 50.,
        'retries': 0,
        'retry_tolerance': 0.,
    })
    printer.add_object('z_tilt', z_tilt_mod.load_config(z_tilt_cfg))

    printer.send_event("klippy:connect")
    printer.send_event("klippy:ready")
    del CORRECTIONS[:]
    return printer, gcode, toolhead, gm, responses


def gcode_position(gm):
    return [round(lp - bp, 9)
            for lp, bp in zip(gm.last_position, gm.base_position)]


# Record the correction each levelling pass actually commands.  The
# toolhead's absolute position is not a useful measure of it: probing
# walks the bed up and down between points and Z_TILT_ADJUST does not
# return to where it started, so only the adjust move itself counts.
CORRECTIONS = []
_real_adjust_axes = z_tilt_mod.ZAdjustHelper._adjust_axes


def recording_adjust_axes(self, stepper_axes, adjustments, speed):
    toolhead = self.printer.lookup_object('toolhead')
    before = toolhead.get_position()
    res = _real_adjust_axes(self, stepper_axes, adjustments, speed)
    after = toolhead.get_position()
    CORRECTIONS.append([after[a] - before[a] for a in BED_AXES])
    return res


z_tilt_mod.ZAdjustHelper._adjust_axes = recording_adjust_axes


def run_z_tilt(printer, gcode, toolhead, max_points=12):
    # Send Z_TILT_ADJUST and play the operator at every manual probe
    manual = printer.lookup_object('manual_probe')
    probed = []
    gcode.run_script_from_command("Z_TILT_ADJUST")
    for _ in range(max_points):
        if not manual.status['is_active']:
            break
        x, y = toolhead.position[0], toolhead.position[1]
        target = toolhead.touch_z(x, y)
        probed.append((x, y, target))
        delta = target - toolhead.position[BED_AXES[0]]
        gcode.run_script_from_command("TESTZ Z=%.9f" % (delta,))
        gcode.run_script_from_command("ACCEPT")
    else:
        raise AssertionError("manual probing did not finish")
    return probed


# ------------------------------------------------- 1. the tilt is removed ---
print("Z_TILT_ADJUST on a plate whose B screw sits 0.8 mm high")
print("-" * 70)

PLATE = [0., 0., 0.8]
START = [150., 150., 10., 10., 10., 0.]
printer, gcode, toolhead, gm, responses = build_printer(PLATE, START)

before = toolhead.out_of_level()
probed = run_z_tilt(printer, gcode, toolhead)
after = toolhead.out_of_level()

print("    probed heights        %s" % (["%.6f" % p[2] for p in probed],))
print("    out of level before   %.6f mm" % (before,))
print("    out of level after    %.6f mm" % (after,))
ok(len(probed) == len(PROBE_POINTS),
   "every configured point was probed (%d)" % (len(probed),))
ok(before > 0.5, "the plate starts meaningfully out of level")
ok(after < 1e-6, "one pass of Z_TILT_ADJUST levels it")

first_correction = list(CORRECTIONS[-1])
print("    correction applied    %s"
      % (["%+.6f" % c for c in first_correction],))
ok(max([abs(c) for c in first_correction]) > 0.1,
   "the pass did apply a real correction")
ok(abs(sum(first_correction)) < 1e-9,
   "the correction sums to zero, so the bed is not raised or lowered bodily")

# The tilt is the difference between the screws, and the plate's true
# error is known, so the correction must match it up to that common shift
want = [e - sum(PLATE) / len(PLATE) for e in PLATE]
ok(max([abs(c - w) for c, w in zip(first_correction, want)]) < 1e-9,
   "and it matches the plate's actual error %s"
   % (["%+.4f" % w for w in want],))

# A second pass must find nothing left to correct
run_z_tilt(printer, gcode, toolhead)
second_correction = CORRECTIONS[-1]
print("    second pass applies   %s"
      % (["%+.9f" % c for c in second_correction],))
ok(max([abs(c) for c in second_correction]) < 1e-7,
   "a second pass corrects nothing, so it converged")
ok(toolhead.out_of_level() < 1e-6, "and the bed is still level")

# The offsets carry the correction, so the tilt lives in them rather
# than in the coordinate the operator commands
offsets = [gm.homing_position[a] for a in BED_AXES]
rel = [o - offsets[0] for o in offsets]
print("    offsets Z/A/B         %s" % (["%+.6f" % o for o in offsets],))
print("    relative to Z         %s" % (["%+.6f" % r for r in rel],))
plate_rel = [e - PLATE[0] for e in PLATE]
ok(max([abs(r - p) for r, p in zip(rel, plate_rel)]) < 1e-9,
   "A and B sit %s above Z in the offsets, the tilt that was removed"
   % (["%+.4f" % p for p in plate_rel[1:]],))
ok(abs(sum(offsets)) < 1e-9,
   "the offsets sum to zero, so no bodily shift was recorded either")
ok(len(set([round(p, 9) for p in
            [gcode_position(gm)[a] for a in BED_AXES]])) == 1,
   "the three bed axes report one common coordinate (%.3f)"
   % (gcode_position(gm)[BED_AXES[0]],))


# -------------------------------- 2. an offset set beforehand survives it ---
print()
print("A tool change offset set before levelling survives it")
print("-" * 70)

printer2, gcode2, toolhead2, gm2, responses2 = build_printer(PLATE, START)
gcode2.run_script_from_command(
    "SET_GCODE_OFFSET Z_ADJUST=%.3f A_ADJUST=%.3f B_ADJUST=%.3f MOVE=1"
    % (TOOL_OFFSET, TOOL_OFFSET, TOOL_OFFSET))

off_before = [gm2.homing_position[a] for a in BED_AXES]
run_z_tilt(printer2, gcode2, toolhead2)
off_after = [gm2.homing_position[a] for a in BED_AXES]
correction = CORRECTIONS[-1]
reported = [gcode_position(gm2)[a] for a in BED_AXES]

print("    offsets before        %s" % (["%+.6f" % o for o in off_before],))
print("    correction applied    %s" % (["%+.6f" % c for c in correction],))
print("    offsets after         %s" % (["%+.6f" % o for o in off_after],))
print("    reported Z/A/B after  %s" % (["%.6f" % r for r in reported],))

ok(off_before == [TOOL_OFFSET] * 3,
   "SET_GCODE_OFFSET applied the tool offset to all three bed axes")
ok(toolhead2.out_of_level() < 1e-6,
   "levelling still worked with an offset in place")
ok(abs(sum(off_after) / 3. - TOOL_OFFSET) < 1e-9,
   "the mean offset is still the tool offset, so it was not discarded")
ok(max([abs(a - (b + c))
        for a, b, c in zip(off_after, off_before, correction)]) < 1e-9,
   "each offset shifted by exactly the correction its own screw received")
ok(len(set([round(r, 9) for r in reported])) == 1,
   "the three bed axes still report one common coordinate (%.3f)"
   % (reported[0],))


# ---------------------------- 3. the wrong sign is caught by this harness ---
print()
print("The harness would catch the sign being wrong")
print("-" * 70)

original = z_tilt_mod.ZAdjustHelper._adjust_axes


def inverted_adjust_axes(self, stepper_axes, adjustments, speed):
    flipped = [-a for a in adjustments]
    return original(self, stepper_axes, flipped, speed)


z_tilt_mod.ZAdjustHelper._adjust_axes = inverted_adjust_axes
try:
    printer3, gcode3, toolhead3, gm3, _r3 = build_printer(PLATE, START)
    bad_before = toolhead3.out_of_level()
    for _ in range(3):
        run_z_tilt(printer3, gcode3, toolhead3)
    bad_after = toolhead3.out_of_level()
finally:
    z_tilt_mod.ZAdjustHelper._adjust_axes = original

print("    out of level before   %.6f mm" % (bad_before,))
print("    after three passes    %.6f mm" % (bad_after,))
ok(bad_after > bad_before * 3,
   "subtracting the adjustment diverges, so this test has teeth")


# ------------------ 4. and the offsets being assigned rather than added ----
print()
print("The harness would catch the offsets being assigned rather than added")
print("-" * 70)


def assigning_adjust_axes(self, stepper_axes, adjustments, speed):
    # What the code did before: write the correction into the session
    # offsets absolutely, which throws away whatever they already held
    toolhead = self.printer.lookup_object('toolhead')
    mean_adj = sum(adjustments) / len(adjustments)
    curpos = toolhead.get_position()
    final_pos = list(curpos)
    for axis, adj in zip(stepper_axes, adjustments):
        final_pos[axis] = curpos[axis] + (adj - mean_adj)
    toolhead.move(final_pos, speed)
    toolhead.set_position(final_pos)
    gm_obj = self.printer.lookup_object('gcode_move')
    for axis in set(stepper_axes):
        delta = final_pos[axis] - curpos[axis]
        gm_obj.base_position[axis] = delta
        gm_obj.homing_position[axis] = delta
    gm_obj.reset_last_position()


z_tilt_mod.ZAdjustHelper._adjust_axes = assigning_adjust_axes
try:
    printer4, gcode4, toolhead4, gm4, _r4 = build_printer(PLATE, START)
    gcode4.run_script_from_command(
        "SET_GCODE_OFFSET Z_ADJUST=%.3f A_ADJUST=%.3f B_ADJUST=%.3f MOVE=1"
        % (TOOL_OFFSET, TOOL_OFFSET, TOOL_OFFSET))
    run_z_tilt(printer4, gcode4, toolhead4)
    lost = [gm4.homing_position[a] for a in BED_AXES]
finally:
    z_tilt_mod.ZAdjustHelper._adjust_axes = recording_adjust_axes

print("    tool offset was       %+.3f mm on every bed axis" % (TOOL_OFFSET,))
print("    offsets afterwards    %s" % (["%+.6f" % o for o in lost],))
ok(abs(sum(lost) / 3. - TOOL_OFFSET) > 0.5,
   "assigning the correction discards the tool offset, so this has teeth too")

print()
if FAIL:
    print("%d FAILURE(S):" % (len(FAIL),))
    for f in FAIL:
        print("  ! " + f)
    sys.exit(1)
print("Z_TILT_ADJUST behaves correctly end to end.")
