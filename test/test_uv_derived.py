"""Verify the derived U/V carriage positions against the reference solver.

The horizontal bed-joint carriages are not axes - their position is computed
from the commanded (Z, A, B) by plate rigidity.  The equations live in
klippy/chelper/kin_bedjoint.c; this test checks them against the
Turn-Motion reference implementation vendored in test/reference/printerkin.

Runs standalone with plain Python - no compiler and no running printer:

    python test/test_uv_derived.py

If a compiled c_helper.so is importable the real C solver is exercised as
well; otherwise the Python transcription of the same equations is checked
(and the C source is verified to match it textually).
"""
import sys, os, math, itertools

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'reference'))
sys.path.insert(0, os.path.join(HERE, '..', 'klippy'))

from printerkin import Kinematics                                # noqa: E402
from printerkin.solver import InfeasibleConfig, SingularConfig   # noqa: E402

FAIL = []


def ok(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAIL.append(msg)


REF = Kinematics()
A_HOME_X = REF.machine.A.base[0]      # -23.0
B_HOME_X = REF.machine.B.base[0]      # +623.0
HOME_Z = REF.machine.home[2]          # 468.6
D_X = B_HOME_X - A_HOME_X             # 646.0
MID_X = 0.5 * (A_HOME_X + B_HOME_X)   # 300.0
MIN_CSX2_FRAC = 0.01                  # must match kin_bedjoint.c


def derived_uv(z, a, b):
    """Python transcription of bedjoint_stepper_calc_position() in
    klippy/chelper/kin_bedjoint.c - returns SLIDE coordinates."""
    z_mz = z - 0.5 * (a + b)
    z_ab = b - a
    d_x2 = D_X * D_X
    rad = d_x2 - z_ab * z_ab
    floor_rad = MIN_CSX2_FRAC * d_x2
    if rad < floor_rad:
        rad = floor_rad
    csx = math.sqrt(rad)
    common = MID_X + z_mz * z_ab / csx
    return (common - 0.5 * csx - A_HOME_X,      # U slide
            common + 0.5 * csx - B_HOME_X)      # V slide


# ── Exercise the real compiled C solver ──────────────────────────────────────
# chelper.get_ffi() builds c_helper.so on demand, so this normally works
# anywhere a compiler is present.  Set REQUIRE_C_SOLVER=1 (verify-axes.sh
# does) to make an unavailable C solver a hard failure instead of silently
# degrading to the Python transcription.
REQUIRE_C = os.environ.get('REQUIRE_C_SOLVER', '') not in ('', '0')
c_solver = None
c_solver_error = None
try:
    import chelper
    ffi_main, ffi_lib = chelper.get_ffi()

    # The bed axes of the cartesianxy3z / cartesianxy3zr roster
    Z_AXIS, A_AXIS, B_AXIS = 2, 3, 4

    def _c_solver(z, a, b):
        out = []
        for side in (-1.0, 1.0):
            sk = ffi_lib.bedjoint_stepper_alloc(
                A_HOME_X, B_HOME_X, side, Z_AXIS, A_AXIS, B_AXIS)
            out.append(ffi_lib.itersolve_calc_position_from_coord(
                sk, 5, [0., 0., z, a, b]))
            ffi_lib.free(sk)
        return tuple(out)
    _c_solver(HOME_Z, HOME_Z, HOME_Z)   # smoke test
    c_solver = _c_solver
except Exception as e:                                    # pragma: no cover
    c_solver_error = str(e)

print("Compiled C solver")
print("-" * 70)
if c_solver is not None:
    print("  compiled c_helper.so loaded - the REAL C solver is under test")
else:
    print("  UNAVAILABLE: %s" % (c_solver_error or "unknown")[:100])
    print("  falling back to the Python transcription of the same equations")
if REQUIRE_C:
    ok(c_solver is not None,
       "REQUIRE_C_SOLVER is set - the compiled C solver must be exercised")
else:
    print("  (set REQUIRE_C_SOLVER=1 to make this a hard requirement)")
print()


print("Geometry taken from the reference machine")
print("-" * 70)
print("  A home ball joint X : %.4f" % A_HOME_X)
print("  B home ball joint X : %.4f" % B_HOME_X)
print("  d_x (A<->B spacing) : %.4f" % D_X)
print("  midpoint            : %.4f" % MID_X)
print("  home Z=A=B          : %.4f" % HOME_Z)
print()

# ── 1. Derived vs reference over a pose sweep ────────────────────────────────
print("Derived U/V vs reference solve_passive() over a pose sweep")
print("-" * 70)
worst = 0.0
worst_pose = None
rows = skipped = 0
for dz, dab in itertools.product(range(-180, 181, 15), range(-240, 241, 20)):
    z = HOME_Z + dz
    a = HOME_Z - dab / 2.0
    b = HOME_Z + dab / 2.0
    try:
        ps = REF.solve_passive(z=z, a=a, b=b)
    except (InfeasibleConfig, SingularConfig):
        skipped += 1
        continue
    u, v = derived_uv(z, a, b)
    err = max(abs(u - ps.A_slide), abs(v - ps.B_slide))
    if err > worst:
        worst, worst_pose = err, (z, a, b)
    rows += 1
    if c_solver is not None:
        cu, cv = c_solver(z, a, b)
        cerr = max(abs(cu - ps.A_slide), abs(cv - ps.B_slide))
        if cerr > worst:
            worst, worst_pose = cerr, (z, a, b)
print("  poses compared: %d   skipped (infeasible/singular): %d"
      % (rows, skipped))
print("  worst error: %.3e mm at %s" % (worst, worst_pose))
print("  implementation under test: %s"
      % ("compiled C + Python transcription" if c_solver is not None
         else "Python transcription only"))
ok(rows > 400, "swept a meaningful number of poses (%d)" % rows)
ok(worst < 1e-9, "derived U/V match the reference to < 1e-9 mm")
if c_solver is not None:
    ok(True, "compiled C solver exercised and included in the comparison")

# ── 2. Home pose ─────────────────────────────────────────────────────────────
print()
print("Home pose")
print("-" * 70)
u, v = derived_uv(HOME_Z, HOME_Z, HOME_Z)
ok(abs(u) < 1e-12 and abs(v) < 1e-12,
   "level bed at home -> both slides are 0 (U=%.3e V=%.3e)" % (u, v))

# ── 3. Pitch alone must not move the carriages ───────────────────────────────
print()
print("Pitch-only poses (A == B) must produce zero carriage travel")
print("-" * 70)
for dz in (-180, -90, -30, 30, 90, 180):
    z = HOME_Z + dz
    u, v = derived_uv(z, HOME_Z, HOME_Z)
    pitch = math.degrees(math.asin(max(-1., min(1., dz / 304.6))))
    ok(abs(u) < 1e-12 and abs(v) < 1e-12,
       "pitch %+7.3f deg -> U=%.3e V=%.3e" % (pitch, u, v))

# ── 4. Rigidity invariant ────────────────────────────────────────────────────
print()
print("Rigidity invariant  (V_world - U_world)^2 + (B-A)^2 == d_x^2")
print("-" * 70)
bad = 0
for dz, dab in itertools.product((-150, -50, 0, 50, 150),
                                 (-200, -60, 0, 60, 200)):
    z, a, b = HOME_Z + dz, HOME_Z - dab / 2.0, HOME_Z + dab / 2.0
    u, v = derived_uv(z, a, b)
    u_world, v_world = u + A_HOME_X, v + B_HOME_X
    lhs = (v_world - u_world) ** 2 + (b - a) ** 2
    if abs(lhs - D_X * D_X) > 1e-6:
        bad += 1
ok(bad == 0, "invariant holds at every sampled pose (%d violations)" % bad)

# ── 5. Degenerate pose must stay finite ──────────────────────────────────────
print()
print("Degenerate / unreachable poses stay finite (no NaN into the step gen)")
print("-" * 70)
for dab in (D_X - 1.0, D_X, D_X + 500.0):
    u, v = derived_uv(HOME_Z, HOME_Z - dab / 2.0, HOME_Z + dab / 2.0)
    ok(math.isfinite(u) and math.isfinite(v),
       "|B-A| = %.1f -> U=%.3f V=%.3f (finite)" % (dab, u, v))

# ── 6. The C source really implements these equations ────────────────────────
print()
print("C source cross-check")
print("-" * 70)
csrc = open(os.path.join(HERE, '..', 'klippy', 'chelper',
                         'kin_bedjoint.c')).read()
for frag, desc in (
        ("double z_mz = cz - 0.5 * (ca + cb);", "z_mz = Z - (A+B)/2"),
        ("double z_ab = cb - ca;", "z_ab = B - A"),
        ("bs->d_x2 - z_ab * z_ab", "rad = d_x^2 - z_ab^2"),
        ("bs->mid_x + z_mz * z_ab / csx + bs->side * 0.5 * csx",
         "world_x = mid + z_mz*z_ab/csx + side*csx/2"),
        ("world_x - bs->home_x", "slide = world_x - home_x"),
        ("AF_AXIS(z_axis) | AF_AXIS(a_axis)", "active on the bed axes"),
        ("MIN_CSX2_FRAC 0.01", "degenerate-pose floor matches this test")):
    ok(frag in csrc, "kin_bedjoint.c contains: %s" % desc)

print()
if FAIL:
    print("%d FAILURE(S):" % len(FAIL))
    for f in FAIL:
        print("  ! " + f)
    sys.exit(1)
print("All derived-U/V checks passed.")
