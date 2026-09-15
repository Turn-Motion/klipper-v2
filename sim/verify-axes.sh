#!/bin/bash
# Verification gate for the multi-flavour kinematics.
#
# One build, three machines.  This script proves that the same host code
# runs every flavour, that the unit tests pass against the compiled C
# solvers, and that each profile boots and moves in batch mode.
#
#   1. make (host build prerequisites)
#   2. build the Linux-process MCU (with the simulated GPIO backend)
#   3. unit tests:
#        test_axis_motion.py  - axis roster, distance groups, move checking,
#                               the rotary modes, the derived carriages
#        test_uv_derived.py   - the derived carriage equations against the
#                               vendored reference solver, with the compiled
#                               C solver REQUIRED
#   4. klippy in batch mode against each profile:
#        cartesianxy3z    baseline, five axes
#        cartesianxy3z    + [bed_joint_carriages]: U and V derived from the
#                         bed pose.  Runs the SAME g-code as the baseline -
#                         adding the carriages must not change the g-code.
#        cartesianxy3zr   the same machine plus the rotary R axis
#
# Run with:
#   docker compose -f docker-compose.sim.yml run --rm \
#       --entrypoint /klipper/sim/verify-axes.sh klipper-sim
#
# Exits non-zero if any stage fails.
set -u
cd /klipper

RED=""; GRN=""; RST=""
if [ -t 1 ]; then RED=$'\033[31m'; GRN=$'\033[32m'; RST=$'\033[0m'; fi
fail() { echo "${RED}FAIL${RST}: $*"; exit 1; }

find /klipper/scripts -name "*.sh" -exec sed -i 's/\r//' {} + 2>/dev/null || true
sed -i 's/\r//' /klipper/Makefile 2>/dev/null || true

echo "============================================================"
echo " [1/6] clean build tree"
echo "============================================================"
# c_helper.so is NOT built by make - klippy compiles it lazily on first
# start.  Removing it forces a fresh compile below, which is what actually
# validates the C.  distclean drops any stale MCU config so that the Linux
# host MCU is the only target built here (no avr toolchain needed).
rm -f /klipper/klippy/chelper/c_helper.so
make distclean >/dev/null 2>&1
echo "--> clean OK"

echo
echo "============================================================"
echo " [2/6] building Linux host MCU (out/klipper.elf + .dict)"
echo "============================================================"
cp /klipper/sim/linux-mcu.config /klipper/.config
make olddefconfig CROSS_PREFIX= CC=gcc >/dev/null 2>&1
GPIO_ORIG=/tmp/gpio.c.orig
cp /klipper/src/linux/gpio.c "$GPIO_ORIG"
cp /klipper/src/linux/gpio_sim.c /klipper/src/linux/gpio.c
make CROSS_PREFIX= CC=gcc -j"$(nproc)" out/klipper.elf 2>&1 | tail -25
MCU_RC=${PIPESTATUS[0]}
cp "$GPIO_ORIG" /klipper/src/linux/gpio.c
[ "$MCU_RC" -eq 0 ] || fail "MCU build (rc=$MCU_RC)"
[ -f /klipper/out/klipper.dict ] || fail "out/klipper.dict not produced"
echo "--> Linux MCU OK"

echo
echo "============================================================"
echo " [3/6] unit tests"
echo "============================================================"
echo "--- test_axis_motion.py (roster, distance groups, move checking) ---"
python /klipper/test/test_axis_motion.py || fail "test_axis_motion.py"
echo
# REQUIRE_C_SOLVER makes an unavailable compiled solver a hard failure, so
# this stage cannot silently degrade to the Python transcription.
echo "--- test_uv_derived.py (vs the vendored reference solver) ---"
REQUIRE_C_SOLVER=1 python /klipper/test/test_uv_derived.py \
    || fail "test_uv_derived.py"

run_profile () {
    local label="$1" cfg="$2" gcode="$3" step="$4"
    echo
    echo "============================================================"
    echo " [$step/6] klippy batch: $label"
    echo "        config: $cfg"
    echo "        gcode : $gcode"
    echo "============================================================"
    local log="/tmp/klippy_${label}.log"
    rm -f "$log" "/tmp/ser_${label}.out"
    # [gcode_arcs] is not enabled on the production printer, so add it in a
    # temp config that includes the real one rather than diverging the
    # committed sim configs.  The include is an absolute path, so includes
    # *inside* the profile still resolve next to the profile itself.
    local tmpcfg="/tmp/cfg_${label}.cfg"
    printf '[include %s]\n\n[gcode_arcs]\nresolution: 1.0\n' "$cfg" \
        > "$tmpcfg"
    python klippy/klippy.py "$tmpcfg" \
        -i "$gcode" \
        -o "/tmp/ser_${label}.out" \
        -d /klipper/out/klipper.dict \
        -l "$log"
    local rc=$?
    echo "--- klippy exit code: $rc ---"

    # klippy.log embeds the whole config file between these markers.  Strip
    # it before scanning for errors, otherwise Jinja macro text in the
    # config trips the error patterns.
    local scan="/tmp/scan_${label}.log"
    sed '/^===== Config file =====/,/^=======================$/d' \
        "$log" > "$scan"

    if grep -qE "Traceback \(most recent call last\)" "$scan"; then
        grep -A 25 "Traceback (most recent call last)" "$scan" | head -40
        fail "$label: python traceback"
    fi
    if grep -qiE "Internal error|MCU .* shutdown|Lost communication" "$scan";
    then
        grep -iE "Internal error|shutdown|Lost communication" "$scan" | head -20
        fail "$label: klippy shutdown"
    fi
    if grep -qiE "Unable to load module|is not a valid config|Option .* is not valid|Unknown command|Must home first|NameError|AttributeError" "$scan"; then
        grep -inE "Unable to load module|is not a valid config|Option .* is not valid|Unknown command|Must home first|NameError|AttributeError" "$scan" | head -20
        fail "$label: config or command error"
    fi
    [ "$rc" -eq 0 ] || fail "$label: klippy exited $rc"

    echo "--- chelper compile ---"
    grep -E "Building C code module|Unable to build" "$log" | head -3
    echo "--- kinematics ---"
    grep -E "cartesianxy3z[r]?: axes=|bed_joint_carriages: " "$log" | head -3
    echo "--- homing ---"
    grep -iE "Coordinated bed homing|Homing [A-Z] axis" "$log" | head -8
    echo "--- final position ---"
    grep -E "^(mcu:|stepper:|kinematic:|toolhead:|gcode:)" "$log" | tail -6
    echo "${GRN}PASS${RST}: $label"
}

run_profile "baseline" /klipper/sim/printer.sim.cfg \
            /klipper/gcode/bed_pose_smoke.gcode 4
run_profile "uvderived" /klipper/sim/printer.sim.uvderived.cfg \
            /klipper/gcode/bed_pose_smoke.gcode 5
run_profile "rotary" /klipper/sim/printer.sim.rotary.cfg \
            /klipper/gcode/rotary_smoke.gcode 6

echo
echo "--- derived carriage positions (uvderived only) ---"
grep -E "^stepper: " /tmp/klippy_uvderived.log | tail -1 | tr ' ' '\n' \
    | grep -E "stepper_[uv]:" || echo "(no stepper_u/stepper_v in the report)"
echo
echo "--- rotary axis position (rotary only) ---"
grep -E "^kinematic: " /tmp/klippy_rotary.log | tail -1 \
    || echo "(no kinematic position report)"

echo
echo "============================================================"
echo " ${GRN}ALL CHECKS PASSED${RST}"
echo "   cartesianxy3z    - baseline, five axes"
echo "   + bed joints     - U and V derived from the bed pose"
echo "   cartesianxy3zr   - the same machine plus the rotary R axis"
echo "============================================================"
