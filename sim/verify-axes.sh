#!/bin/bash
# Regression gate for the derived horizontal-carriage kinematics.
#
#   1. make (host build prerequisites)
#   2. build the Linux-process MCU (with the simulated GPIO backend)
#   3. unit tests: test_uv_derived.py (vs the vendored reference solver,
#      with the compiled C solver REQUIRED) and test_uv_motion.py
#      (three-group limiter + derived-carriage range rejection)
#   4. run klippy in batch mode against BOTH kinematics profiles, using the
#      SAME G-code file (both take only X Y Z A B):
#        - cartesianxy3z     (baseline, no horizontal carriages)
#        - cartesianxy3z2h   (U/V derived from Z/A/B)
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
echo " [1/5] make (host build prerequisites)"
echo "============================================================"
# NOTE: c_helper.so is NOT built by make - klippy compiles it lazily on first
# start.  Removing it forces a fresh compile in stage 4, which is what
# actually validates the C.
rm -f /klipper/klippy/chelper/c_helper.so
make CROSS_PREFIX= CC=gcc -j"$(nproc)" 2>&1 | tail -20
[ "${PIPESTATUS[0]}" -eq 0 ] || fail "make"
echo "--> make OK"

echo
echo "============================================================"
echo " [2/5] building Linux host MCU (out/klipper.elf + .dict)"
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

run_profile () {
    local label="$1" cfg="$2" gcode="$3" step="$4"
    echo
    echo "============================================================"
    echo " [$step/5] klippy batch: $label"
    echo "        config: $cfg"
    echo "        gcode : $gcode"
    echo "============================================================"
    local log="/tmp/klippy_${label}.log"
    rm -f "$log" "/tmp/ser_${label}.out"
    # [gcode_arcs] is not enabled on the production printer, so append it to a
    # temp copy rather than diverging the committed sim configs.
    local tmpcfg="/tmp/cfg_${label}.cfg"
    { cat "$cfg"; printf '\n[gcode_arcs]\nresolution: 1.0\n'; } > "$tmpcfg"
    python klippy/klippy.py "$tmpcfg" \
        -i "$gcode" \
        -o "/tmp/ser_${label}.out" \
        -d /klipper/out/klipper.dict \
        -l "$log"
    local rc=$?
    echo "--- klippy exit code: $rc ---"

    # klippy.log embeds the whole config file between these markers.  Strip it
    # before scanning for errors, otherwise Jinja macro text in the config
    # (e.g. "{% if params.S is not defined %}") trips the error patterns.
    local scan="/tmp/scan_${label}.log"
    sed '/^===== Config file =====/,/^=======================$/d' \
        "$log" > "$scan"

    if grep -qE "Traceback \(most recent call last\)" "$scan"; then
        grep -A 25 "Traceback (most recent call last)" "$scan" | head -40
        fail "$label: python traceback"
    fi
    if grep -qiE "Internal error|MCU .* shutdown|Lost communication" "$scan"; then
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
    echo "--- kinematics limits ---"
    grep -E "CartesianXY3Z(2H)?: " "$log" | head -3
    # [z_tilt] requires exactly len(z_positions)==3 bed steppers or klippy
    # refuses to start (Config error), so a clean start doubles as the
    # runtime assertion that the derived carriages are excluded.
    echo "--- z_tilt ---"
    grep -qE "Config error|z_positions needs exactly" "$scan" \
        && fail "$label: z_tilt stepper scan rejected the config"
    echo "z_tilt accepted the bed stepper set"
    echo "--- homing ---"
    grep -iE "Coordinated Z/A/B|Homing [ZAB] axis" "$log" | head -6
    echo "--- final position ---"
    grep -E "^(X:|mcu:|stepper:|kinematic:|toolhead:|gcode:)" "$log" | tail -8
    echo "${GRN}PASS${RST}: $label"
}

echo
echo "============================================================"
echo " [3/5] unit tests"
echo "============================================================"
# REQUIRE_C_SOLVER makes an unavailable compiled solver a hard failure, so
# this stage cannot silently degrade to the Python transcription.  Output is
# NOT truncated - the C-solver banner prints at the top.
echo "--- test_uv_derived.py (vs vendored reference solver) ---"
REQUIRE_C_SOLVER=1 python /klipper/test/test_uv_derived.py
[ $? -eq 0 ] || fail "test_uv_derived.py"
echo
echo "--- test_uv_motion.py (limiter + range rejection) ---"
python /klipper/test/test_uv_motion.py
[ $? -eq 0 ] || fail "test_uv_motion.py"

run_profile "baseline5" /klipper/sim/printer.sim.cfg \
            /klipper/gcode/bed_pose_smoke.gcode 4
run_profile "uvderived" /klipper/sim/printer.sim.uvderived.cfg \
            /klipper/gcode/bed_pose_smoke.gcode 5

echo
echo "--- derived carriage positions (uvderived only) ---"
grep -E "^stepper: " /tmp/klippy_uvderived.log | tail -1 | tr ' ' '\n' \
    | grep -E "stepper_[uv]:" || echo "(no stepper_u/stepper_v in the report)"

echo
echo "============================================================"
echo " ${GRN}ALL CHECKS PASSED${RST}"
echo "   cartesianxy3z    - baseline unaffected"
echo "   cartesianxy3z2h  - U/V derived, matches the reference solver"
echo "============================================================"
