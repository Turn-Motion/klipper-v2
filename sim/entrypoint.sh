#!/bin/bash
# Klipper simulation container entrypoint
# Runs on every (re)start, so branch switches just need a container restart.
set -e

BRANCH=$(git -C /klipper rev-parse --abbrev-ref HEAD 2>/dev/null || echo "detached")
echo "============================================"
echo "  Klipper Simulation Container"
echo "  Branch : ${BRANCH}"
echo "============================================"

cd /klipper

# ── Step 1: Build chelper ────────────────────────────────────────────────────
# chelper is the C motion-planning core (trapq, itersolve).
# Pure math — no MCU, no GPIO. Must be rebuilt per branch.
echo "--> [1/2] Building chelper..."
make CROSS_PREFIX= CC=gcc -j$(nproc) 2>&1 | tail -5
echo "--> chelper OK"

# ── Step 2: Build Linux host MCU ────────────────────────────────────────────
# We only need out/klipper.dict — klippy uses it to validate pin names at
# config-load time. In --debugoutput mode no serial connection is made.
echo "--> [2/2] Building Linux host MCU (needed for pin dictionary)..."
cp /klipper/sim/linux-mcu.config /klipper/.config
make olddefconfig CROSS_PREFIX= CC=gcc >/dev/null 2>&1

# Build only the dict-generating object, not the full elf (avoids check-gcc.sh)
make CROSS_PREFIX= CC=gcc -j$(nproc) out/klipper.dict 2>&1
if [ ! -f /klipper/out/klipper.dict ]; then
    echo "ERROR: out/klipper.dict was not produced. MCU build failed."
    exit 1
fi
echo "--> Linux MCU OK  (out/klipper.dict)"

# ── Start klippy ─────────────────────────────────────────────────────────────
CFG="${KLIPPER_CFG:-/config/printer.sim.cfg}"
echo ""
echo "--> Starting klippy"
echo "    config  : ${CFG}"
echo "    output  : /tmp/klippy.log"
echo "    socket  : /tmp/klippy.sock  (connect via API)"
echo ""

# --debugoutput requires an explicit --dictionary when the mcu serial is a path
# rather than a real serial device.
exec python klippy/klippy.py \
    "${CFG}" \
    --debugoutput=/tmp/klipper-mcu.out \
    --dictionary=/klipper/out/klipper.dict \
    -a /tmp/klippy.sock \
    -l /tmp/klippy.log &

# Wait for the socket to appear, then bridge it to TCP port 7126
# so Windows host visualizer can connect without docker exec
echo "--> Waiting for klippy socket..."
for i in $(seq 1 30); do
    [ -S /tmp/klippy.sock ] && break
    sleep 0.5
done
echo "--> Bridging /tmp/klippy.sock → TCP 0.0.0.0:7126"
socat TCP-LISTEN:7126,reuseaddr,fork UNIX-CONNECT:/tmp/klippy.sock
