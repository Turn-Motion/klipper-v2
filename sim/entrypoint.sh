#!/bin/bash
# Klipper simulation container entrypoint
# Runs on every restart, so branch switches only need a container restart.
set -e

BRANCH=$(git -C /klipper rev-parse --abbrev-ref HEAD 2>/dev/null || echo "detached")
echo "============================================"
echo "  Klipper Simulation Container"
echo "  Branch : ${BRANCH}"
echo "============================================"

cd /klipper

# Strip Windows CRLF line endings from build scripts and Makefile.
# Safe to do in-place: .gitattributes marks these files as eol=lf, so git
# treats LF-only content as unmodified on all platforms.
find /klipper/scripts -name "*.sh" -exec sed -i 's/\r//' {} + 2>/dev/null || true
sed -i 's/\r//' /klipper/Makefile 2>/dev/null || true

echo "--> [1/2] Building chelper..."
make CROSS_PREFIX= CC=gcc -j$(nproc) 2>&1 | tail -5
echo "--> chelper OK"

echo "--> [2/2] Building Linux host MCU..."
cp /klipper/sim/linux-mcu.config /klipper/.config
make olddefconfig CROSS_PREFIX= CC=gcc >/dev/null 2>&1

# Use the simulated GPIO implementation so no /dev/gpiochip0 device is needed.
GPIO_ORIG=/tmp/gpio.c.orig.$$
cp /klipper/src/linux/gpio.c "$GPIO_ORIG"
cp /klipper/src/linux/gpio_sim.c /klipper/src/linux/gpio.c
make CROSS_PREFIX= CC=gcc -j$(nproc) out/klipper.elf 2>&1
cp "$GPIO_ORIG" /klipper/src/linux/gpio.c
rm -f "$GPIO_ORIG"
if [ ! -f /klipper/out/klipper.elf ]; then
    echo "ERROR: out/klipper.elf was not produced. MCU build failed."
    exit 1
fi
echo "--> Linux MCU OK  (out/klipper.elf)"

CFG="${KLIPPER_CFG:-/config/printer.sim.cfg}"
echo ""
echo "--> Starting klippy"
echo "    config  : ${CFG}"
echo "    output  : /tmp/klippy.log"
echo "    socket  : /tmp/klippy.sock  (connect via API)"
echo ""

# Make shared Unix sockets permissive for Moonraker in the separate container.
# This is simulation-only and avoids Linux UID/GID coordination.
umask 000

# Deterministic simulated temperatures for temperature_host sensors
# Values are in millidegrees C.
echo 210000 > /tmp/sim_temp_hotend
echo 60000 > /tmp/sim_temp_bed
chmod 666 /tmp/sim_temp_hotend /tmp/sim_temp_bed || true

echo "--> Starting Linux host MCU on /tmp/klipper_host_mcu"
/klipper/out/klipper.elf -I /tmp/klipper_host_mcu &

echo "--> Waiting for Linux host MCU socket..."
for i in $(seq 1 30); do
    [ -e /tmp/klipper_host_mcu ] && break
    sleep 0.5
done
if [ ! -e /tmp/klipper_host_mcu ]; then
    echo "ERROR: /tmp/klipper_host_mcu not created"
    exit 1
fi
chmod 666 /tmp/klipper_host_mcu || true

python klippy/klippy.py \
    "${CFG}" \
    -a /tmp/klippy.sock \
    -l /tmp/klippy.log &

echo "--> Waiting for klippy socket..."
for i in $(seq 1 30); do
    [ -S /tmp/klippy.sock ] && break
    sleep 0.5
done

if [ -S /tmp/klippy.sock ]; then
    chmod 666 /tmp/klippy.sock || true
fi

echo "--> Bridging /tmp/klippy.sock to TCP 0.0.0.0:7126"
exec socat TCP-LISTEN:7126,reuseaddr,fork UNIX-CONNECT:/tmp/klippy.sock
