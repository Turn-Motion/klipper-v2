#!/bin/bash
# Build the MCU dictionaries and run the upstream regression suite in a
# Debian/python container.  See FORK_CHANGES.md section 7.
#
#   run-regression.sh <source tree (read-only ok)> <output dir>
set -u
SRC=${1:-/klipper}
OUT=${2:-/out}
WORK=/work/klipper
mkdir -p "$OUT"
log() { echo "### $(date +%T) $*" | tee -a "$OUT/regression-summary.log"; }

log "installing toolchain"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq >/dev/null 2>&1
apt-get install -y -qq gcc make libffi-dev pkg-config gcc-avr avr-libc binutils-avr >/dev/null 2>&1 \
    || { log "apt install FAILED"; exit 1; }
pip install -q cffi pyserial greenlet jinja2 numpy scipy >/dev/null 2>&1 \
    || { log "pip install FAILED"; exit 1; }

log "copying source tree (LF line endings)"
mkdir -p "$WORK"
(cd "$SRC" && tar --exclude=.git --exclude=out --exclude='*.so' --exclude='*.pyc' -cf - .) \
    | (cd "$WORK" && tar -xf -)
find "$WORK" -type f \( -name Makefile -o -name '*.sh' -o -name '*.py' -o -name '*.c' \
    -o -name '*.h' -o -name Kconfig -o -name '*.cfg' -o -name '*.test' -o -name '*.config' \
    -o -name '*.lds' -o -name '*.mk' -o -name '*.gcode' \) -exec sed -i 's/\r$//' {} +
cd "$WORK"

build_dict() {   # config-file dict-name
    make distclean >/dev/null 2>&1
    cp "$1" .config
    make olddefconfig >/dev/null 2>&1
    if make -j4 > "$OUT/build-$2.log" 2>&1; then
        mkdir -p dict && cp out/klipper.dict "dict/$2.dict"
        log "built $2.dict"
    else
        log "BUILD FAILED for $2 (see build-$2.log)"
    fi
}
for c in atmega2560 atmega644p atmega1284p atmega1280 at90usb1286; do
    build_dict "test/configs/$c.config" "$c"
done
build_dict sim/linux-mcu.config linuxprocess

log "building chelper"
python -c "import sys; sys.path.insert(0, 'klippy'); import chelper; chelper.get_ffi()" \
    || { log "chelper build FAILED"; exit 1; }

log "selecting tests with available dictionaries"
TESTS=""
for t in test/klippy/*.test; do
    ok=1
    for d in $(grep '^DICTIONARY' "$t" | sed 's/DICTIONARY//; s/[a-z]*=//g'); do
        [ -f "dict/$d" ] || ok=0
    done
    [ $ok = 1 ] && TESTS="$TESTS $t"
done
log "$(echo $TESTS | wc -w) tests selected"
python scripts/test_klippy.py -d dict $TESTS > "$OUT/regression.log" 2>&1
rc=$?
log "regression exit code: $rc"
grep -E 'FAILED|test cases passed' "$OUT/regression.log" | tee -a "$OUT/regression-summary.log"
exit $rc
