#!/bin/bash
# Render docs/Turn_Kinematics_Architecture.html to PDF.
#
# The document is plain HTML with inline SVG, so any Chromium-based browser
# renders it identically.  Pass a browser binary as $1 to override the
# search below.
#
#   ./docs/build-architecture-pdf.sh
#   ./docs/build-architecture-pdf.sh /usr/bin/chromium
set -eu

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$HERE/Turn_Kinematics_Architecture.html"
OUT="$HERE/Turn_Kinematics_Architecture.pdf"

BROWSER=${1:-}
if [ -z "$BROWSER" ]; then
    for candidate in \
        google-chrome chromium chromium-browser microsoft-edge \
        "/c/Program Files/Google/Chrome/Application/chrome.exe" \
        "/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe"
    do
        if command -v "$candidate" >/dev/null 2>&1; then
            BROWSER=$(command -v "$candidate"); break
        elif [ -x "$candidate" ]; then
            BROWSER=$candidate; break
        fi
    done
fi
[ -n "$BROWSER" ] || {
    echo "No Chromium-based browser found; pass one as the first argument." >&2
    exit 1
}

echo "rendering with: $BROWSER"
"$BROWSER" --headless=new --disable-gpu --no-sandbox --no-pdf-header-footer \
    --virtual-time-budget=10000 \
    --print-to-pdf="$OUT" "file://$SRC"

[ -f "$OUT" ] || { echo "PDF was not produced" >&2; exit 1; }
echo "wrote $OUT"
