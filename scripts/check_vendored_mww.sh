#!/bin/bash
# Verifies the vendored micro_wake_word component (see its FPH_VENDOR.md).
#
# The vendor's whole contract is "upstream, byte-identical, plus the FPH-marked high-water
# register" - this script is that contract made executable, so an ESPHome bump can't silently
# ship a drifted or stale copy. It fails unless:
#
#   1. every line the vendored copy ADDS over the pinned upstream package carries an "FPH"
#      marker (or is blank - the register block ends on one),
#   2. the vendored copy DELETES nothing from upstream,
#   3. the two detection log lines mww_runtime_loader::on_mww_log_ parses still exist verbatim
#      in the vendored micro_wake_word.cpp (that dependency predates the vendor; a bump that
#      rewords them makes the tuner silently record nothing).
#
# Usage: scripts/check_vendored_mww.sh [site-packages-component-dir]
# With no argument it looks in .venv (scripts/setup_build_env.sh's layout).

set -euo pipefail
cd "$(dirname "$0")/.."

VENDORED="esphome/components/micro_wake_word"

UPSTREAM="${1:-}"
if [ -z "$UPSTREAM" ]; then
    UPSTREAM=$(ls -d .venv/lib/python*/site-packages/esphome/components/micro_wake_word 2>/dev/null | head -1 || true)
fi
if [ -z "$UPSTREAM" ] || [ ! -d "$UPSTREAM" ]; then
    echo "error: pinned upstream micro_wake_word not found (run scripts/setup_build_env.sh first," >&2
    echo "       or pass the site-packages component directory as the first argument)" >&2
    exit 2
fi

echo "Vendored: $VENDORED"
echo "Upstream: $UPSTREAM ($(grep -i '^esphome==' requirements.txt))"

fail=0

# ---- 1 & 2: the diff may only ADD FPH-marked (or blank) lines ------------------------------
# diff -r catches extra/missing files too (FPH_VENDOR.md is the one allowed extra).
raw=$(diff -r -u \
    --exclude=__pycache__ --exclude=FPH_VENDOR.md \
    "$UPSTREAM" "$VENDORED" || true)

# Added lines ('+' but not the '+++' file header) must say FPH or be blank.
bad_added=$(printf '%s\n' "$raw" | grep -E '^\+' | grep -vE '^\+\+\+' | grep -vE '^\+\s*$' | grep -v 'FPH' || true)
# Removed lines ('-' but not the '---' header) are never allowed: the vendor only adds.
removed=$(printf '%s\n' "$raw" | grep -E '^-' | grep -vE '^---' || true)
# Files present on one side only ("Only in ...") are never allowed either.
lopsided=$(printf '%s\n' "$raw" | grep '^Only in ' || true)

if [ -n "$bad_added" ]; then
    echo "FAIL: vendored lines without an FPH marker:" >&2
    printf '%s\n' "$bad_added" >&2
    fail=1
fi
if [ -n "$removed" ]; then
    echo "FAIL: the vendored copy deletes upstream lines (the vendor only ever adds):" >&2
    printf '%s\n' "$removed" >&2
    fail=1
fi
if [ -n "$lopsided" ]; then
    echo "FAIL: file set differs from upstream:" >&2
    printf '%s\n' "$lopsided" >&2
    fail=1
fi

# ---- 3: the log lines the tuner parses are pinned ------------------------------------------
# Kept in lockstep with mww_runtime_loader::on_mww_log_'s two sscanf format strings.
for needle in \
    "Detected '%s' with sliding average probability is %.2f and max probability is %.2f" \
    "Wake word model predicts '%s', but VAD model doesn't." \
; do
    if ! grep -qF "$needle" "$VENDORED/micro_wake_word.cpp"; then
        echo "FAIL: pinned detection log line missing from micro_wake_word.cpp:" >&2
        echo "      $needle" >&2
        echo "      (on_mww_log_ in mww_runtime_loader parses this; re-check both.)" >&2
        fail=1
    fi
done

if [ "$fail" -ne 0 ]; then
    exit 1
fi
echo "OK: vendored copy is upstream + FPH-marked register only, and the parsed log lines hold."
