#!/bin/sh
# Sets up the Satellite1 ESPHome build environment:
#   - fetches git submodules, if any
#   - creates/repairs a Python venv on a version ESPHome supports (3.12-3.14)
#   - installs the pinned ESPHome from requirements.txt
# POSIX sh only (no bashisms) so it runs under bash, zsh, dash, ash, etc.
set -e

# Always operate from the repo root, wherever the script is invoked from
cd "$(dirname "$0")/.."

venv=".venv"

# --- Submodules -------------------------------------------------------------
if git rev-parse --git-dir >/dev/null 2>&1; then
    echo "Updating git submodules"
    git submodule update --init --recursive || { echo "Submodule update failed"; exit 1; }
else
    echo "Not a git checkout; skipping submodule update"
fi

# --- Python -----------------------------------------------------------------
# Pick the first Python that ESPHome 2026.x supports (3.12-3.14)
PYTHON=""
for p in python3.14 python3.13 python3.12; do
    if command -v "$p" >/dev/null 2>&1; then PYTHON="$p"; break; fi
done
if [ -z "$PYTHON" ]; then
    echo "Need Python 3.12-3.14. On macOS: brew install python@3.12"
    exit 1
fi

# Rebuild the venv if it exists but was made with an unsupported Python
if [ -d "${venv}" ] && ! "${venv}/bin/python" -c 'import sys; sys.exit(not (3,12) <= sys.version_info[:2] < (3,15))' 2>/dev/null; then
    echo "Existing ${venv} uses an unsupported Python, recreating"
    rm -rf "${venv}"
fi

if [ ! -d "${venv}" ]; then
    echo "Creating virtual environment at ${venv} with ${PYTHON}"
    "${PYTHON}" -m venv "${venv}"
fi
. "${venv}/bin/activate"

# --- Dependencies -----------------------------------------------------------
echo 'Installing Python dependencies'
pip3 install --upgrade pip
pip3 install -r requirements.txt

echo
echo "Done. Activate the environment in your shell with:"
echo "  . ${venv}/bin/activate"
