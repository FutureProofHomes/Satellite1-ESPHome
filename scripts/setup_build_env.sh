#!/bin/sh
# Sets up the Satellite1 ESPHome build environment:
#   - fetches git submodules, if any
#   - creates/repairs a Python venv on a version ESPHome supports (3.12-3.14)
#   - installs the pinned ESPHome from requirements.txt
#   - leaves the venv active in your shell
#
# Source it so the venv stays active:
#   source scripts/setup_build_env.sh
#
# POSIX sh only (no bashisms) so it works from bash, zsh, dash, ash, etc.
# Because it is sourced, nothing here may change the caller's shell options
# or directory: the work runs in a subshell and only the activate is sourced.

# Find the repo root. $0 is the caller's shell when sourced, so try git
# and the current directory before falling back to the script's location.
_sbe_root=""
for _sbe_dir in "$(git rev-parse --show-toplevel 2>/dev/null)" "$(pwd)" "$(dirname "$0")/.."; do
    if [ -n "$_sbe_dir" ] && [ -f "$_sbe_dir/scripts/setup_build_env.sh" ]; then
        _sbe_root=$(cd "$_sbe_dir" && pwd)
        break
    fi
done
unset _sbe_dir

if [ -z "$_sbe_root" ]; then
    echo "Can't find the Satellite1-ESPHome repo; source this from inside it"
    unset _sbe_root
    return 1 2>/dev/null || exit 1
fi

(
    set -e
    cd "$_sbe_root"
    venv=".venv"

    # Only report what actually changed; snapshots are compared before/after
    changed=""
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT

    # --- Submodules ---------------------------------------------------------
    if git rev-parse --git-dir >/dev/null 2>&1; then
        git submodule status --recursive > "$tmp/subs_before" 2>/dev/null || true
        git submodule --quiet update --init --recursive || { echo "Submodule update failed"; exit 1; }
        git submodule status --recursive > "$tmp/subs_after" 2>/dev/null || true
        if ! cmp -s "$tmp/subs_before" "$tmp/subs_after"; then
            echo "==> Updated git submodules:"
            # Lines are "<state><sha> <path> ...", state '-' = not checked out
            grep -vxF -f "$tmp/subs_before" "$tmp/subs_after" | awk '{ sub(/^[-+U ]/, "", $1); print "    " $2 " -> " substr($1, 1, 8) }'
            changed=1
        fi
    else
        echo "Not a git checkout; skipping submodule update"
    fi

    # --- Python -------------------------------------------------------------
    # Pick the first Python that ESPHome 2026.x supports (3.12-3.14)
    PYTHON=""
    for p in python3.14 python3.13 python3.12; do
        if command -v "$p" >/dev/null 2>&1; then PYTHON="$p"; break; fi
    done
    if [ -z "$PYTHON" ]; then
        echo "Need Python 3.12-3.14 (as python3.12, python3.13 or python3.14). Install it with:"
        case "$(uname -s)" in
            Darwin)
                echo "    brew install python@3.12" ;;
            Linux)
                if command -v apt-get >/dev/null 2>&1; then
                    echo "    sudo apt install python3.12 python3.12-venv"
                    echo "(Ubuntu 22.04 and Debian 12 don't package 3.12; use pyenv or, on Ubuntu, ppa:deadsnakes/ppa)"
                elif command -v dnf >/dev/null 2>&1; then
                    echo "    sudo dnf install python3.12"
                elif command -v pacman >/dev/null 2>&1; then
                    echo "    sudo pacman -S python"
                elif command -v zypper >/dev/null 2>&1; then
                    echo "    sudo zypper install python312"
                else
                    echo "    your distribution's package manager, or pyenv"
                fi ;;
            *)
                echo "    https://www.python.org/downloads/" ;;
        esac
        exit 1
    fi

    # Rebuild the venv if it exists but was made with an unsupported Python
    if [ -d "${venv}" ] && ! "${venv}/bin/python" -c 'import sys; sys.exit(not (3,12) <= sys.version_info[:2] < (3,15))' 2>/dev/null; then
        echo "==> Existing ${venv} uses an unsupported Python, recreating"
        rm -rf "${venv}"
    fi

    # Rebuild the venv if an earlier creation failed part way (e.g. Debian/Ubuntu
    # without python3.X-venv leaves a .venv with no activate script or pip)
    if [ -d "${venv}" ] && { [ ! -f "${venv}/bin/activate" ] || ! "${venv}/bin/python" -m pip --version >/dev/null 2>&1; }; then
        echo "==> Existing ${venv} is incomplete, recreating"
        rm -rf "${venv}"
    fi

    # Rebuild the venv if it was created at another path (repo moved or
    # renamed): activate would put a missing bin/ on PATH and its scripts'
    # shebangs would point at a Python that no longer exists
    # (Python quotes the recorded path differently by version, so strip quotes;
    # if it can't be read at all, leave the venv alone)
    venv_path="$(pwd -P)/${venv}"
    recorded=""
    if [ -f "${venv}/bin/activate" ]; then
        recorded=$(sed -n 's/^[[:space:]]*export VIRTUAL_ENV=//p' "${venv}/bin/activate" | tail -n 1 | tr -d "\"'")
    fi
    if [ -n "${recorded}" ] && [ "${recorded}" != "${venv_path}" ]; then
        echo "==> Existing ${venv} was created at another path, recreating"
        rm -rf "${venv}"
    fi

    fresh=""
    if [ ! -d "${venv}" ]; then
        echo "==> Creating virtual environment at ${venv} with ${PYTHON} (first install takes a few minutes)"
        if ! "${PYTHON}" -m venv "${venv}"; then
            # Don't leave a half-built venv behind for the next run to trip on
            rm -rf "${venv}"
            echo "Creating the virtual environment failed"
            if command -v apt-get >/dev/null 2>&1; then
                echo "On Debian/Ubuntu, install venv support with: sudo apt install ${PYTHON}-venv"
            fi
            exit 1
        fi
        fresh=1
        changed=1
    fi

    # --- Dependencies -------------------------------------------------------
    pip() { "${venv}/bin/python" -m pip --disable-pip-version-check "$@"; }
    pip freeze --all 2>/dev/null | sort > "$tmp/pkgs_before"
    pip install --quiet --upgrade pip
    pip install --quiet -r requirements.txt
    pip freeze --all 2>/dev/null | sort > "$tmp/pkgs_after"

    if [ -n "$fresh" ]; then
        echo "==> Installed $(wc -l < "$tmp/pkgs_after" | tr -d ' ') Python packages"
    elif ! cmp -s "$tmp/pkgs_before" "$tmp/pkgs_after"; then
        echo "==> Python packages changed:"
        # freeze lines are "name==version"; match names case-insensitively
        awk -F'==' '
            NR == FNR { old[tolower($1)] = $2; next }
            { k = tolower($1); new[k] = $2; name[k] = $1 }
            END {
                for (k in new)
                    if (!(k in old))          printf "    + %s %s\n", name[k], new[k]
                    else if (old[k] != new[k]) printf "    ~ %s %s -> %s\n", name[k], old[k], new[k]
                for (k in old)
                    if (!(k in new))          printf "    - %s %s\n", k, old[k]
            }' "$tmp/pkgs_before" "$tmp/pkgs_after" | sort -k2
        changed=1
    fi

    [ -n "$changed" ] || echo "==> Already up to date, nothing changed"
)
_sbe_status=$?

if [ "$_sbe_status" -eq 0 ]; then
    . "$_sbe_root/.venv/bin/activate"
    echo "==> Build environment active in this shell (run deactivate to leave)"
else
    echo "Setup failed (exit $_sbe_status)"
fi

unset _sbe_root
if [ "$_sbe_status" -eq 0 ]; then unset _sbe_status; true; else unset _sbe_status; false; fi
