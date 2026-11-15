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
#
# The file has two halves. The wrapper runs in the caller's shell when sourced,
# so it stays tiny and only finds the repo, runs the worker and activates the
# venv. The worker does the real work in a fresh, non-interactive /bin/sh, so
# the caller's aliases, functions, shell options and CDPATH can't affect it,
# and its `set -e` holds even when the source is part of `||`, `&&` or `if`.

if [ "${_SBE_WORKER:-}" != 1 ]; then
    # ======================= wrapper (caller's shell) =======================

    # Was this file sourced, or executed as a program? Each shell has its own
    # tell; the fallback works because $0 is the shell's name when sourced.
    _sbe_sourced=""
    if [ -n "${ZSH_EVAL_CONTEXT:-}" ]; then
        case "$ZSH_EVAL_CONTEXT" in *:file*) _sbe_sourced=1 ;; esac
    elif [ -n "${BASH_SOURCE:-}" ]; then
        [ "$BASH_SOURCE" = "$0" ] || _sbe_sourced=1
    else
        case "$0" in */setup_build_env.sh | setup_build_env.sh) ;; *) _sbe_sourced=1 ;; esac
    fi

    # This file's own path, when the shell tells us (bash, zsh, or executed)
    _sbe_self="${BASH_SOURCE:-$0}"
    case "$_sbe_self" in
        */setup_build_env.sh) _sbe_self_dir="${_sbe_self%/*}/.." ;;
        setup_build_env.sh) _sbe_self_dir=".." ;;
        *) _sbe_self_dir="" ;;
    esac

    # Find the repo root: this file's own location first, then git, then the
    # current directory. `command` skips aliases/functions (e.g. a cd wrapper),
    # and CDPATH or a chpwd hook can't leak into the captured path.
    _sbe_root=""
    for _sbe_dir in "$_sbe_self_dir" "$(command git rev-parse --show-toplevel 2>/dev/null)" "$(command pwd)"; do
        if [ -n "$_sbe_dir" ] && [ -f "$_sbe_dir/scripts/setup_build_env.sh" ]; then
            _sbe_root=$(CDPATH='' command cd -P -- "$_sbe_dir" >/dev/null 2>&1 && command pwd -P)
            [ -n "$_sbe_root" ] && break
        fi
    done

    if [ -z "$_sbe_root" ]; then
        echo "Can't find the Satellite1-ESPHome repo; source this from inside it"
        _sbe_status=1
    else
        _SBE_WORKER=1 command sh "$_sbe_root/scripts/setup_build_env.sh"
        _sbe_status=$?
        if [ "$_sbe_status" -ne 0 ]; then
            echo "Setup failed (exit $_sbe_status)"
        elif [ -n "$_sbe_sourced" ]; then
            . "$_sbe_root/.venv/bin/activate"
            echo "==> Build environment active in this shell (run deactivate to leave)"
        else
            echo "==> Build environment ready. This was run, not sourced, so it isn't active"
            echo "    in your shell. To activate it, run:  source scripts/setup_build_env.sh"
        fi
    fi

    if [ -n "$_sbe_sourced" ]; then
        unset _sbe_sourced _sbe_self _sbe_self_dir _sbe_dir _sbe_root
        if [ "$_sbe_status" -eq 0 ]; then unset _sbe_status; return 0; fi
        unset _sbe_status
        return 1
    fi
    exit "$_sbe_status"
fi

# ========================= worker (fresh /bin/sh) =========================
set -e
unset CDPATH
cd "$(dirname -- "$0")/.." || exit 1
venv=".venv"

# The Python versions ESPHome 2026.x supports, as a check any Python can run
py_ok='import sys; sys.exit(not (3, 12) <= sys.version_info[:2] < (3, 15))'

# Only report what actually changed; snapshots are compared before/after
changed=""
tmp=$(mktemp -d) || { echo "Can't create a temporary directory"; exit 1; }
trap 'rm -rf "$tmp"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

venv_pip() { "${venv}/bin/python" -m pip --disable-pip-version-check "$@"; }

# --- Submodules -------------------------------------------------------------
# Fetch submodules that aren't checked out and move clean ones to the commit
# the repo records. A submodule someone is working in (on a branch, or with
# local changes) is left alone with a warning rather than reset.
if ! git rev-parse --git-dir >/dev/null 2>&1; then
    echo "Not a git checkout; skipping submodule update"
elif [ -f .gitmodules ]; then
    git submodule status --recursive > "$tmp/subs_before" 2>/dev/null || true
    git config -f .gitmodules --get-regexp '^submodule\..*\.path$' 2>/dev/null \
        | sed 's/^[^ ]* //' > "$tmp/sub_paths" || true
    while IFS= read -r sub; do
        [ -n "$sub" ] || continue
        if [ ! -e "$sub/.git" ]; then
            # Not checked out yet: the build needs it, so failing here is fatal
            git submodule --quiet update --init --recursive -- "$sub" \
                || { echo "Fetching submodule $sub failed"; exit 1; }
            continue
        fi
        want=$(git ls-files -s -- "$sub" | awk '{ print $2 }')
        have=$(git -C "$sub" rev-parse HEAD 2>/dev/null || true)
        [ "$want" != "$have" ] || continue
        if git -C "$sub" symbolic-ref -q HEAD >/dev/null 2>&1 || [ -n "$(git -C "$sub" status --porcelain 2>/dev/null)" ]; then
            echo "==> Leaving submodule $sub alone (you're on a branch or have local changes there);"
            echo "    the repo expects $(printf '%.8s' "$want"); to switch to it, run: git submodule update -- $sub"
            continue
        fi
        git submodule --quiet update --init --recursive -- "$sub" \
            || echo "==> Couldn't update submodule $sub; using the version already checked out"
    done < "$tmp/sub_paths"
    git submodule status --recursive > "$tmp/subs_after" 2>/dev/null || true
    if ! cmp -s "$tmp/subs_before" "$tmp/subs_after"; then
        echo "==> Updated git submodules:"
        # Lines are "<state><sha> <path> ...", state '-' = not checked out
        grep -vxF -f "$tmp/subs_before" "$tmp/subs_after" | awk '{ sub(/^[-+U ]/, "", $1); print "    " $2 " -> " substr($1, 1, 8) }' || true
        changed=1
    fi
fi

# --- Python -----------------------------------------------------------------
# Pick the first supported Python that actually runs (a pyenv shim resolves on
# PATH even when its version isn't active, then fails when called)
PYTHON=""
for p in python3.14 python3.13 python3.12 python3; do
    if command -v "$p" >/dev/null 2>&1 && "$p" -c "$py_ok" >/dev/null 2>&1; then
        PYTHON="$p"
        break
    fi
done
if [ -z "$PYTHON" ]; then
    echo "Need Python 3.12-3.14 (as python3.12, python3.13, python3.14 or python3). Install it with:"
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
if [ -d "${venv}" ] && ! "${venv}/bin/python" -c "$py_ok" 2>/dev/null; then
    echo "==> Existing ${venv} uses an unsupported Python, recreating"
    rm -rf "${venv}"
fi

# Rebuild the venv if an earlier creation failed part way (e.g. Debian/Ubuntu
# without python3.X-venv leaves a .venv with no activate script)
if [ -d "${venv}" ] && [ ! -f "${venv}/bin/activate" ]; then
    echo "==> Existing ${venv} is incomplete, recreating"
    rm -rf "${venv}"
fi

# A venv without pip is fine (uv creates them that way); add pip rather than
# throwing the venv and everything installed in it away
if [ -d "${venv}" ] && ! venv_pip --version >/dev/null 2>&1; then
    echo "==> Existing ${venv} has no pip, adding it"
    if "${venv}/bin/python" -m ensurepip --upgrade >/dev/null 2>&1; then
        changed=1
    else
        echo "==> Couldn't add pip to ${venv}, recreating"
        rm -rf "${venv}"
    fi
fi

# Rebuild the venv if it was created at another path (repo moved or renamed):
# activate would put a missing bin/ on PATH and its scripts' shebangs would
# point at a Python that no longer exists. Python writes the path shell-quoted
# (and differently by version), so decode it with shlex rather than by hand;
# if it can't be read at all, leave the venv alone.
venv_path="$(pwd -P)/${venv}"
recorded=""
if [ -f "${venv}/bin/activate" ]; then
    raw=$(sed -n -e 's/^[[:space:]]*export VIRTUAL_ENV=//p' -e 's/^[[:space:]]*VIRTUAL_ENV=//p' "${venv}/bin/activate" \
        | grep -v '\$' | tail -n 1)
    if [ -n "$raw" ]; then
        recorded=$("$PYTHON" -c 'import shlex, sys
try:
    words = shlex.split(sys.argv[1])
except ValueError:
    words = []
print(words[0] if len(words) == 1 else "")' "$raw" 2>/dev/null || true)
    fi
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

# --- Dependencies -----------------------------------------------------------
venv_pip freeze --all 2>/dev/null | sort > "$tmp/pkgs_before" || true
# Only a fresh venv needs a newer pip; upgrading on every run would need the
# network even when nothing else changed
if [ -n "$fresh" ]; then
    venv_pip install --quiet --upgrade pip || { echo "Upgrading pip failed"; exit 1; }
fi
venv_pip install --quiet -r requirements.txt || { echo "Installing requirements.txt failed"; exit 1; }
venv_pip freeze --all 2>/dev/null | sort > "$tmp/pkgs_after" || true

if [ -n "$fresh" ]; then
    echo "==> Installed $(wc -l < "$tmp/pkgs_after" | tr -d ' ') Python packages"
elif ! cmp -s "$tmp/pkgs_before" "$tmp/pkgs_after"; then
    echo "==> Python packages changed:"
    # freeze lines are "name==version"; match names case-insensitively
    awk -F'==' '
        NR == FNR { old[tolower($1)] = $2; name[tolower($1)] = $1; next }
        { k = tolower($1); new[k] = $2; name[k] = $1 }
        END {
            for (k in new)
                if (!(k in old))          printf "    + %s %s\n", name[k], new[k]
                else if (old[k] != new[k]) printf "    ~ %s %s -> %s\n", name[k], old[k], new[k]
            for (k in old)
                if (!(k in new))          printf "    - %s %s\n", name[k], old[k]
        }' "$tmp/pkgs_before" "$tmp/pkgs_after" | sort -k2
    changed=1
fi

[ -n "$changed" ] || echo "==> Already up to date, nothing changed"
