#!/usr/bin/env bash
#
# Deploy the glider tree to the board's filesystem as PRECOMPILED .mpy.
#
# Pipeline: collect -> gate+compile (host only) -> wipe board -> push. Nothing touches the board
# until the whole tree has passed ruff and mpy-cross, so a syntax error can never leave a half-built
# firmware on the device.
#
# WHY .mpy (7/27): this script used to compile to a throwaway path and push the .py, so the board
# recompiled every module at boot -- heap churn and fragmentation at exactly the moment the flight
# code is about to switch GC off.
#
# WHY THE WIPE: a renamed or deleted module used to linger on the board forever, and a hand-copied
# file could silently shadow a newer one (a stale hitl_run.py cost a debugging cycle). Wipe-then-push
# makes the board's contents a function of the source tree alone. What is SPARED matters:
# launch.config is the operator's launch configuration and *.creds are the wifi secrets -- runtime
# state that exists only on the board -- so root is pruned by EXTENSION and never wholesale.
#
# WHY main.py STAYS SOURCE: measured, not assumed. With only main.mpy present the board boots to a
# bare REPL -- the runtime looks for that literal filename. Compiling it would silently stop the
# firmware from starting.
#
# THE FPU CAVEAT: mpy-cross -march=rv32imc has no hardware FPU, so @micropython.native float
# functions come out soft-float -- numerically identical, just slower. test/test_native_gate.py
# compares native/viper against a bytecode baseline and is part of `make test`.
#
# THE single deploy path: no second copy, no symlink. Two deploy scripts disagreeing about what the
# board should contain is how modules go stale on the device, which has bitten this project
# repeatedly. Everything needing a deploy calls THIS with params (see src/glider/test/run_tests.sh).
#
# Usage: tools/deploy.sh [file ...]   # default: every module + packages + *.creds + test/*.py
# Env:   PORT (default /dev/ttyACM0)

set -u

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$TOOLS/.." && pwd)"
GLIDER="$ROOT/src/glider"
PORT="${PORT:-/dev/ttyACM0}"
RETRIES=5           # a deployed main.py auto-runs on reset, so the board is often busy: every
RETRY_WAIT=1.5      # mpremote step races the raw REPL and must be retried, not just attempted

# board-run operator harnesses that live in tools/ rather than in the firmware tree
HARNESSES=(hitl_run attitude_soak oom_soak wind_soak launch_config)

if [ -t 1 ]; then G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; N=$'\e[0m'; else G=; R=; Y=; N=; fi

die()  { echo "${R}$*${N}" >&2; exit 1; }
warn() { echo "${Y}$*${N}" >&2; }

# ---------------------------------------------------------------- toolchain

check_tools() {
    command -v mpremote >/dev/null || die "mpremote not found"
    have_ruff=1; command -v ruff >/dev/null || { have_ruff=0; warn "warning: ruff not found"; }
    # prefer the repo-built mpy-cross: the stock one lacks the RV32 native emitter and errors
    # ("invalid arch") on @micropython.viper modules
    MPYX="$TOOLS/mpy-cross.v1.29.0"; MARCH=(-march=rv32imc)
    if [ ! -x "$MPYX" ]; then
        command -v mpy-cross >/dev/null || die "mpy-cross not found -- cannot build .mpy"
        MPYX=mpy-cross; MARCH=()
        warn "warning: using PATH mpy-cross, not the repo-built rv32imc build"
    fi
}

# Run one mpremote step until it wins the raw-REPL race. $1 = description, rest = mpremote args.
board_do() {
    local what="$1"; shift
    local attempt
    for attempt in $(seq "$RETRIES"); do
        mpremote connect "$PORT" "$@" >"$tmp/mpremote.out" 2>&1 && return 0
        sleep "$RETRY_WAIT"
    done
    warn "$what failed after $RETRIES attempts:"; sed 's/^/  /' "$tmp/mpremote.out" | tail -15 >&2
    return 1
}

# ---------------------------------------------------------------- collect

# Stamp the firmware version as YYYY.MM.DD.commit (commit date, so the same commit -> the same
# version) for config_default to report; version.py is gitignored.
stamp_version() {
    local version
    version="$(git -C "$GLIDER" show -s --date=format:'%Y.%m.%d' --format='%cd.%h' --abbrev=12 HEAD 2>/dev/null || echo dev)"
    printf "VERSION = '%s'\n" "$version" > "$GLIDER/version.py"
}

collect_files() {
    files=()
    if [ "$#" -gt 0 ]; then files=("$@"); return; fi
    local f
    for f in "$GLIDER"/*.py "$GLIDER"/*.creds;            do [ -e "$f" ] && files+=("$f"); done
    for f in "$GLIDER"/drivers/*.py "$GLIDER"/tasks/*.py; do [ -e "$f" ] && files+=("$f"); done
    for f in "$GLIDER"/test/*.py;                         do [ -e "$f" ] && files+=("$f"); done
    for f in "${HARNESSES[@]}";                           do [ -e "$TOOLS/$f.py" ] && files+=("$TOOLS/$f.py"); done
}

# ---------------------------------------------------------------- gate + compile

# Where a source file's compiled output belongs in the staging tree (empty = do not compile).
staged_target() {
    case "$1" in
        */main.py)   echo "" ;;                                        # boot entry: stays source
        */drivers/*) echo "$tmp/drivers/$(basename "${1%.py}").mpy" ;;
        */tasks/*)   echo "$tmp/tasks/$(basename "${1%.py}").mpy" ;;
        *)           echo "$tmp/$(basename "${1%.py}").mpy" ;;
    esac
}

lint() {
    [ "$have_ruff" = 1 ] || return 0
    ruff check "$1" || die "ruff failed: $1"
}

# Lint everything; compile everything that is firmware, into $tmp. Test files are staged as SOURCE:
# they run from the host (`mpremote run test/x.py`), so compiling them buys nothing.
build() {
    compiled=0
    local f out
    for f in "${files[@]}"; do
        case "$f" in
            */test/*.py) lint "$f"; cp "$f" "$tmp/test/" ;;
            *.py)
                lint "$f"
                out="$(staged_target "$f")"
                [ -n "$out" ] || continue
                "$MPYX" -O3 "${MARCH[@]}" "$f" -o "$out" 2>"$tmp/err" \
                    || { sed 's/^/ /' "$tmp/err" >&2; die "mpy-cross failed: $f"; }
                compiled=$((compiled + 1))
                ;;
        esac
    done
}

# ---------------------------------------------------------------- board

# Remove every module so a rename or delete cannot leave a stale one behind, and a leftover .py can
# never shadow the .mpy about to land. Package dirs go wholesale (pure deploy output -- this is also
# what finally clears a stray host __pycache__); root is pruned by extension so operator state
# survives. See the header.
wipe_board() {
    board_do "board wipe" exec "
import os
def rmtree(path):
    try:
        entries = os.listdir(path)
    except OSError:
        return
    for entry in entries:
        full = path + '/' + entry
        try:
            if os.stat(full)[0] & 0x4000:
                rmtree(full); os.rmdir(full)
            else:
                os.remove(full)
        except OSError:
            pass
for d in ('/drivers', '/tasks', '/test', '/__pycache__'):
    rmtree(d)
    try: os.rmdir(d)
    except OSError: pass
for entry in os.listdir('/'):
    if entry.endswith('.py') or entry.endswith('.mpy'):
        try: os.remove('/' + entry)
        except OSError: pass
" || warn "warning: wipe incomplete -- stale modules may survive"
}

# One chained mpremote session: batching keeps the deploy to a single board reset, where per-file
# copies would each relaunch main.py and race the next copy. Directories go via `cp -r`, which
# CREATES them -- a separate mkdir pass loses the raw-REPL race and leaves /test missing.
push_board() {
    local cmd=() sep= f
    add() { cmd+=($sep "$@"); sep=+; }
    add cp -r "$tmp/drivers" "$tmp/tasks" "$tmp/test" :
    for f in "$tmp"/*.mpy; do [ -e "$f" ] && add cp "$f" ":$(basename "$f")"; done
    for f in "${files[@]}"; do
        case "$f" in
            */main.py) add cp "$f" ":main.py" ;;
            *.creds)   add cp "$f" ":$(basename "$f")" ;;
        esac
    done
    board_do "push" "${cmd[@]}" || die "push failed"
}

# ---------------------------------------------------------------- main

check_tools
tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
mkdir -p "$tmp/drivers" "$tmp/tasks" "$tmp/test"

stamp_version
collect_files "$@"
build
wipe_board
push_board

echo " ${G}deployed${N} $compiled .mpy + main.py (+ test/ sources) -- board wiped first"
