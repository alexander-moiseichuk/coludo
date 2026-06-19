#!/usr/bin/env bash
# Deploy this glider tree to the board's filesystem. Every Python file is ruff-checked and
# mpy-cross compiled first (fail before touching the board); then everything is pushed in ONE
# mpremote session. A deployed main.py auto-runs on every soft-reset, so per-file copies would each
# relaunch it and race the next copy -- batching keeps it to a single reset. drivers/ and tasks/ go
# as packages (cp -r merges, no nesting); test/*.py -> :test/; *.creds pushed as-is. Lives in
# src/glider, so the source dir is the script's dir.
#
# Usage:  ./deploy.sh [file ...]      # default: every module + packages + *.creds + test/*.py
# Env:    PORT (default /dev/ttyACM0)

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"   # absolute (handles relative invocation); = src/glider
PORT="${PORT:-/dev/ttyACM0}"

if [ -t 1 ]; then G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; N=$'\e[0m'; else G=; R=; Y=; N=; fi
command -v mpremote >/dev/null || { echo "${R}mpremote not found${N}"; exit 2; }
have_ruff=1; command -v ruff >/dev/null      || { have_ruff=0; echo "${Y}warning: ruff not found${N}"; }
have_mpy=1;  command -v mpy-cross >/dev/null || { have_mpy=0;  echo "${Y}warning: mpy-cross not found${N}"; }

tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT

# stamp the firmware version from the git commit so config_default can report it (version.py is gitignored)
printf "VERSION = '%s'\n" "$(git -C "$HERE" rev-parse --short=12 HEAD 2>/dev/null || echo dev)" > "$HERE/version.py"

# build the file list (args, or all modules + packages + tests + ssid.creds)
files=()
if [ "$#" -gt 0 ]; then
    files=("$@")
else
    for f in "$HERE"/*.py "$HERE"/*.creds; do [ -e "$f" ] && files+=("$f"); done
    for f in "$HERE"/drivers/*.py "$HERE"/tasks/*.py; do [ -e "$f" ] && files+=("$f"); done
    for f in "$HERE"/test/*.py; do [ -e "$f" ] && files+=("$f"); done
fi

# 1) gate every .py (ruff + mpy-cross) before touching the board
for f in "${files[@]}"; do
    case "$f" in
        *.py)
            if [ "$have_ruff" = 1 ] && ! ruff check "$f"; then echo "${R}ruff failed: $f${N}"; exit 1; fi
            if [ "$have_mpy" = 1 ] && ! mpy-cross -O3 "$f" -o "$tmp/c.mpy" 2>"$tmp/err"; then
                echo "${R}mpy-cross failed: $f${N}"; sed 's/^/    /' "$tmp/err"; exit 1
            fi
            ;;
    esac
done

# 2) ensure dest dirs, then push everything in one chained mpremote session (packages via cp -r)
for d in test drivers tasks; do mpremote connect "$PORT" mkdir ":$d" >/dev/null 2>&1 || true; done
cmd=(); sep=; pkg_done=
add() { cmd+=($sep "$@"); sep=+; }
for f in "${files[@]}"; do
    case "$f" in
        */test/*)              add cp "$f" ":test/$(basename "$f")" ;;
        */drivers/*|*/tasks/*) [ -z "$pkg_done" ] && { add cp -r "$HERE/drivers" "$HERE/tasks" :; pkg_done=1; } ;;
        *)                     add cp "$f" ":$(basename "$f")" ;;
    esac
done

for _ in 1 2 3 4 5; do
    if mpremote connect "$PORT" "${cmd[@]}" >/dev/null 2>&1; then
        echo "  ${G}deployed${N} ${#files[@]} files (modules + drivers/ tasks/ + test/)"; exit 0
    fi
    sleep 1.5
done
echo "${R}push failed${N}"; exit 1
