#!/usr/bin/env bash
# Deploy this glider tree to the board's filesystem. Each Python file is ruff-checked and
# mpy-cross compiled (fail before touching the board); non-Python files (e.g. <ssid>.creds) are
# pushed as-is. test/*.py -> :test/. Lives in src/glider, so the source dir is the script's dir.
#
# Usage:  ./deploy.sh [file ...]      # default: every module + *.creds + test/*.py
# Env:    PORT (default /dev/ttyACM0)

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"   # absolute (handles relative invocation); = src/glider
PORT="${PORT:-/dev/ttyACM0}"

if [ -t 1 ]; then G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; N=$'\e[0m'; else G=; R=; Y=; N=; fi
command -v mpremote >/dev/null || { echo "${R}mpremote not found${N}"; exit 2; }
have_ruff=1; command -v ruff >/dev/null      || { have_ruff=0; echo "${Y}warning: ruff not found${N}"; }
have_mpy=1;  command -v mpy-cross >/dev/null || { have_mpy=0;  echo "${Y}warning: mpy-cross not found${N}"; }

tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT

# settle the board and ensure :test exists
mpremote connect "$PORT" reset >/dev/null 2>&1 || true
sleep 2
mpremote connect "$PORT" mkdir :test >/dev/null 2>&1 || true

check_and_push() {
    local file="$1" dest="$2"
    case "$file" in
        *.py)
            if [ "$have_ruff" = 1 ] && ! ruff check "$file"; then
                echo "${R}ruff failed: $dest${N}"; return 1
            fi
            if [ "$have_mpy" = 1 ] && ! mpy-cross -O3 "$file" -o "$tmp/c.mpy" 2>"$tmp/err"; then
                echo "${R}mpy-cross failed: $dest${N}"; sed 's/^/    /' "$tmp/err"; return 1
            fi
            ;;
    esac
    if mpremote connect "$PORT" cp "$file" ":$dest" >/dev/null 2>&1; then
        echo "  ${G}ok${N}  $dest"
    else
        echo "${R}push failed: $dest${N}"; return 1
    fi
}

# build the file list (args, or all modules + tests + ssid.creds)
files=()
if [ "$#" -gt 0 ]; then
    files=("$@")
else
    for f in "$HERE"/*.py "$HERE"/*.creds; do [ -e "$f" ] && files+=("$f"); done
    for f in "$HERE"/test/*.py; do [ -e "$f" ] && files+=("$f"); done
fi

fail=0
for f in "${files[@]}"; do
    case "$f" in
        */test/*) dest="test/$(basename "$f")" ;;
        *)        dest="$(basename "$f")" ;;
    esac
    check_and_push "$f" "$dest" || fail=1
done
exit $fail
